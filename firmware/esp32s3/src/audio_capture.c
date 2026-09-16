/** =================================================================*
 * @file   audio_capture.c
 * @brief  音声キャプチャ
 * ================================================================= */
#include "audio_capture.h"                                  /* 音声キャプチャAPI */

#include "acoustic_protocol.h"                              /* 特徴量イベント長とbin数 */
#include "app_config.h"                                     /* I2S接続とタスク設定 */
#include "driver/i2s_std.h"                                 /* ESP-IDF標準I2S API */
#include "esp_attr.h"                                       /* IRAM配置属性 */
#include "esp_check.h"                                      /* ESP-IDFエラー検査マクロ */
#include "esp_log.h"                                        /* ESP-IDFログタグ型 */
#include "esp_timer.h"                                      /* 単調時刻取得API */
#include "freertos/FreeRTOS.h"                              /* FreeRTOS基本型 */
#include "freertos/task.h"                                  /* FreeRTOSタスクAPI */
#include "log_mel_extractor.h"                              /* ストリーミングlog-mel抽出 */
#include <limits.h>                                         /* 整数型の最小値 */
#include <math.h>                                           /* 音量計算API */
#include <stddef.h>                                         /* size_t */
#include <string.h>                                         /* 特徴量リングコピー */

static char const * const TAG = "audio_capture";            /**< ESP-IDFログ識別子 */
static i2s_chan_handle_t s_rx_channel;                      /**< XVF3800音声入力I2Sチャネル */
static log_mel_extractor_t s_log_mel_extractor;             /**< log-mel係数、繰越し、作業領域 */
/**< 新しい順に上書きする800 ms特徴量リング */
static int8_t s_feature_ring[ACOUSTIC_FEATURE_EVENT_FRAME_COUNT][ACOUSTIC_FEATURE_BIN_COUNT];
static size_t s_feature_write_index;                        /**< 次回特徴量書込位置 */
static uint32_t s_generated_feature_frame_count;            /**< 累積特徴量フレーム数 */
static audio_capture_feature_event_t s_feature_event;       /**< 送信待ちまたは収集中の特徴量イベント */
static uint16_t s_feature_event_frame_count;                /**< イベントに退避済みのフレーム数 */
static bool s_feature_event_active;                         /**< 後続500 msを収集中 */
static bool s_feature_event_ready;                          /**< 80フレーム揃い、送信側が取得可能 */
/**< 音声状態を保護する排他ロック */
static portMUX_TYPE s_snapshot_lock = portMUX_INITIALIZER_UNLOCKED;
/**< 音声タスクと参照元で共有する最新状態 */
static audio_capture_snapshot_t s_snapshot = {
    .level_dbfs_x100 = INT16_MIN,
    .peak_dbfs_x100 = INT16_MIN,
};

static int16_t amplitude_to_dbfs_x100(double amplitude);    /* 振幅をdBFS x100へ変換 */
static void audio_capture_feature_callback(int8_t const frame[LOG_MEL_BIN_COUNT], void * context); /* 特徴量保持 */
/* I2S受信オーバーラン通知 */
static bool audio_capture_overrun_callback(i2s_chan_handle_t handle, i2s_event_data_t * event, void * user_context);
static void audio_capture_task(void * context);             /* 音量測定タスク */

/** =================================================================*
 * @brief  振幅dBFS変換
 * @param[in] amplitude 正規化済み振幅
 * @return dBFS x100値。無音はINT16_MIN
 * ================================================================= */
static int16_t amplitude_to_dbfs_x100(double amplitude) {
    double const minimum_amplitude = 1.0 / (double) INT32_MAX;

    if (amplitude < minimum_amplitude) {
        return INT16_MIN;
    }

    double dbfs_x100 = 2000.0 * log10(amplitude);
    if (dbfs_x100 < (double) INT16_MIN) {
        dbfs_x100 = (double) INT16_MIN;
    } else if (dbfs_x100 > 0.0) {
        dbfs_x100 = 0.0;
    }

    return (int16_t) lround(dbfs_x100);
}

/** =================================================================*
 * @brief  特徴量リング更新
 * @param[in] frame 10 ms周期の32 bin int8 log-mel
 * @param[in] context 未使用
 * @details 音声キャプチャタスクだけから呼び出し、直近800 msを常時保持する。
 * ================================================================= */
static void audio_capture_feature_callback(int8_t const frame[LOG_MEL_BIN_COUNT], void * context) {
    (void) context;

    portENTER_CRITICAL(&s_snapshot_lock);
    memcpy(s_feature_ring[s_feature_write_index], frame, sizeof(s_feature_ring[0]));
    s_feature_write_index = (s_feature_write_index + 1U) % ACOUSTIC_FEATURE_EVENT_FRAME_COUNT;
    s_generated_feature_frame_count++;
    if (s_feature_event_active) {
        memcpy(s_feature_event.frames[s_feature_event_frame_count], frame, sizeof(s_feature_event.frames[0]));
        s_feature_event_frame_count++;
        if (ACOUSTIC_FEATURE_EVENT_FRAME_COUNT == s_feature_event_frame_count) {
            s_feature_event_active = false;
            s_feature_event_ready = true;
        }
    }
    portEXIT_CRITICAL(&s_snapshot_lock);
}

/** =================================================================*
 * @brief  I2S受信オーバーラン通知
 * @param[in] handle I2Sチャネル
 * @param[in] event I2Sイベント情報
 * @param[in] user_context 登録時のユーザーコンテキスト
 * @return 高優先度タスクの起床要求有無
 * ================================================================= */
static bool IRAM_ATTR audio_capture_overrun_callback(i2s_chan_handle_t handle, i2s_event_data_t * event,
                                                     void * user_context) {
    (void) handle;
    (void) event;
    (void) user_context;

    portENTER_CRITICAL_ISR(&s_snapshot_lock);
    s_snapshot.overrun_count++;
    portEXIT_CRITICAL_ISR(&s_snapshot_lock);
    return false;
}

/** =================================================================*
 * @brief  音量測定タスク
 * @param[in] context FreeRTOSタスク引数
 * @details XVF3800のI2S出力からRMSとピーク音量を周期的に更新する。
 * ================================================================= */
static void audio_capture_task(void * context) {
    (void) context;

    int32_t samples[APP_AUDIO_BLOCK_FRAMES * APP_AUDIO_CHANNEL_COUNT];
    int32_t mono_samples[APP_AUDIO_BLOCK_FRAMES];
    double filtered_rms = 0.0;
    double filtered_peak = 0.0;

    while (true) {
        size_t bytes_read = 0U;
        esp_err_t const err = i2s_channel_read(s_rx_channel, samples, sizeof(samples), &bytes_read, portMAX_DELAY);

        if ((err != ESP_OK) || (bytes_read == 0U)) {
            portENTER_CRITICAL(&s_snapshot_lock);
            s_snapshot.valid = false;
            portEXIT_CRITICAL(&s_snapshot_lock);
            continue;
        }

        size_t const sample_count = bytes_read / sizeof(samples[0]);
        size_t const audio_frame_count = sample_count / APP_AUDIO_CHANNEL_COUNT;
        double square_sum = 0.0;
        double block_peak = 0.0;

        for (size_t index = 0U; index < sample_count; index++) {
            double const normalized = (double) samples[index] / (double) INT32_MAX;
            double const magnitude = fabs(normalized);
            square_sum += normalized * normalized;
            if (magnitude > block_peak) {
                block_peak = magnitude;
            }
        }

        for (size_t index = 0U; index < audio_frame_count; index++) {
            mono_samples[index] = samples[index * APP_AUDIO_CHANNEL_COUNT];
        }
        int64_t const log_mel_started_us = esp_timer_get_time();
        size_t const generated_features =
            log_mel_extractor_feed(&s_log_mel_extractor, mono_samples, audio_frame_count,
                                   audio_capture_feature_callback, NULL);
        uint32_t const log_mel_elapsed_us = (uint32_t) (esp_timer_get_time() - log_mel_started_us);

        double const block_rms = sqrt(square_sum / (double) sample_count);
        if (!s_snapshot.valid) {
            filtered_rms = block_rms;
            filtered_peak = block_peak;
        } else {
            filtered_rms += 0.25 * (block_rms - filtered_rms);
            filtered_peak *= 0.90;
            if (block_peak > filtered_peak) {
                filtered_peak = block_peak;
            }
        }

        portENTER_CRITICAL(&s_snapshot_lock);
        s_snapshot.level_dbfs_x100 = amplitude_to_dbfs_x100(filtered_rms);
        s_snapshot.peak_dbfs_x100 = amplitude_to_dbfs_x100(filtered_peak);
        s_snapshot.frame_count += (uint32_t) audio_frame_count;
        s_snapshot.feature_frame_count = s_generated_feature_frame_count;
        s_snapshot.feature_ring_frames = (uint16_t) ((s_generated_feature_frame_count <
                                                       ACOUSTIC_FEATURE_EVENT_FRAME_COUNT)
                                                          ? s_generated_feature_frame_count
                                                          : ACOUSTIC_FEATURE_EVENT_FRAME_COUNT);
        if (generated_features > 0U) {
            s_snapshot.log_mel_block_last_us = log_mel_elapsed_us;
            if (log_mel_elapsed_us > s_snapshot.log_mel_block_max_us) {
                s_snapshot.log_mel_block_max_us = log_mel_elapsed_us;
            }
        }
        s_snapshot.captured_at_ms = (uint32_t) ((uint64_t) esp_timer_get_time() / 1000ULL);
        s_snapshot.valid = true;
        portEXIT_CRITICAL(&s_snapshot_lock);
    }
}

/** =================================================================*
 * @brief  特徴量イベント収集を開始
 * @param[in] event_id フロントエンドが採番したイベントID
 * @return 開始結果。300 msの履歴不足または前イベント未消費時は失敗
 * @details 収集開始時の最新30フレームを退避し、残る50フレームはI2Sタスクの
 *          log-melコールバックで収集する。リングの上書きに依存しない。
 * ================================================================= */
esp_err_t audio_capture_feature_event_start(uint16_t event_id) {
    uint32_t const pre_trigger_frames = APP_FEATURE_PRE_TRIGGER_FRAMES;

    portENTER_CRITICAL(&s_snapshot_lock);
    if (s_feature_event_active || s_feature_event_ready) {
        portEXIT_CRITICAL(&s_snapshot_lock);
        return ESP_ERR_INVALID_STATE;
    }
    if (s_generated_feature_frame_count < pre_trigger_frames) {
        portEXIT_CRITICAL(&s_snapshot_lock);
        return ESP_ERR_INVALID_SIZE;
    }

    uint32_t const first_frame = s_generated_feature_frame_count - pre_trigger_frames;
    for (uint32_t frame_index = 0U; frame_index < pre_trigger_frames; frame_index++) {
        size_t const ring_index =
            (size_t) ((first_frame + frame_index) % ACOUSTIC_FEATURE_EVENT_FRAME_COUNT);
        memcpy(s_feature_event.frames[frame_index], s_feature_ring[ring_index], sizeof(s_feature_event.frames[0]));
    }
    s_feature_event.event_id = event_id;
    s_feature_event_frame_count = (uint16_t) pre_trigger_frames;
    s_feature_event_active = true;
    portEXIT_CRITICAL(&s_snapshot_lock);
    return ESP_OK;
}

/** =================================================================*
 * @brief  完成した特徴量イベントを取得
 * @param[out] event 取得先
 * @return 完成済みイベントを取得できた場合true
 * @details ready状態では音声タスクがイベント領域へ書き込まないため、状態だけを
 *          ロックで確保してから大きなコピーを行う。
 * ================================================================= */
bool audio_capture_feature_event_take(audio_capture_feature_event_t * event) {
    if (event == NULL) {
        return false;
    }

    portENTER_CRITICAL(&s_snapshot_lock);
    if (!s_feature_event_ready) {
        portEXIT_CRITICAL(&s_snapshot_lock);
        return false;
    }
    s_feature_event_ready = false;
    portEXIT_CRITICAL(&s_snapshot_lock);

    memcpy(event, &s_feature_event, sizeof(*event));
    return true;
}

/** =================================================================*
 * @brief  特徴量イベントを破棄
 * @details USB再接続後に古いセッションのイベントを送らないために使う。
 * ================================================================= */
void audio_capture_feature_event_discard(void) {
    portENTER_CRITICAL(&s_snapshot_lock);
    s_feature_event_frame_count = 0U;
    s_feature_event_active = false;
    s_feature_event_ready = false;
    portEXIT_CRITICAL(&s_snapshot_lock);
}

/** =================================================================*
 * @brief  音声キャプチャ開始
 * @return I2S初期化またはタスク生成結果
 * ================================================================= */
esp_err_t audio_capture_start(void) {
    _Static_assert(LOG_MEL_SAMPLE_RATE_HZ == APP_AUDIO_SAMPLE_RATE_HZ, "log-mel sample rate mismatch");
    _Static_assert(LOG_MEL_BIN_COUNT == ACOUSTIC_FEATURE_BIN_COUNT, "log-mel bin count mismatch");
    _Static_assert((APP_FEATURE_PRE_TRIGGER_FRAMES + APP_FEATURE_POST_TRIGGER_FRAMES) ==
                       ACOUSTIC_FEATURE_EVENT_FRAME_COUNT,
                   "feature event duration mismatch");

    log_mel_extractor_init(&s_log_mel_extractor);
    s_snapshot.log_mel_self_test_pass = log_mel_extractor_self_test(&s_log_mel_extractor);
    i2s_chan_config_t const channel_config = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
    ESP_RETURN_ON_ERROR(i2s_new_channel(&channel_config, NULL, &s_rx_channel), TAG, "I2S RX channel creation failed");

    i2s_std_config_t const standard_config = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(APP_AUDIO_SAMPLE_RATE_HZ),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg =
            {
                .mclk = I2S_GPIO_UNUSED,
                .bclk = APP_XVF_I2S_BCLK_GPIO,
                .ws = APP_XVF_I2S_WS_GPIO,
                .dout = APP_XVF_I2S_DOUT_GPIO,
                .din = APP_XVF_I2S_DIN_GPIO,
                .invert_flags =
                    {
                        .mclk_inv = false,
                        .bclk_inv = false,
                        .ws_inv = false,
                    },
            },
    };

    ESP_RETURN_ON_ERROR(i2s_channel_init_std_mode(s_rx_channel, &standard_config), TAG,
                        "I2S standard mode initialization failed");

    i2s_event_callbacks_t const callbacks = {
        .on_recv = NULL,
        .on_recv_q_ovf = audio_capture_overrun_callback,
        .on_sent = NULL,
        .on_send_q_ovf = NULL,
    };
    ESP_RETURN_ON_ERROR(i2s_channel_register_event_callback(s_rx_channel, &callbacks, NULL), TAG,
                        "I2S callback registration failed");
    ESP_RETURN_ON_ERROR(i2s_channel_enable(s_rx_channel), TAG, "I2S RX channel enable failed");

    BaseType_t const result = xTaskCreate(audio_capture_task, "audio_capture", APP_AUDIO_TASK_STACK_SIZE, NULL,
                                          APP_AUDIO_TASK_PRIORITY, NULL);
    if (result != pdPASS) {
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

/** =================================================================*
 * @brief  音声状態取得
 * @param[out] snapshot 最新の音声キャプチャ状態
 * ================================================================= */
void audio_capture_get_snapshot(audio_capture_snapshot_t * snapshot) {
    if (snapshot == NULL) {
        return;
    }

    portENTER_CRITICAL(&s_snapshot_lock);
    *snapshot = s_snapshot;
    portEXIT_CRITICAL(&s_snapshot_lock);

    if (!snapshot->valid) {
        snapshot->level_dbfs_x100 = INT16_MIN;
        snapshot->peak_dbfs_x100 = INT16_MIN;
    }
}
