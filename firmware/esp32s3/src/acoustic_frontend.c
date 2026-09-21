/** =================================================================*
 * @file   acoustic_frontend.c
 * @brief  音響観測フロントエンド
 * ================================================================= */
#include "acoustic_frontend.h"                              /* 音響観測フロントエンドAPI */

#include "acoustic_protocol.h"                              /* CPU0との音響通信プロトコル */
#include "app_config.h"                                     /* 観測周期とタスク設定 */
#include "audio_capture.h"                                  /* 音量取得API */
#include "esp_random.h"                                     /* 起動ID生成API */
#include "esp_timer.h"                                      /* 単調時刻取得API */
#include "freertos/FreeRTOS.h"                              /* FreeRTOS基本型 */
#include "freertos/task.h"                                  /* FreeRTOSタスクAPI */
#include "usb_link.h"                                       /* CPU0向けUSB CDC通信API */
#include "wifi_telemetry.h"                                 /* Wi-Fi接続状態取得API */
#include "xvf3800_control.h"                                /* XVF3800到来方向取得API */
#include <limits.h>                                         /* 整数型の最小値 */
#include <math.h>                                           /* 循環平均と角度変換 */
#include <stdbool.h>                                        /* 真偽値 */
#include <stddef.h>                                         /* size_t */
#include <stdint.h>                                         /* 固定幅整数型 */
#include <string.h>                                         /* 特徴量パケットへのコピー */

#define FRONTEND_FIRMWARE_MAJOR            (1U)
#define FRONTEND_FIRMWARE_MINOR            (1U)
#define FRONTEND_FIRMWARE_PATCH            (0U)
#define FRONTEND_PI                        (3.14159265358979323846F)
#define FRONTEND_DEG_TO_RAD(deg)           ((deg) * (FRONTEND_PI / 180.0F))
#define FRONTEND_RAD_TO_DEG(rad)           ((rad) * (180.0F / FRONTEND_PI))

static uint32_t s_sequence;                                 /**< USB送信フレーム連番 */
static uint32_t s_observation_sequence;                     /**< DoA観測専用連番 */
static uint32_t s_boot_id;                                  /**< 起動ごとのランダムID */
static uint32_t s_i2c_error_count;                          /**< XVF3800通信失敗累積数 */
static uint32_t s_previous_overrun_count;                   /**< 前回観測時のI2Sオーバーラン数 */
static uint16_t s_next_feature_event_id;                    /**< 次に収集する特徴量イベントID */
/**< 有効な到来方向の取得済み状態 */
static acoustic_xvf_status_t s_xvf_status = ACOUSTIC_XVF_STATUS_STARTING;
static bool s_has_valid_doa;                                /**< 起動後に有効な到来方向を得た状態 */

volatile uint32_t g_acoustic_frontend_read_count;           /**< XVF3800読出し回数 */
volatile uint32_t g_acoustic_frontend_read_error_count;     /**< XVF3800読出し失敗回数 */
volatile uint32_t g_acoustic_frontend_tx_count;             /**< DoA観測送信成功回数 */
volatile uint32_t g_acoustic_frontend_tx_error_count;       /**< DoA観測送信失敗回数 */
volatile uint32_t g_acoustic_frontend_last_period_ms;       /**< 直近DoA読出し周期[ms] */
volatile uint32_t g_acoustic_frontend_min_period_ms;        /**< 最小DoA読出し周期[ms] */
volatile uint32_t g_acoustic_frontend_max_period_ms;        /**< 最大DoA読出し周期[ms] */
volatile uint32_t g_acoustic_frontend_processing_us;        /**< 直近DoA処理時間[us] */
volatile uint32_t g_acoustic_frontend_deadline_miss_count;  /**< DoA処理期限超過回数 */
volatile uint32_t g_acoustic_frontend_last_sample_ms;       /**< 直近DoA観測時刻[ms] */
volatile uint32_t g_acoustic_frontend_observation_sequence; /**< 直近DoA観測sequence */
volatile uint16_t g_acoustic_frontend_raw_doa_deg;          /**< 直近XVF3800 DoA[deg] */
volatile uint16_t g_acoustic_frontend_filtered_doa_deg;     /**< 循環平均DoA[deg] */
volatile uint8_t g_acoustic_frontend_doa_confidence;        /**< DoA品質[0..100] */

typedef struct {
    uint16_t samples_deg[APP_DOA_FILTER_WINDOW];            /**< 新しい順に上書きするDoA履歴 */
    uint8_t count;                                          /**< 有効履歴数 */
    uint8_t next;                                           /**< 次回上書き位置 */
} frontend_doa_filter_t;

static frontend_doa_filter_t s_doa_filter;                  /**< ESP32S3側の循環DoAフィルタ */

typedef struct {
    audio_capture_feature_event_t event;                    /**< 送信中イベントの固定80フレーム */
    uint16_t next_frame_index;                              /**< 次に送るイベント内フレーム位置 */
    uint32_t next_packet_at_ms;                             /**< 次の2フレームパケット送信時刻 */
    bool active;                                            /**< USBへ送信中 */
} frontend_feature_burst_t;

static frontend_feature_burst_t s_feature_burst;            /**< 送信中イベント。4 KiBのタスクスタックを圧迫しない */

static uint32_t frontend_uptime_ms(void);                   /* 起動からの経過時刻取得 */
static void frontend_doa_filter_update(xvf3800_doa_result_t const * doa, uint16_t * filtered_deg,
                                       uint8_t * confidence); /* 循環平均と品質算出 */
static esp_err_t frontend_send_frame(uint8_t const * frame, size_t length); /* 音響プロトコルフレーム送信 */
static esp_err_t frontend_send_hello(void);                 /* 起動情報送信 */
/* 音響観測結果送信 */
static esp_err_t frontend_send_observation(xvf3800_doa_result_t const * doa, bool i2c_ok, bool i2s_overrun,
                                           bool i2s_stale, audio_capture_snapshot_t const * audio);
/* 音響フロントエンド稼働状態送信 */
static esp_err_t frontend_send_health(bool i2c_ok, bool i2s_overrun, bool i2s_stale,
                                      audio_capture_snapshot_t const * audio);
static bool frontend_feature_trigger_active(bool i2s_stale,
                                            audio_capture_snapshot_t const * audio); /* 音量トリガ判定 */
static esp_err_t frontend_send_feature_packet(frontend_feature_burst_t const * burst); /* 2フレーム送信 */
static bool frontend_feature_burst_take(frontend_feature_burst_t * burst, uint32_t now_ms); /* 完成イベント取得 */
static void acoustic_frontend_task(void * context);         /* 音響観測・送信タスク */

/** =================================================================*
 * @brief  DoA循環平均と品質算出
 * @details 集中度へ履歴充足率を掛け、fallbackは上限70、VADなしは上限40とする。
 * @param[in] doa XVF3800直近読出し
 * @param[out] filtered_deg 循環平均DoA[deg]
 * @param[out] confidence DoA品質[0..100]
 * ================================================================= */
static void frontend_doa_filter_update(xvf3800_doa_result_t const * doa, uint16_t * filtered_deg,
                                       uint8_t * confidence) {
    if ((doa == NULL) || (filtered_deg == NULL) || (confidence == NULL) || !doa->doa_valid) {
        memset(&s_doa_filter, 0, sizeof(s_doa_filter));
        if (filtered_deg != NULL) {
            *filtered_deg = ACOUSTIC_PROTOCOL_DOA_INVALID;
        }
        if (confidence != NULL) {
            *confidence = 0U;
        }
        return;
    }

    s_doa_filter.samples_deg[s_doa_filter.next] = doa->doa_deg;
    s_doa_filter.next = (uint8_t) ((s_doa_filter.next + 1U) % APP_DOA_FILTER_WINDOW);
    if (s_doa_filter.count < APP_DOA_FILTER_WINDOW) {
        s_doa_filter.count++;
    }

    float sum_sin = 0.0F;
    float sum_cos = 0.0F;
    for (uint8_t index = 0U; index < s_doa_filter.count; index++) {
        float const angle_rad = FRONTEND_DEG_TO_RAD((float) s_doa_filter.samples_deg[index]);
        sum_sin += sinf(angle_rad);
        sum_cos += cosf(angle_rad);
    }
    float angle_deg = FRONTEND_RAD_TO_DEG(atan2f(sum_sin, sum_cos));
    if (angle_deg < 0.0F) {
        angle_deg += 360.0F;
    }
    uint16_t rounded = (uint16_t) (angle_deg + 0.5F);
    *filtered_deg = (rounded >= 360U) ? 0U : rounded;

    float const concentration = sqrtf((sum_sin * sum_sin) + (sum_cos * sum_cos)) /
                                (float) s_doa_filter.count;
    float quality = concentration * 100.0F * (float) s_doa_filter.count / (float) APP_DOA_FILTER_WINDOW;
    if (doa->used_aec_fallback && (quality > 70.0F)) {
        quality = 70.0F;
    }
    if ((0U == doa->speech_detected_raw) && (quality > 40.0F)) {
        quality = 40.0F;
    }
    *confidence = (uint8_t) ((quality < 0.0F) ? 0U : (quality > 100.0F) ? 100U : (uint8_t) (quality + 0.5F));
}

/** =================================================================*
 * @brief  起動経過時刻取得
 * @return 起動からの経過時刻[ms]
 * ================================================================= */
static uint32_t frontend_uptime_ms(void) {
    return (uint32_t) ((uint64_t) esp_timer_get_time() / 1000ULL);
}

/** =================================================================*
 * @brief  音響プロトコルフレーム送信
 * @param[in] frame 送信フレーム
 * @param[in] length 送信長[byte]
 * @return USB CDC送信結果
 * ================================================================= */
static esp_err_t frontend_send_frame(uint8_t const * frame, size_t length) {
    if (length == 0U) {
        return ESP_ERR_INVALID_SIZE;
    }

    esp_err_t const err = usb_link_send(frame, length);
    s_sequence++;
    return err;
}

/** =================================================================*
 * @brief  起動情報送信
 * @return USB CDC送信結果
 * ================================================================= */
static esp_err_t frontend_send_hello(void) {
    acoustic_hello_t const hello = {
        .firmware_major = FRONTEND_FIRMWARE_MAJOR,
        .firmware_minor = FRONTEND_FIRMWARE_MINOR,
        .firmware_patch = FRONTEND_FIRMWARE_PATCH,
        .reserved = 0U,
        .capabilities = ACOUSTIC_CAPABILITY_DOA | ACOUSTIC_CAPABILITY_VAD | ACOUSTIC_CAPABILITY_LEVEL |
                        ACOUSTIC_CAPABILITY_WIFI | ACOUSTIC_CAPABILITY_DOA_DIAGNOSTICS,
        .boot_id = s_boot_id,
    };
    uint8_t frame[ACOUSTIC_PROTOCOL_MAX_FRAME_SIZE];
    size_t const length =
        acoustic_protocol_encode_hello(s_sequence, frontend_uptime_ms(), &hello, frame, sizeof(frame));
    return frontend_send_frame(frame, length);
}

/** =================================================================*
 * @brief  音響観測結果送信
 * @param[in] doa XVF3800の到来方向取得結果
 * @param[in] i2c_ok XVF3800通信成功状態
 * @param[in] i2s_overrun I2Sオーバーラン検出状態
 * @param[in] i2s_stale 音量取得の期限超過状態
 * @param[in] audio 最新音声キャプチャ状態
 * @return USB CDC送信結果
 * ================================================================= */
static esp_err_t frontend_send_observation(xvf3800_doa_result_t const * doa, bool i2c_ok, bool i2s_overrun,
                                           bool i2s_stale, audio_capture_snapshot_t const * audio) {
    uint8_t flags = 0U;
    if (i2s_overrun) {
        flags |= ACOUSTIC_AUDIO_FLAG_I2S_OVERRUN;
    }
    if (!i2c_ok) {
        flags |= ACOUSTIC_AUDIO_FLAG_I2C_ERROR;
    }
    if (i2s_stale) {
        flags |= ACOUSTIC_AUDIO_FLAG_I2S_STALE;
    }
    if (doa->used_aec_fallback) {
        flags |= ACOUSTIC_AUDIO_FLAG_DOA_FALLBACK;
    }

    uint16_t filtered_doa_deg = ACOUSTIC_PROTOCOL_DOA_INVALID;
    uint8_t doa_confidence = 0U;
    /* I2C失敗・無効DoAでは履歴を消し、復帰後の値へ古い方向を混ぜない。 */
    frontend_doa_filter_update(doa, &filtered_doa_deg, &doa_confidence);

    acoustic_observation_t const observation = {
        .doa_deg = filtered_doa_deg,
        .raw_doa_deg = (i2c_ok && doa->doa_valid) ? doa->doa_deg : ACOUSTIC_PROTOCOL_DOA_INVALID,
        .level_dbfs_x100 = audio->level_dbfs_x100,
        .peak_dbfs_x100 = audio->peak_dbfs_x100,
        .vad = (uint8_t) (i2c_ok && (doa->speech_detected_raw != 0U)),
        .doa_confidence = doa_confidence,
        .xvf_status = (uint8_t) s_xvf_status,
        .audio_flags = flags,
        .xvf_raw_status = doa->raw_status,
        .reserved = 0U,
        .audio_frame_count = audio->frame_count,
        .sample_sequence = s_observation_sequence,
    };
    g_acoustic_frontend_raw_doa_deg = observation.raw_doa_deg;
    g_acoustic_frontend_filtered_doa_deg = observation.doa_deg;
    g_acoustic_frontend_doa_confidence = observation.doa_confidence;
    uint32_t const sample_ms = frontend_uptime_ms();
    g_acoustic_frontend_last_sample_ms = sample_ms;
    g_acoustic_frontend_observation_sequence = observation.sample_sequence;
    uint8_t frame[ACOUSTIC_PROTOCOL_MAX_FRAME_SIZE];
    size_t const length =
        acoustic_protocol_encode_observation(s_sequence, sample_ms, &observation, frame, sizeof(frame));
    esp_err_t const err = frontend_send_frame(frame, length);
    s_observation_sequence++;
    if (ESP_OK == err) {
        g_acoustic_frontend_tx_count++;
    } else {
        g_acoustic_frontend_tx_error_count++;
    }
    return err;
}

/** =================================================================*
 * @brief  稼働状態送信
 * @param[in] i2c_ok XVF3800通信成功状態
 * @param[in] i2s_overrun I2Sオーバーラン検出状態
 * @param[in] i2s_stale 音量取得の期限超過状態
 * @param[in] audio 最新音声キャプチャ状態
 * @return USB CDC送信結果
 * ================================================================= */
static esp_err_t frontend_send_health(bool i2c_ok, bool i2s_overrun, bool i2s_stale,
                                      audio_capture_snapshot_t const * audio) {
    uint8_t flags = 0U;
    if (i2s_overrun) {
        flags |= ACOUSTIC_AUDIO_FLAG_I2S_OVERRUN;
    }
    if (!i2c_ok) {
        flags |= ACOUSTIC_AUDIO_FLAG_I2C_ERROR;
    }
    if (i2s_stale) {
        flags |= ACOUSTIC_AUDIO_FLAG_I2S_STALE;
    }

    acoustic_health_t const health = {
        .xvf_status = (uint8_t) s_xvf_status,
        .audio_flags = flags,
        .usb_connected = (uint8_t) usb_link_is_mounted(),
        .wifi_connected = (uint8_t) wifi_telemetry_is_connected(),
        .i2c_error_count = s_i2c_error_count,
        .i2s_overrun_count = audio->overrun_count,
    };
    uint8_t frame[ACOUSTIC_PROTOCOL_MAX_FRAME_SIZE];
    size_t const length =
        acoustic_protocol_encode_health(s_sequence, frontend_uptime_ms(), &health, frame, sizeof(frame));
    return frontend_send_frame(frame, length);
}

/** =================================================================*
 * @brief  特徴量イベントの音量トリガ判定
 * @return I2S入力が有効かつしきい値以上ならtrue
 * @details 学習対象は音声に限定しない。XVF3800のVADは音声向けのため、拍手・打音などを
 *          取りこぼさないようイベント収集の条件には使用しない。
 * ================================================================= */
static bool frontend_feature_trigger_active(bool i2s_stale, audio_capture_snapshot_t const * audio) {
    return !i2s_stale && audio->valid && audio->log_mel_self_test_pass &&
           (audio->level_dbfs_x100 >= APP_FEATURE_TRIGGER_LEVEL_DBFS_X100);
}

/** =================================================================*
 * @brief  特徴量イベントの2フレームをUSBへ送信
 * @param[in] burst 送信中イベント
 * @return USB CDC送信結果
 * @details flagsは凍結済みプロトコルで将来用に予約されているため、現時点では0を送る。
 * ================================================================= */
static esp_err_t frontend_send_feature_packet(frontend_feature_burst_t const * burst) {
    if ((burst == NULL) || !burst->active ||
        ((uint16_t) (burst->next_frame_index + ACOUSTIC_FEATURE_FRAMES_PER_PACKET) >
         ACOUSTIC_FEATURE_EVENT_FRAME_COUNT)) {
        return ESP_ERR_INVALID_ARG;
    }

    acoustic_feature_t feature = {
        .event_id = burst->event.event_id,
        .frame_index = burst->next_frame_index,
        .frame_count = ACOUSTIC_FEATURE_EVENT_FRAME_COUNT,
        .n_bins = ACOUSTIC_FEATURE_BIN_COUNT,
        .flags = 0U,
    };
    memcpy(feature.mel, burst->event.frames[burst->next_frame_index], sizeof(feature.mel));

    uint8_t frame[ACOUSTIC_PROTOCOL_MAX_FRAME_SIZE];
    size_t const length =
        acoustic_protocol_encode_feature(s_sequence, frontend_uptime_ms(), &feature, frame, sizeof(frame));
    return frontend_send_frame(frame, length);
}

/** =================================================================*
 * @brief  完成済み特徴量イベントをUSB送信用に取得
 * @param[out] burst 送信状態
 * @param[in] now_ms 現在時刻
 * @return 完成イベントを取得できた場合true
 * ================================================================= */
static bool frontend_feature_burst_take(frontend_feature_burst_t * burst, uint32_t now_ms) {
    if ((burst == NULL) || burst->active || !audio_capture_feature_event_take(&burst->event)) {
        return false;
    }
    burst->next_frame_index = 0U;
    burst->next_packet_at_ms = now_ms;
    burst->active = true;
    return true;
}

/** =================================================================*
 * @brief  音響観測タスク
 * @param[in] context FreeRTOSタスク引数
 * @details XVF3800とI2S音量を定期取得し、CPU0へUSB CDCで送信する。
 * ================================================================= */
static void acoustic_frontend_task(void * context) {
    (void) context;

    uint32_t last_health_ms = 0U;
    uint32_t last_hello_ms = 0U;
    uint32_t last_observation_ms = 0U;
    uint32_t last_feature_event_started_ms = 0U;
    bool hello_pending = false;
    TickType_t last_wake_tick = xTaskGetTickCount();

    while (true) {
        if (usb_link_take_new_session()) {
            audio_capture_feature_event_discard();
            s_feature_burst.active = false;
            last_feature_event_started_ms = 0U;
            last_observation_ms = 0U;
            memset(&s_doa_filter, 0, sizeof(s_doa_filter));
            hello_pending = true;
            last_hello_ms = 0U;
        }
        if (!usb_link_is_mounted()) {
            audio_capture_feature_event_discard();
            s_feature_burst.active = false;
            last_feature_event_started_ms = 0U;
            vTaskDelayUntil(&last_wake_tick, pdMS_TO_TICKS(APP_OBSERVATION_PERIOD_MS));
            continue;
        }
        uint32_t const loop_now_ms = frontend_uptime_ms();
        if (hello_pending || ((loop_now_ms - last_hello_ms) >= APP_HELLO_PERIOD_MS)) {
            if (frontend_send_hello() == ESP_OK) {
                hello_pending = false;
                last_hello_ms = loop_now_ms;
            }
        }

        {
            int64_t const processing_started_us = esp_timer_get_time();
            bool period_deadline_missed = false;
            if (0U != last_observation_ms) {
                uint32_t const period_ms = loop_now_ms - last_observation_ms;
                g_acoustic_frontend_last_period_ms = period_ms;
                if ((0U == g_acoustic_frontend_min_period_ms) ||
                    (period_ms < g_acoustic_frontend_min_period_ms)) {
                    g_acoustic_frontend_min_period_ms = period_ms;
                }
                if (period_ms > g_acoustic_frontend_max_period_ms) {
                    g_acoustic_frontend_max_period_ms = period_ms;
                }
                if (period_ms > APP_OBSERVATION_PERIOD_MS) {
                    period_deadline_missed = true;
                }
            }
            last_observation_ms = loop_now_ms;
            g_acoustic_frontend_read_count++;
            xvf3800_doa_result_t doa = {0};
            esp_err_t const i2c_result = xvf3800_control_read_doa(&doa);
            bool const i2c_ok = i2c_result == ESP_OK;
            if (i2c_ok) {
                if (doa.doa_valid) {
                    s_has_valid_doa = true;
                }
                s_xvf_status = s_has_valid_doa ? ACOUSTIC_XVF_STATUS_READY : ACOUSTIC_XVF_STATUS_STARTING;
            } else {
                s_i2c_error_count++;
                g_acoustic_frontend_read_error_count++;
                s_xvf_status = ACOUSTIC_XVF_STATUS_ERROR;
            }

            audio_capture_snapshot_t audio = {0};
            audio_capture_get_snapshot(&audio);
            uint32_t const now_ms = frontend_uptime_ms();
            bool const i2s_overrun = audio.overrun_count != s_previous_overrun_count;
            bool const i2s_stale = !audio.valid || ((now_ms - audio.captured_at_ms) > APP_AUDIO_STALE_TIMEOUT_MS);
            if (i2s_stale) {
                audio.level_dbfs_x100 = INT16_MIN;
                audio.peak_dbfs_x100 = INT16_MIN;
            }
            (void) frontend_send_observation(&doa, i2c_ok, i2s_overrun, i2s_stale, &audio);

            if ((now_ms - last_health_ms) >= APP_HEALTH_PERIOD_MS) {
                (void) frontend_send_health(i2c_ok, i2s_overrun, i2s_stale, &audio);
                last_health_ms = now_ms;
            }
            s_previous_overrun_count = audio.overrun_count;
            int64_t const processing_us = esp_timer_get_time() - processing_started_us;
            g_acoustic_frontend_processing_us =
                (processing_us > (int64_t) UINT32_MAX) ? UINT32_MAX : (uint32_t) processing_us;
            if (period_deadline_missed ||
                (processing_us > ((int64_t) APP_OBSERVATION_PERIOD_MS * 1000LL))) {
                g_acoustic_frontend_deadline_miss_count++;
            }

            bool const feature_trigger_active = frontend_feature_trigger_active(i2s_stale, &audio);
            if (!feature_trigger_active) {
                /* 無音後は次の音量立ち上がりを即座に新イベントとして収集する。 */
                last_feature_event_started_ms = 0U;
            } else if (((0U == last_feature_event_started_ms) ||
                        ((uint32_t) (now_ms - last_feature_event_started_ms) >=
                         APP_FEATURE_RETRIGGER_PERIOD_MS)) &&
                       (audio.feature_ring_frames >= APP_FEATURE_PRE_TRIGGER_FRAMES)) {
                if (audio_capture_feature_event_start(s_next_feature_event_id) == ESP_OK) {
                    last_feature_event_started_ms = now_ms;
                    s_next_feature_event_id++;
                }
            }
        }

        uint32_t const burst_now_ms = frontend_uptime_ms();
        (void) frontend_feature_burst_take(&s_feature_burst, burst_now_ms);
        if (s_feature_burst.active && ((int32_t) (burst_now_ms - s_feature_burst.next_packet_at_ms) >= 0)) {
            if (frontend_send_feature_packet(&s_feature_burst) != ESP_OK) {
                /* 欠落後に続きフレームを送らず、次イベントのframe_index=0で再同期させる。 */
                s_feature_burst.active = false;
            } else {
                s_feature_burst.next_frame_index =
                    (uint16_t) (s_feature_burst.next_frame_index + ACOUSTIC_FEATURE_FRAMES_PER_PACKET);
                if (s_feature_burst.next_frame_index == ACOUSTIC_FEATURE_EVENT_FRAME_COUNT) {
                    s_feature_burst.active = false;
                } else {
                    s_feature_burst.next_packet_at_ms = frontend_uptime_ms() + APP_FEATURE_PACKET_PERIOD_MS;
                }
            }
        }

        vTaskDelayUntil(&last_wake_tick, pdMS_TO_TICKS(APP_OBSERVATION_PERIOD_MS));
    }
}

/** =================================================================*
 * @brief  音響観測タスク開始
 * @return 初期化結果
 * ================================================================= */
esp_err_t acoustic_frontend_start(void) {
    s_boot_id = esp_random();
    s_next_feature_event_id = (uint16_t) esp_random();
    g_acoustic_frontend_raw_doa_deg = ACOUSTIC_PROTOCOL_DOA_INVALID;
    g_acoustic_frontend_filtered_doa_deg = ACOUSTIC_PROTOCOL_DOA_INVALID;

    BaseType_t const result = xTaskCreate(acoustic_frontend_task, "acoustic_frontend", APP_FRONTEND_TASK_STACK_SIZE,
                                          NULL, APP_FRONTEND_TASK_PRIORITY, NULL);
    return (result == pdPASS) ? ESP_OK : ESP_ERR_NO_MEM;
}
