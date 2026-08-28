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
#include <stdbool.h>                                        /* 真偽値 */
#include <stddef.h>                                         /* size_t */
#include <stdint.h>                                         /* 固定幅整数型 */

#define FRONTEND_FIRMWARE_MAJOR            (1U)
#define FRONTEND_FIRMWARE_MINOR            (0U)
#define FRONTEND_FIRMWARE_PATCH            (0U)

static uint32_t s_sequence;                                 /**< USB送信フレーム連番 */
static uint32_t s_boot_id;                                  /**< 起動ごとのランダムID */
static uint32_t s_i2c_error_count;                          /**< XVF3800通信失敗累積数 */
static uint32_t s_previous_overrun_count;                   /**< 前回観測時のI2Sオーバーラン数 */
/**< 有効な到来方向の取得済み状態 */
static acoustic_xvf_status_t s_xvf_status = ACOUSTIC_XVF_STATUS_STARTING;
static bool s_has_valid_doa;                                /**< 起動後に有効な到来方向を得た状態 */

static uint32_t frontend_uptime_ms(void);                   /* 起動からの経過時刻取得 */
static esp_err_t frontend_send_frame(uint8_t const * frame, size_t length); /* 音響プロトコルフレーム送信 */
static esp_err_t frontend_send_hello(void);                 /* 起動情報送信 */
/* 音響観測結果送信 */
static esp_err_t frontend_send_observation(xvf3800_doa_result_t const * doa, bool i2c_ok, bool i2s_overrun,
                                           bool i2s_stale, audio_capture_snapshot_t const * audio);
/* 音響フロントエンド稼働状態送信 */
static esp_err_t frontend_send_health(bool i2c_ok, bool i2s_overrun, bool i2s_stale,
                                      audio_capture_snapshot_t const * audio);
static void acoustic_frontend_task(void * context);         /* 音響観測・送信タスク */

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
        .capabilities =
            ACOUSTIC_CAPABILITY_DOA | ACOUSTIC_CAPABILITY_VAD | ACOUSTIC_CAPABILITY_LEVEL | ACOUSTIC_CAPABILITY_WIFI,
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

    acoustic_observation_t const observation = {
        .doa_deg = (i2c_ok && doa->doa_valid) ? doa->doa_deg : ACOUSTIC_PROTOCOL_DOA_INVALID,
        .level_dbfs_x100 = audio->level_dbfs_x100,
        .peak_dbfs_x100 = audio->peak_dbfs_x100,
        .vad = (uint8_t) (i2c_ok && (doa->speech_detected_raw != 0U)),
        .xvf_status = (uint8_t) s_xvf_status,
        .audio_flags = flags,
        .xvf_raw_status = doa->raw_status,
        .audio_frame_count = audio->frame_count,
    };
    uint8_t frame[ACOUSTIC_PROTOCOL_MAX_FRAME_SIZE];
    size_t const length =
        acoustic_protocol_encode_observation(s_sequence, frontend_uptime_ms(), &observation, frame, sizeof(frame));
    return frontend_send_frame(frame, length);
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
 * @brief  音響観測タスク
 * @param[in] context FreeRTOSタスク引数
 * @details XVF3800とI2S音量を定期取得し、CPU0へUSB CDCで送信する。
 * ================================================================= */
static void acoustic_frontend_task(void * context) {
    (void) context;

    uint32_t last_health_ms = 0U;
    uint32_t last_hello_ms = 0U;
    bool hello_pending = false;

    while (true) {
        if (usb_link_take_new_session()) {
            hello_pending = true;
            last_hello_ms = 0U;
        }
        if (!usb_link_is_mounted()) {
            vTaskDelay(pdMS_TO_TICKS(APP_OBSERVATION_PERIOD_MS));
            continue;
        }
        uint32_t const loop_now_ms = frontend_uptime_ms();
        if (hello_pending || ((loop_now_ms - last_hello_ms) >= APP_HELLO_PERIOD_MS)) {
            if (frontend_send_hello() == ESP_OK) {
                hello_pending = false;
                last_hello_ms = loop_now_ms;
            }
            vTaskDelay(pdMS_TO_TICKS(APP_OBSERVATION_PERIOD_MS));
            continue;
        }

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

        vTaskDelay(pdMS_TO_TICKS(APP_OBSERVATION_PERIOD_MS));
    }
}

/** =================================================================*
 * @brief  音響観測タスク開始
 * @return 初期化結果
 * ================================================================= */
esp_err_t acoustic_frontend_start(void) {
    s_boot_id = esp_random();

    BaseType_t const result = xTaskCreate(acoustic_frontend_task, "acoustic_frontend", APP_FRONTEND_TASK_STACK_SIZE,
                                          NULL, APP_FRONTEND_TASK_PRIORITY, NULL);
    return (result == pdPASS) ? ESP_OK : ESP_ERR_NO_MEM;
}
