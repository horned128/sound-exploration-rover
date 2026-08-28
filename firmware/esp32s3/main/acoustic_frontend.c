#include "acoustic_frontend.h"

#include <limits.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "acoustic_protocol.h"
#include "app_config.h"
#include "audio_capture.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "usb_link.h"
#include "wifi_telemetry.h"
#include "xvf3800_control.h"

#define FRONTEND_FIRMWARE_MAJOR               (1U)
#define FRONTEND_FIRMWARE_MINOR               (0U)
#define FRONTEND_FIRMWARE_PATCH               (0U)

static uint32_t s_sequence;
static uint32_t s_boot_id;
static uint32_t s_i2c_error_count;
static uint32_t s_previous_overrun_count;
static acoustic_xvf_status_t s_xvf_status =
    ACOUSTIC_XVF_STATUS_STARTING;
static bool s_has_valid_doa;

static uint32_t frontend_uptime_ms(void);
static esp_err_t frontend_send_frame(uint8_t const *frame, size_t length);
static esp_err_t frontend_send_hello(void);
static esp_err_t frontend_send_observation(
    xvf3800_doa_result_t const *doa,
    bool i2c_ok,
    bool i2s_overrun,
    bool i2s_stale,
    audio_capture_snapshot_t const *audio);
static esp_err_t frontend_send_health(
    bool i2c_ok,
    bool i2s_overrun,
    bool i2s_stale,
    audio_capture_snapshot_t const *audio);
static void acoustic_frontend_task(void *context);

static uint32_t frontend_uptime_ms(void) {
    return (uint32_t) ((uint64_t) esp_timer_get_time() / 1000ULL);
}

static esp_err_t frontend_send_frame(uint8_t const *frame, size_t length) {
    if (length == 0U) {
        return ESP_ERR_INVALID_SIZE;
    }

    esp_err_t const err = usb_link_send(frame, length);
    s_sequence++;
    return err;
}

static esp_err_t frontend_send_hello(void) {
    acoustic_hello_t const hello = {
        .firmware_major = FRONTEND_FIRMWARE_MAJOR,
        .firmware_minor = FRONTEND_FIRMWARE_MINOR,
        .firmware_patch = FRONTEND_FIRMWARE_PATCH,
        .reserved = 0U,
        .capabilities = ACOUSTIC_CAPABILITY_DOA |
                        ACOUSTIC_CAPABILITY_VAD |
                        ACOUSTIC_CAPABILITY_LEVEL |
                        ACOUSTIC_CAPABILITY_WIFI,
        .boot_id = s_boot_id,
    };
    uint8_t frame[ACOUSTIC_PROTOCOL_MAX_FRAME_SIZE];
    size_t const length = acoustic_protocol_encode_hello(
        s_sequence,
        frontend_uptime_ms(),
        &hello,
        frame,
        sizeof(frame));
    return frontend_send_frame(frame, length);
}

static esp_err_t frontend_send_observation(
    xvf3800_doa_result_t const *doa,
    bool i2c_ok,
    bool i2s_overrun,
    bool i2s_stale,
    audio_capture_snapshot_t const *audio) {
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
        .doa_deg = (i2c_ok && doa->doa_valid) ?
                   doa->doa_deg : ACOUSTIC_PROTOCOL_DOA_INVALID,
        .level_dbfs_x100 = audio->level_dbfs_x100,
        .peak_dbfs_x100 = audio->peak_dbfs_x100,
        .vad = (uint8_t) (i2c_ok &&
                          (doa->speech_detected_raw != 0U)),
        .xvf_status = (uint8_t) s_xvf_status,
        .audio_flags = flags,
        .xvf_raw_status = doa->raw_status,
        .audio_frame_count = audio->frame_count,
    };
    uint8_t frame[ACOUSTIC_PROTOCOL_MAX_FRAME_SIZE];
    size_t const length = acoustic_protocol_encode_observation(
        s_sequence,
        frontend_uptime_ms(),
        &observation,
        frame,
        sizeof(frame));
    return frontend_send_frame(frame, length);
}

static esp_err_t frontend_send_health(
    bool i2c_ok,
    bool i2s_overrun,
    bool i2s_stale,
    audio_capture_snapshot_t const *audio) {
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
    size_t const length = acoustic_protocol_encode_health(
        s_sequence,
        frontend_uptime_ms(),
        &health,
        frame,
        sizeof(frame));
    return frontend_send_frame(frame, length);
}

static void acoustic_frontend_task(void *context) {
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
        if (hello_pending ||
            ((loop_now_ms - last_hello_ms) >= APP_HELLO_PERIOD_MS)) {
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
            s_xvf_status = s_has_valid_doa ?
                           ACOUSTIC_XVF_STATUS_READY :
                           ACOUSTIC_XVF_STATUS_STARTING;
        } else {
            s_i2c_error_count++;
            s_xvf_status = ACOUSTIC_XVF_STATUS_ERROR;
        }

        audio_capture_snapshot_t audio = {0};
        audio_capture_get_snapshot(&audio);
        uint32_t const now_ms = frontend_uptime_ms();
        bool const i2s_overrun =
            audio.overrun_count != s_previous_overrun_count;
        bool const i2s_stale =
            !audio.valid ||
            ((now_ms - audio.captured_at_ms) >
             APP_AUDIO_STALE_TIMEOUT_MS);
        if (i2s_stale) {
            audio.level_dbfs_x100 = INT16_MIN;
            audio.peak_dbfs_x100 = INT16_MIN;
        }
        (void) frontend_send_observation(
            &doa,
            i2c_ok,
            i2s_overrun,
            i2s_stale,
            &audio);

        if ((now_ms - last_health_ms) >= APP_HEALTH_PERIOD_MS) {
            (void) frontend_send_health(
                i2c_ok,
                i2s_overrun,
                i2s_stale,
                &audio);
            last_health_ms = now_ms;
        }
        s_previous_overrun_count = audio.overrun_count;

        vTaskDelay(pdMS_TO_TICKS(APP_OBSERVATION_PERIOD_MS));
    }
}

esp_err_t acoustic_frontend_start(void) {
    s_boot_id = esp_random();

    BaseType_t const result = xTaskCreate(
        acoustic_frontend_task,
        "acoustic_frontend",
        APP_FRONTEND_TASK_STACK_SIZE,
        NULL,
        APP_FRONTEND_TASK_PRIORITY,
        NULL);
    return (result == pdPASS) ? ESP_OK : ESP_ERR_NO_MEM;
}
