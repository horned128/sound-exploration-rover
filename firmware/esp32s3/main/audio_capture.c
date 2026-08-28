#include "audio_capture.h"

#include <limits.h>
#include <math.h>
#include <stddef.h>
#include "app_config.h"
#include "driver/i2s_std.h"
#include "esp_attr.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static char const *const TAG = "audio_capture";
static i2s_chan_handle_t s_rx_channel;
static portMUX_TYPE s_snapshot_lock = portMUX_INITIALIZER_UNLOCKED;
static audio_capture_snapshot_t s_snapshot = {
    .level_dbfs_x100 = INT16_MIN,
    .peak_dbfs_x100 = INT16_MIN,
};

static int16_t amplitude_to_dbfs_x100(double amplitude);
static bool audio_capture_overrun_callback(
    i2s_chan_handle_t handle,
    i2s_event_data_t *event,
    void *user_context);
static void audio_capture_task(void *context);

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

static bool IRAM_ATTR audio_capture_overrun_callback(
    i2s_chan_handle_t handle,
    i2s_event_data_t *event,
    void *user_context) {
    (void) handle;
    (void) event;
    (void) user_context;

    portENTER_CRITICAL_ISR(&s_snapshot_lock);
    s_snapshot.overrun_count++;
    portEXIT_CRITICAL_ISR(&s_snapshot_lock);
    return false;
}

static void audio_capture_task(void *context) {
    (void) context;

    int32_t samples[APP_AUDIO_BLOCK_FRAMES * APP_AUDIO_CHANNEL_COUNT];
    double filtered_rms = 0.0;
    double filtered_peak = 0.0;

    while (true) {
        size_t bytes_read = 0U;
        esp_err_t const err = i2s_channel_read(
            s_rx_channel,
            samples,
            sizeof(samples),
            &bytes_read,
            portMAX_DELAY);

        if ((err != ESP_OK) || (bytes_read == 0U)) {
            portENTER_CRITICAL(&s_snapshot_lock);
            s_snapshot.valid = false;
            portEXIT_CRITICAL(&s_snapshot_lock);
            continue;
        }

        size_t const sample_count = bytes_read / sizeof(samples[0]);
        double square_sum = 0.0;
        double block_peak = 0.0;

        for (size_t index = 0U; index < sample_count; index++) {
            double const normalized =
                (double) samples[index] / (double) INT32_MAX;
            double const magnitude = fabs(normalized);
            square_sum += normalized * normalized;
            if (magnitude > block_peak) {
                block_peak = magnitude;
            }
        }

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
        s_snapshot.level_dbfs_x100 =
            amplitude_to_dbfs_x100(filtered_rms);
        s_snapshot.peak_dbfs_x100 =
            amplitude_to_dbfs_x100(filtered_peak);
        s_snapshot.frame_count +=
            (uint32_t) (sample_count / APP_AUDIO_CHANNEL_COUNT);
        s_snapshot.captured_at_ms =
            (uint32_t) ((uint64_t) esp_timer_get_time() / 1000ULL);
        s_snapshot.valid = true;
        portEXIT_CRITICAL(&s_snapshot_lock);
    }
}

esp_err_t audio_capture_start(void) {
    i2s_chan_config_t const channel_config =
        I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
    ESP_RETURN_ON_ERROR(
        i2s_new_channel(&channel_config, NULL, &s_rx_channel),
        TAG,
        "I2S RX channel creation failed");

    i2s_std_config_t const standard_config = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(APP_AUDIO_SAMPLE_RATE_HZ),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
            I2S_DATA_BIT_WIDTH_32BIT,
            I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = APP_XVF_I2S_BCLK_GPIO,
            .ws = APP_XVF_I2S_WS_GPIO,
            .dout = APP_XVF_I2S_DOUT_GPIO,
            .din = APP_XVF_I2S_DIN_GPIO,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };

    ESP_RETURN_ON_ERROR(
        i2s_channel_init_std_mode(s_rx_channel, &standard_config),
        TAG,
        "I2S standard mode initialization failed");

    i2s_event_callbacks_t const callbacks = {
        .on_recv = NULL,
        .on_recv_q_ovf = audio_capture_overrun_callback,
        .on_sent = NULL,
        .on_send_q_ovf = NULL,
    };
    ESP_RETURN_ON_ERROR(
        i2s_channel_register_event_callback(
            s_rx_channel,
            &callbacks,
            NULL),
        TAG,
        "I2S callback registration failed");
    ESP_RETURN_ON_ERROR(
        i2s_channel_enable(s_rx_channel),
        TAG,
        "I2S RX channel enable failed");

    BaseType_t const result = xTaskCreate(
        audio_capture_task,
        "audio_capture",
        APP_AUDIO_TASK_STACK_SIZE,
        NULL,
        APP_AUDIO_TASK_PRIORITY,
        NULL);
    if (result != pdPASS) {
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

void audio_capture_get_snapshot(audio_capture_snapshot_t *snapshot) {
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
