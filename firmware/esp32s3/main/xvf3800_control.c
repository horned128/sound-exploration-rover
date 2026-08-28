#include "xvf3800_control.h"

#include <math.h>
#include <stddef.h>
#include <string.h>
#include "app_config.h"
#include "driver/i2c_master.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define XVF3800_I2C_ADDRESS                  (0x2CU)
#define XVF3800_GPO_SERVICER_RESOURCE_ID     (20U)
#define XVF3800_DOA_COMMAND_ID               (18U)
#define XVF3800_AEC_RESOURCE_ID              (33U)
#define XVF3800_AEC_AZIMUTH_COMMAND_ID       (75U)
#define XVF3800_AEC_SPENERGY_COMMAND_ID      (80U)
#define XVF3800_AUDIO_MGR_RESOURCE_ID        (35U)
#define XVF3800_AUDIO_MGR_SELECTED_AZIMUTH_COMMAND_ID (11U)
#define XVF3800_RESOURCE_READ_BIT            (0x80U)
#define XVF3800_DOA_RESPONSE_SIZE            (5U)
#define XVF3800_AEC_RESPONSE_SIZE            (17U)
#define XVF3800_SELECTED_AZIMUTH_RESPONSE_SIZE (9U)
#define XVF3800_AEC_AUTO_BEAM_INDEX          (3U)
#define XVF3800_SELECTED_PROCESSED_DOA_INDEX (0U)
#define XVF3800_CONTROL_SUCCESS              (0x00U)
#define XVF3800_CONTROL_RETRY                (0x40U)
#define XVF3800_CONTROL_RETRY_COUNT          (3U)
#define XVF3800_RADIANS_TO_DEGREES           (57.2957795F)

static char const *const TAG = "xvf3800";
static i2c_master_bus_handle_t s_i2c_bus;
static i2c_master_dev_handle_t s_xvf_device;

static esp_err_t xvf3800_control_read_resource(
    uint8_t resource_id,
    uint8_t command_id,
    uint8_t *response,
    size_t response_size);
static float xvf3800_control_decode_float(uint8_t const *bytes);
static void xvf3800_control_read_auto_beam_speech_energy(
    xvf3800_doa_result_t *result);
static esp_err_t xvf3800_control_read_selected_azimuth(
    xvf3800_doa_result_t *result);
static esp_err_t xvf3800_control_read_aec_fallback(
    xvf3800_doa_result_t *result);

static esp_err_t xvf3800_control_read_resource(
    uint8_t resource_id,
    uint8_t command_id,
    uint8_t *response,
    size_t response_size) {
    if ((response == NULL) ||
        (response_size == 0U) ||
        (response_size > UINT8_MAX)) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t const request[] = {
        resource_id,
        command_id | XVF3800_RESOURCE_READ_BIT,
        (uint8_t) response_size,
    };
    for (uint32_t attempt = 0U;
         attempt < XVF3800_CONTROL_RETRY_COUNT;
         attempt++) {
        ESP_RETURN_ON_ERROR(
            i2c_master_transmit_receive(
                s_xvf_device,
                request,
                sizeof(request),
                response,
                response_size,
                APP_XVF_I2C_TIMEOUT_MS),
            TAG,
            "XVF3800 resource read failed");
        if (response[0] != XVF3800_CONTROL_RETRY) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(1U));
    }
    return ESP_OK;
}

static float xvf3800_control_decode_float(uint8_t const *bytes) {
    uint32_t const bits =
        (uint32_t) bytes[0] |
        ((uint32_t) bytes[1] << 8U) |
        ((uint32_t) bytes[2] << 16U) |
        ((uint32_t) bytes[3] << 24U);
    float value;
    memcpy(&value, &bits, sizeof(value));
    return value;
}

static void xvf3800_control_read_auto_beam_speech_energy(
    xvf3800_doa_result_t *result) {
    uint8_t energy_response[XVF3800_AEC_RESPONSE_SIZE] = {0U};
    esp_err_t const energy_error = xvf3800_control_read_resource(
        XVF3800_AEC_RESOURCE_ID,
        XVF3800_AEC_SPENERGY_COMMAND_ID,
        energy_response,
        sizeof(energy_response));
    if ((energy_error == ESP_OK) &&
        (energy_response[0] == XVF3800_CONTROL_SUCCESS)) {
        size_t const offset = 1U +
            (XVF3800_AEC_AUTO_BEAM_INDEX * sizeof(float));
        float const energy = xvf3800_control_decode_float(
            &energy_response[offset]);
        result->speech_detected_raw =
            (uint16_t) (isfinite(energy) && (energy > 0.0F));
    }
}

static esp_err_t xvf3800_control_read_selected_azimuth(
    xvf3800_doa_result_t *result) {
    uint8_t response[XVF3800_SELECTED_AZIMUTH_RESPONSE_SIZE] = {0U};
    ESP_RETURN_ON_ERROR(
        xvf3800_control_read_resource(
            XVF3800_AUDIO_MGR_RESOURCE_ID,
            XVF3800_AUDIO_MGR_SELECTED_AZIMUTH_COMMAND_ID,
            response,
            sizeof(response)),
        TAG,
        "XVF3800 selected azimuth read failed");
    if (response[0] != XVF3800_CONTROL_SUCCESS) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    size_t const offset = 1U +
        (XVF3800_SELECTED_PROCESSED_DOA_INDEX * sizeof(float));
    float degrees = xvf3800_control_decode_float(&response[offset]) *
                    XVF3800_RADIANS_TO_DEGREES;
    if (!isfinite(degrees)) {
        return ESP_ERR_INVALID_RESPONSE;
    }
    while (degrees < 0.0F) {
        degrees += 360.0F;
    }
    while (degrees >= 360.0F) {
        degrees -= 360.0F;
    }

    result->doa_deg = (uint16_t) (degrees + 0.5F);
    if (result->doa_deg >= 360U) {
        result->doa_deg = 0U;
    }
    result->speech_detected_raw = 0U;
    result->doa_valid = true;
    result->used_aec_fallback = true;
    xvf3800_control_read_auto_beam_speech_energy(result);
    return ESP_OK;
}

static esp_err_t xvf3800_control_read_aec_fallback(
    xvf3800_doa_result_t *result) {
    uint8_t azimuth_response[XVF3800_AEC_RESPONSE_SIZE] = {0U};
    ESP_RETURN_ON_ERROR(
        xvf3800_control_read_resource(
            XVF3800_AEC_RESOURCE_ID,
            XVF3800_AEC_AZIMUTH_COMMAND_ID,
            azimuth_response,
            sizeof(azimuth_response)),
        TAG,
        "XVF3800 AEC azimuth read failed");
    if (azimuth_response[0] != XVF3800_CONTROL_SUCCESS) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    size_t const offset =
        1U + (XVF3800_AEC_AUTO_BEAM_INDEX * sizeof(float));
    float degrees = xvf3800_control_decode_float(
        &azimuth_response[offset]) * XVF3800_RADIANS_TO_DEGREES;
    if (!isfinite(degrees)) {
        return ESP_ERR_INVALID_RESPONSE;
    }
    while (degrees < 0.0F) {
        degrees += 360.0F;
    }
    while (degrees >= 360.0F) {
        degrees -= 360.0F;
    }

    result->doa_deg = (uint16_t) (degrees + 0.5F);
    if (result->doa_deg >= 360U) {
        result->doa_deg = 0U;
    }
    result->speech_detected_raw = 0U;
    result->doa_valid = true;
    result->used_aec_fallback = true;
    xvf3800_control_read_auto_beam_speech_energy(result);
    return ESP_OK;
}

esp_err_t xvf3800_control_init(void) {
    i2c_master_bus_config_t const bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = APP_XVF_I2C_SDA_GPIO,
        .scl_io_num = APP_XVF_I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7U,
        .flags.enable_internal_pullup = true,
    };
    ESP_RETURN_ON_ERROR(
        i2c_new_master_bus(&bus_config, &s_i2c_bus),
        TAG,
        "I2C bus initialization failed");

    i2c_device_config_t const device_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = XVF3800_I2C_ADDRESS,
        .scl_speed_hz = APP_XVF_I2C_FREQUENCY_HZ,
    };
    ESP_RETURN_ON_ERROR(
        i2c_master_bus_add_device(
            s_i2c_bus,
            &device_config,
            &s_xvf_device),
        TAG,
        "XVF3800 I2C device registration failed");

    return ESP_OK;
}

esp_err_t xvf3800_control_read_doa(xvf3800_doa_result_t *result) {
    if (result == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    *result = (xvf3800_doa_result_t) {0};
    uint8_t response[XVF3800_DOA_RESPONSE_SIZE] = {0U};

    ESP_RETURN_ON_ERROR(
        xvf3800_control_read_resource(
            XVF3800_GPO_SERVICER_RESOURCE_ID,
            XVF3800_DOA_COMMAND_ID,
            response,
            sizeof(response)),
        TAG,
        "XVF3800 DoA read failed");

    result->raw_status = response[0];
    if (response[0] == XVF3800_CONTROL_SUCCESS) {
        result->doa_deg =
            (uint16_t) response[1] | ((uint16_t) response[2] << 8U);
        result->speech_detected_raw =
            (uint16_t) response[3] | ((uint16_t) response[4] << 8U);
        result->doa_valid = result->doa_deg < 360U;
        if (result->doa_valid) {
            return ESP_OK;
        }
    }

    esp_err_t const selected_azimuth_error =
        xvf3800_control_read_selected_azimuth(result);
    if (selected_azimuth_error == ESP_OK) {
        return ESP_OK;
    }

    return xvf3800_control_read_aec_fallback(result);
}
