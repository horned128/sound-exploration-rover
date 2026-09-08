/** =================================================================*
 * @file   vl53l1x.c
 * @brief  VL53L1X ToF距離センサー実装
 * ================================================================= */
#include "vl53l1x.h"                                       /* VL53L1X API */
#include "platform/i2c_bus.h"                              /* I2CバスAPI */
#include "config/sensor_config.h"                          /* ToFアドレス、距離範囲 */
#include <string.h>                                         /* memcpy */

#define VL53L1X_REG_MODEL_ID                (0x010FU)
#define VL53L1X_REG_CONFIGURATION_START     (0x002DU)
#define VL53L1X_REG_VHV_TIMEOUT              (0x0008U)
#define VL53L1X_REG_VHV_INIT                 (0x000BU)
#define VL53L1X_REG_GPIO_HV_MUX_CTRL         (0x0030U)
#define VL53L1X_REG_GPIO_TIO_HV_STATUS       (0x0031U)
#define VL53L1X_REG_RANGE_STATUS             (0x0089U)
#define VL53L1X_REG_INTERRUPT_CLEAR          (0x0086U)
#define VL53L1X_REG_MODE_START               (0x0087U)
#define VL53L1X_REG_DISTANCE_MM              (0x0096U)
#define VL53L1X_MODEL_ID                     (0xEACCU)
#define VL53L1X_RANGE_STATUS_VALID           (9U)

/* ST VL53L1X Ultra Lite Driverの既定測距設定（0x002D～0x0087）。 */
LOCAL UB const vl53l1x_default_configuration[] = {
    0x00U, 0x00U, 0x00U, 0x01U, 0x02U, 0x00U, 0x02U, 0x08U, 0x00U, 0x08U, 0x10U, 0x01U,
    0x01U, 0x00U, 0x00U, 0x00U, 0x00U, 0xFFU, 0x00U, 0x0FU, 0x00U, 0x00U, 0x00U, 0x00U,
    0x00U, 0x20U, 0x0BU, 0x00U, 0x00U, 0x02U, 0x0AU, 0x21U, 0x00U, 0x00U, 0x05U, 0x00U,
    0x00U, 0x00U, 0x00U, 0xC8U, 0x00U, 0x00U, 0x38U, 0xFFU, 0x01U, 0x00U, 0x08U, 0x00U,
    0x00U, 0x01U, 0xCCU, 0x0FU, 0x01U, 0xF1U, 0x0DU, 0x01U, 0x68U, 0x00U, 0x80U, 0x08U,
    0xB8U, 0x00U, 0x00U, 0x00U, 0x00U, 0x0FU, 0x89U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U,
    0x00U, 0x00U, 0x01U, 0x0FU, 0x0DU, 0x0EU, 0x0EU, 0x00U, 0x00U, 0x02U, 0xC7U,
    0xFFU, 0x9BU, 0x00U, 0x00U, 0x00U, 0x01U, 0x00U, 0x00U,
};

LOCAL fsp_err_t vl53l1x_write_register(UH register_address, UB value); /* 8bitレジスタ書込み */
LOCAL fsp_err_t vl53l1x_read_registers(UH register_address, UB * p_data, UW length); /* レジスタ連続読出し */
LOCAL fsp_err_t vl53l1x_wait_data_ready(BOOL * p_timed_out); /* 新しい測距結果待ち */
LOCAL fsp_err_t vl53l1x_read_measurement(vl53l1x_reading_t * p_reading, BOOL * p_timed_out); /* 生測距結果取得 */

/** =================================================================*
 * @brief  VL53L1Xの8bitレジスタへ書込み
 * @param[in] register_address 16bitレジスタアドレス
 * @param[in] value 書込み値
 * @return FSPエラーコード
 * ================================================================= */
LOCAL fsp_err_t vl53l1x_write_register(UH register_address, UB value) {
    UB data[3] = {
        (UB) (register_address >> 8),
        (UB) register_address,
        value,
    };
    return i2c_bus_write(CPU0_VL53L1X_ADDRESS, data, sizeof(data));
}

/** =================================================================*
 * @brief  VL53L1Xレジスタを連続読出し
 * @param[in] register_address 16bitレジスタアドレス
 * @param[out] p_data 読出し先
 * @param[in] length 読出し長[byte]
 * @return FSPエラーコード
 * ================================================================= */
LOCAL fsp_err_t vl53l1x_read_registers(UH register_address, UB * p_data, UW length) {
    UB const pointer[2] = {
        (UB) (register_address >> 8),
        (UB) register_address,
    };
    return i2c_bus_write_read(CPU0_VL53L1X_ADDRESS, pointer, sizeof(pointer), p_data, length);
}

/** =================================================================*
 * @brief  VL53L1Xの新しい測距結果を待つ
 * @details GPIO_HV_MUX_CTRLの割込み極性とGPIO_TIO_HV_STATUSを用いる。
 * @return 結果が準備できればFSP_SUCCESS、期限超過ならFSP_ERR_TIMEOUT
 * ================================================================= */
LOCAL fsp_err_t vl53l1x_wait_data_ready(BOOL * p_timed_out) {
    if (NULL == p_timed_out) {
        return FSP_ERR_INVALID_ARGUMENT;
    }
    *p_timed_out = FALSE;

    for (UW elapsed_ms = 0U; elapsed_ms < CPU0_TOF_DATA_READY_TIMEOUT_MS; elapsed_ms++) {
        UB gpio_status[2] = {0U};
        fsp_err_t const err = vl53l1x_read_registers(VL53L1X_REG_GPIO_HV_MUX_CTRL, gpio_status,
                                                      sizeof(gpio_status));
        if (FSP_SUCCESS != err) {
            return err;
        }

        BOOL const interrupt_active_high = 0U == (gpio_status[0] & 0x10U);
        if ((gpio_status[1] & 0x01U) == (interrupt_active_high ? 1U : 0U)) {
            return FSP_SUCCESS;
        }
        R_BSP_SoftwareDelay(1U, BSP_DELAY_UNITS_MILLISECONDS);
    }

    *p_timed_out = TRUE;
    return FSP_ERR_TIMEOUT;
}

/** =================================================================*
 * @brief  VL53L1Xを初期化して連続測距を開始
 * @details 同一I2Cアドレスのため、呼出し前にTCA9548Aで対象チャネルを選択する。
 * @return FSPエラーコード
 * ================================================================= */
EXPORT fsp_err_t vl53l1x_init(vl53l1x_result_t * p_result) {
    if (NULL == p_result) {
        return FSP_ERR_INVALID_ARGUMENT;
    }
    *p_result = VL53L1X_RESULT_TRANSPORT_ERROR;

    UB model_id[2] = {0U};
    fsp_err_t err = vl53l1x_read_registers(VL53L1X_REG_MODEL_ID, model_id, sizeof(model_id));
    if (FSP_SUCCESS != err) {
        return err;
    }

    UH const model = (UH) (((UH) model_id[0] << 8) | model_id[1]);
    if (VL53L1X_MODEL_ID != model) {
        return FSP_ERR_NOT_FOUND;
    }

    UB configuration[2U + sizeof(vl53l1x_default_configuration)] = {
        (UB) (VL53L1X_REG_CONFIGURATION_START >> 8),
        (UB) VL53L1X_REG_CONFIGURATION_START,
    };
    memcpy(&configuration[2], vl53l1x_default_configuration, sizeof(vl53l1x_default_configuration));
    err = i2c_bus_write(CPU0_VL53L1X_ADDRESS, configuration, sizeof(configuration));
    if (FSP_SUCCESS != err) {
        return err;
    }

    err = vl53l1x_write_register(VL53L1X_REG_MODE_START, 0x40U);
    if (FSP_SUCCESS != err) {
        return err;
    }
    BOOL data_ready_timed_out = FALSE;
    err = vl53l1x_wait_data_ready(&data_ready_timed_out);
    if (FSP_SUCCESS != err) {
        if (data_ready_timed_out) {
            *p_result = VL53L1X_RESULT_DATA_READY_TIMEOUT;
        }
        return err;
    }
    err = vl53l1x_write_register(VL53L1X_REG_INTERRUPT_CLEAR, 0x01U);
    if (FSP_SUCCESS != err) {
        return err;
    }
    err = vl53l1x_write_register(VL53L1X_REG_MODE_START, 0x00U);
    if (FSP_SUCCESS != err) {
        return err;
    }
    err = vl53l1x_write_register(VL53L1X_REG_VHV_TIMEOUT, 0x09U);
    if (FSP_SUCCESS != err) {
        return err;
    }
    err = vl53l1x_write_register(VL53L1X_REG_VHV_INIT, 0x00U);
    if (FSP_SUCCESS != err) {
        return err;
    }
    err = vl53l1x_write_register(VL53L1X_REG_MODE_START, 0x40U);
    if (FSP_SUCCESS == err) {
        *p_result = VL53L1X_RESULT_VALID;
    }
    return err;
}

/** =================================================================*
 * @brief  VL53L1Xの生測距結果を取得
 * @details Range Statusが無効でもI2C転送自体が成功していれば値を返す。
 * @param[out] p_measurement 生の距離とRange Status
 * @return I2C/待機/割込みクリアが成功すればFSP_SUCCESS
 * ================================================================= */
LOCAL fsp_err_t vl53l1x_read_measurement(vl53l1x_reading_t * p_reading, BOOL * p_timed_out) {
    if ((NULL == p_reading) || (NULL == p_timed_out)) {
        return FSP_ERR_INVALID_ARGUMENT;
    }

    fsp_err_t err = vl53l1x_wait_data_ready(p_timed_out);
    if (FSP_SUCCESS != err) {
        return err;
    }

    UB range_status = 0U;
    UB distance_data[2] = {0U};
    err = vl53l1x_read_registers(VL53L1X_REG_RANGE_STATUS, &range_status, 1U);
    if (FSP_SUCCESS == err) {
        p_reading->range_status = (UB) (range_status & 0x1FU);
        err = vl53l1x_read_registers(VL53L1X_REG_DISTANCE_MM, distance_data, sizeof(distance_data));
    }

    fsp_err_t const clear_err = vl53l1x_write_register(VL53L1X_REG_INTERRUPT_CLEAR, 0x01U);
    if (FSP_SUCCESS != err) {
        return err;
    }
    if (FSP_SUCCESS != clear_err) {
        return clear_err;
    }

    p_reading->distance_mm = (UH) (((UH) distance_data[0] << 8U) | distance_data[1]);
    return FSP_SUCCESS;
}

/** =================================================================*
 * @brief  VL53L1Xの有効距離を取得
 * @param[out] p_distance_mm 距離[mm]
 * @return 有効測距ならFSP_SUCCESS、Range Status/距離範囲不正ならFSP_ERR_INVALID_DATA
 * ================================================================= */
EXPORT fsp_err_t vl53l1x_read_distance(vl53l1x_reading_t * p_reading) {
    if (NULL == p_reading) {
        return FSP_ERR_INVALID_ARGUMENT;
    }

    *p_reading = (vl53l1x_reading_t){
        .range_status = VL53L1X_RANGE_STATUS_UNAVAILABLE,
        .result = VL53L1X_RESULT_TRANSPORT_ERROR,
    };

    BOOL data_ready_timed_out = FALSE;
    fsp_err_t const err = vl53l1x_read_measurement(p_reading, &data_ready_timed_out);
    if (FSP_SUCCESS != err) {
        if (data_ready_timed_out) {
            p_reading->result = VL53L1X_RESULT_DATA_READY_TIMEOUT;
        }
        return err;
    }
    if (VL53L1X_RANGE_STATUS_VALID != p_reading->range_status) {
        p_reading->result = VL53L1X_RESULT_RANGE_STATUS_INVALID;
        return FSP_ERR_INVALID_DATA;
    }
    if ((p_reading->distance_mm < CPU0_TOF_MIN_VALID_MM) || (p_reading->distance_mm > CPU0_TOF_MAX_VALID_MM)) {
        p_reading->result = VL53L1X_RESULT_DISTANCE_INVALID;
        return FSP_ERR_INVALID_DATA;
    }

    p_reading->result = VL53L1X_RESULT_VALID;
    return FSP_SUCCESS;
}
