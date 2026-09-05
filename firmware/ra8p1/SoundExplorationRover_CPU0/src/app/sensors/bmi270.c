/** =================================================================*
 * @file   bmi270.c
 * @brief  BMI270加速度・角速度センサードライバ
 * @details PORまたはソフトリセット後にBosch SensortecのMaximum FIFO設定イメージを転送する。
 *          設定イメージのライセンスはリポジトリ直下のTHIRD_PARTY_NOTICES.mdを参照。
 * ================================================================= */
#include "bmi270.h"                                       /* BMI270公開API */
#include "sensor_i2c_bus.h"                               /* I2CバスAPI */
#include "../../cpu0_config.h"                            /* BMI270設定値 */

#define BMI270_ADDRESS_SDO_LOW              (0x68U)
#define BMI270_ADDRESS_SDO_HIGH             (0x69U)

#define BMI270_REG_CHIP_ID                  (0x00U)
#define BMI270_REG_ACCEL_DATA               (0x0CU)
#define BMI270_REG_INTERNAL_STATUS           (0x21U)
#define BMI270_REG_ACCEL_CONFIG             (0x40U)
#define BMI270_REG_ACCEL_RANGE              (0x41U)
#define BMI270_REG_GYRO_CONFIG              (0x42U)
#define BMI270_REG_GYRO_RANGE               (0x43U)
#define BMI270_REG_INIT_CTRL                (0x59U)
#define BMI270_REG_INIT_ADDR_0              (0x5BU)
#define BMI270_REG_INIT_ADDR_1              (0x5CU)
#define BMI270_REG_INIT_DATA                (0x5EU)
#define BMI270_REG_POWER_CONFIG             (0x7CU)
#define BMI270_REG_POWER_CONTROL            (0x7DU)
#define BMI270_REG_COMMAND                  (0x7EU)

#define BMI270_CHIP_ID                      (0x24U)
#define BMI270_COMMAND_SOFT_RESET           (0xB6U)
#define BMI270_INTERNAL_STATUS_INIT_OK       (0x01U)
#define BMI270_INTERNAL_STATUS_MESSAGE_MASK (0x0FU)

#define BMI270_ACCEL_CONFIG_100HZ           (0xA8U)
#define BMI270_ACCEL_RANGE_4G               (0x01U)
#define BMI270_GYRO_CONFIG_100HZ            (0xA8U)
#define BMI270_GYRO_RANGE_500DPS            (0x02U)
#define BMI270_POWER_CONTROL_ACCEL_GYRO     (0x06U)

/* INIT_ADDRはワード単位のため、設定イメージを偶数byteずつ転送する。 */
#define BMI270_CONFIG_WRITE_CHUNK            (32U)
#define BMI270_CONFIG_STARTUP_DELAY_MS       (30U)

/**< BMI270初期化時に転送するBosch Sensortec Maximum FIFO設定イメージ */
/*
 * Bosch Sensortec BMI270 SensorAPI bmi270_maximum_fifo_config_file。
 * Copyright (c) 2023 Bosch Sensortec GmbH. BSD-3-Clause.
 */
LOCAL const UB bmi270_maximum_fifo_config_file[] = {
    0xc8, 0x2e, 0x00, 0x2e, 0x80, 0x2e, 0x1a, 0x00, 0xc8, 0x2e, 0x00, 0x2e, 0xc8, 0x2e, 0x00, 0x2e, 0xc8, 0x2e, 0x00,
    0x2e, 0xc8, 0x2e, 0x00, 0x2e, 0xc8, 0x2e, 0x00, 0x2e, 0xc8, 0x2e, 0x00, 0x2e, 0x90, 0x32, 0x21, 0x2e, 0x59, 0xf5,
    0x10, 0x30, 0x21, 0x2e, 0x6a, 0xf5, 0x1a, 0x24, 0x22, 0x00, 0x80, 0x2e, 0x3b, 0x00, 0xc8, 0x2e, 0x44, 0x47, 0x22,
    0x00, 0x37, 0x00, 0xa4, 0x00, 0xff, 0x0f, 0xd1, 0x00, 0x07, 0xad, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1,
    0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00,
    0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x11, 0x24, 0xfc, 0xf5, 0x80, 0x30, 0x40, 0x42, 0x50, 0x50, 0x00, 0x30, 0x12, 0x24, 0xeb,
    0x00, 0x03, 0x30, 0x00, 0x2e, 0xc1, 0x86, 0x5a, 0x0e, 0xfb, 0x2f, 0x21, 0x2e, 0xfc, 0xf5, 0x13, 0x24, 0x63, 0xf5,
    0xe0, 0x3c, 0x48, 0x00, 0x22, 0x30, 0xf7, 0x80, 0xc2, 0x42, 0xe1, 0x7f, 0x3a, 0x25, 0xfc, 0x86, 0xf0, 0x7f, 0x41,
    0x33, 0x98, 0x2e, 0xc2, 0xc4, 0xd6, 0x6f, 0xf1, 0x30, 0xf1, 0x08, 0xc4, 0x6f, 0x11, 0x24, 0xff, 0x03, 0x12, 0x24,
    0x00, 0xfc, 0x61, 0x09, 0xa2, 0x08, 0x36, 0xbe, 0x2a, 0xb9, 0x13, 0x24, 0x38, 0x00, 0x64, 0xbb, 0xd1, 0xbe, 0x94,
    0x0a, 0x71, 0x08, 0xd5, 0x42, 0x21, 0xbd, 0x91, 0xbc, 0xd2, 0x42, 0xc1, 0x42, 0x00, 0xb2, 0xfe, 0x82, 0x05, 0x2f,
    0x50, 0x30, 0x21, 0x2e, 0x21, 0xf2, 0x00, 0x2e, 0x00, 0x2e, 0xd0, 0x2e, 0xf0, 0x6f, 0x02, 0x30, 0x02, 0x42, 0x20,
    0x26, 0xe0, 0x6f, 0x02, 0x31, 0x03, 0x40, 0x9a, 0x0a, 0x02, 0x42, 0xf0, 0x37, 0x05, 0x2e, 0x5e, 0xf7, 0x10, 0x08,
    0x12, 0x24, 0x1e, 0xf2, 0x80, 0x42, 0x83, 0x84, 0xf1, 0x7f, 0x0a, 0x25, 0x13, 0x30, 0x83, 0x42, 0x3b, 0x82, 0xf0,
    0x6f, 0x00, 0x2e, 0x00, 0x2e, 0xd0, 0x2e, 0x12, 0x40, 0x52, 0x42, 0x00, 0x2e, 0x12, 0x40, 0x52, 0x42, 0x3e, 0x84,
    0x00, 0x40, 0x40, 0x42, 0x7e, 0x82, 0xe1, 0x7f, 0xf2, 0x7f, 0x98, 0x2e, 0x6a, 0xd6, 0x21, 0x30, 0x23, 0x2e, 0x61,
    0xf5, 0xeb, 0x2c, 0xe1, 0x6f
};

LOCAL UB bmi270_address;                                   /**< 検出済みBMI270 7bitアドレス */
LOCAL BOOL bmi270_initialized;                             /**< 初期化完了状態 */

LOCAL fsp_err_t bmi270_write_register(UB register_address, UB value);
LOCAL fsp_err_t bmi270_write_registers(UB register_address, const UB * p_data, UW length);
LOCAL fsp_err_t bmi270_read_registers(UB register_address, UB * p_data, UW length);
LOCAL fsp_err_t bmi270_load_config(void);
LOCAL H bmi270_little_endian_i16(const UB * p_data);

/** =================================================================*
 * @brief  BMI270の8bitレジスタ書込み
 * @param[in] register_address レジスタアドレス
 * @param[in] value 書込み値
 * @return FSPエラーコード
 * ================================================================= */
LOCAL fsp_err_t bmi270_write_register(UB register_address, UB value) {
    UB const data[2] = {register_address, value};
    fsp_err_t const err = sensor_i2c_bus_write(bmi270_address, data, sizeof(data));
    if (FSP_SUCCESS == err) {
        /* 設定モードのI2C書込み後に必要な450us以上のアイドル時間を確保する。 */
        R_BSP_SoftwareDelay(1U, BSP_DELAY_UNITS_MILLISECONDS);
    }
    return err;
}

/** =================================================================*
 * @brief  BMI270レジスタへの連続書込み
 * @param[in] register_address 先頭レジスタアドレス
 * @param[in] p_data 書込みデータ
 * @param[in] length 書込み長[byte]
 * @return FSPエラーコード
 * ================================================================= */
LOCAL fsp_err_t bmi270_write_registers(UB register_address, const UB * p_data, UW length) {
    if ((NULL == p_data) || (0U == length) || (length > BMI270_CONFIG_WRITE_CHUNK)) {
        return FSP_ERR_INVALID_ARGUMENT;
    }

    UB packet[1U + BMI270_CONFIG_WRITE_CHUNK];
    packet[0] = register_address;
    for (UW index = 0U; index < length; index++) {
        packet[1U + index] = p_data[index];
    }
    fsp_err_t const err = sensor_i2c_bus_write(bmi270_address, packet, length + 1U);
    if (FSP_SUCCESS == err) {
        /* 設定モードのI2C書込み後に必要な450us以上のアイドル時間を確保する。 */
        R_BSP_SoftwareDelay(1U, BSP_DELAY_UNITS_MILLISECONDS);
    }
    return err;
}

/** =================================================================*
 * @brief  BMI270レジスタの連続読出し
 * @param[in] register_address 先頭レジスタアドレス
 * @param[out] p_data 読出し先
 * @param[in] length 読出し長[byte]
 * @return FSPエラーコード
 * ================================================================= */
LOCAL fsp_err_t bmi270_read_registers(UB register_address, UB * p_data, UW length) {
    return sensor_i2c_bus_write_read(bmi270_address, &register_address, 1U, p_data, length);
}

/** =================================================================*
 * @brief  little endian 16bit値の符号付き変換
 * @param[in] p_data 下位byteから始まるデータ
 * @return 符号付き16bit値
 * ================================================================= */
LOCAL H bmi270_little_endian_i16(const UB * p_data) {
    return (H) (UH) (((UH) p_data[1] << 8) | p_data[0]);
}

/** =================================================================*
 * @brief  BMI270設定イメージ転送
 * @details Maximum FIFO設定を分割転送し、内部初期化状態が正常になるまで待つ。
 * @return FSPエラーコード
 * ================================================================= */
LOCAL fsp_err_t bmi270_load_config(void) {
    fsp_err_t err = bmi270_write_register(BMI270_REG_POWER_CONFIG, 0x00U);
    if (FSP_SUCCESS != err) {
        return err;
    }
    err = bmi270_write_register(BMI270_REG_INIT_CTRL, 0x00U);
    if (FSP_SUCCESS != err) {
        return err;
    }

    UW offset = 0U;
    while (offset < (UW) sizeof(bmi270_maximum_fifo_config_file)) {
        UW remaining = (UW) sizeof(bmi270_maximum_fifo_config_file) - offset;
        UW chunk = (remaining > BMI270_CONFIG_WRITE_CHUNK) ? BMI270_CONFIG_WRITE_CHUNK : remaining;
        if (0U != (chunk & 1U)) {
            /* Bosch設定イメージは偶数byteであるため、不正データとして扱う。 */
            return FSP_ERR_INVALID_DATA;
        }

        UW const word_address = offset / 2U;
        err = bmi270_write_register(BMI270_REG_INIT_ADDR_0, (UB) (word_address & 0x0FU));
        if (FSP_SUCCESS == err) {
            err = bmi270_write_register(BMI270_REG_INIT_ADDR_1, (UB) ((word_address >> 4U) & 0xFFU));
        }
        if (FSP_SUCCESS == err) {
            err = bmi270_write_registers(BMI270_REG_INIT_DATA, &bmi270_maximum_fifo_config_file[offset], chunk);
        }
        if (FSP_SUCCESS != err) {
            return err;
        }
        offset += chunk;
    }

    err = bmi270_write_register(BMI270_REG_INIT_CTRL, 0x01U);
    if (FSP_SUCCESS != err) {
        return err;
    }

    R_BSP_SoftwareDelay(BMI270_CONFIG_STARTUP_DELAY_MS, BSP_DELAY_UNITS_MILLISECONDS);
    UB internal_status = 0U;
    err = bmi270_read_registers(BMI270_REG_INTERNAL_STATUS, &internal_status, 1U);
    if (FSP_SUCCESS != err) {
        return err;
    }

    if (BMI270_INTERNAL_STATUS_INIT_OK != (internal_status & BMI270_INTERNAL_STATUS_MESSAGE_MASK)) {
        return FSP_ERR_INVALID_DATA;
    }

    return FSP_SUCCESS;
}

/** =================================================================*
 * @brief  BMI270初期化
 * @details I2Cアドレスを検出し、設定イメージ転送後に加速度・角速度出力を有効化する。
 * @return FSPエラーコード
 * ================================================================= */
EXPORT fsp_err_t bmi270_init(void) {
    UB chip_id = 0U;
    fsp_err_t err = FSP_ERR_NOT_FOUND;
    UB const candidate_addresses[] = {BMI270_ADDRESS_SDO_LOW, BMI270_ADDRESS_SDO_HIGH};

    bmi270_initialized = FALSE;
    bmi270_address = 0U;

    for (UW index = 0U; index < (UW) (sizeof(candidate_addresses) / sizeof(candidate_addresses[0])); index++) {
        bmi270_address = candidate_addresses[index];
        err = bmi270_read_registers(BMI270_REG_CHIP_ID, &chip_id, 1U);
        if ((FSP_SUCCESS == err) && (BMI270_CHIP_ID == chip_id)) {
            break;
        }
    }
    if ((FSP_SUCCESS != err) || (BMI270_CHIP_ID != chip_id)) {
        bmi270_address = 0U;
        return FSP_ERR_NOT_FOUND;
    }

    err = bmi270_write_register(BMI270_REG_COMMAND, BMI270_COMMAND_SOFT_RESET);
    if (FSP_SUCCESS != err) {
        return err;
    }
    R_BSP_SoftwareDelay(CPU0_BMI270_RESET_DELAY_MS, BSP_DELAY_UNITS_MILLISECONDS);

    err = bmi270_read_registers(BMI270_REG_CHIP_ID, &chip_id, 1U);
    if ((FSP_SUCCESS != err) || (BMI270_CHIP_ID != chip_id)) {
        return (FSP_SUCCESS == err) ? FSP_ERR_NOT_FOUND : err;
    }

    err = bmi270_load_config();
    if (FSP_SUCCESS != err) {
        return err;
    }

    err = bmi270_write_register(BMI270_REG_POWER_CONFIG, 0x00U);
    if (FSP_SUCCESS == err) {
        err = bmi270_write_register(BMI270_REG_ACCEL_CONFIG, BMI270_ACCEL_CONFIG_100HZ);
    }
    if (FSP_SUCCESS == err) {
        err = bmi270_write_register(BMI270_REG_ACCEL_RANGE, BMI270_ACCEL_RANGE_4G);
    }
    if (FSP_SUCCESS == err) {
        err = bmi270_write_register(BMI270_REG_GYRO_CONFIG, BMI270_GYRO_CONFIG_100HZ);
    }
    if (FSP_SUCCESS == err) {
        err = bmi270_write_register(BMI270_REG_GYRO_RANGE, BMI270_GYRO_RANGE_500DPS);
    }
    if (FSP_SUCCESS == err) {
        err = bmi270_write_register(BMI270_REG_POWER_CONTROL, BMI270_POWER_CONTROL_ACCEL_GYRO);
    }
    if (FSP_SUCCESS != err) {
        return err;
    }

    R_BSP_SoftwareDelay(CPU0_BMI270_STARTUP_DELAY_MS, BSP_DELAY_UNITS_MILLISECONDS);
    bmi270_initialized = TRUE;
    return FSP_SUCCESS;
}

/** =================================================================*
 * @brief  BMI270 raw加速度・角速度取得
 * @param[out] p_data raw 6軸値
 * @return FSPエラーコード
 * ================================================================= */
EXPORT fsp_err_t bmi270_read_raw(bmi270_raw_data_t * p_data) {
    if (NULL == p_data) {
        return FSP_ERR_INVALID_ARGUMENT;
    }
    if (!bmi270_initialized) {
        return FSP_ERR_NOT_OPEN;
    }

    /* 0x0C～0x17はACC XYZ、GYR XYZの順に連続している。 */
    UB data[12] = {0U};
    fsp_err_t const err = bmi270_read_registers(BMI270_REG_ACCEL_DATA, data, sizeof(data));
    if (FSP_SUCCESS != err) {
        return err;
    }

    for (UW axis = 0U; axis < 3U; axis++) {
        p_data->accel[axis] = bmi270_little_endian_i16(&data[axis * 2U]);
        p_data->gyro[axis] = bmi270_little_endian_i16(&data[6U + (axis * 2U)]);
    }
    return FSP_SUCCESS;
}
