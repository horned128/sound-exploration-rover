/** =================================================================*
 * @file   bmi270.c
 * @brief  BMI270加速度・ジャイロセンサー実装
 * ================================================================= */
#include "bmi270.h"                                        /* BMI270 API */
#include "sensor_i2c_bus.h"                                /* I2CバスAPI */
#include "../../cpu0_config.h"                              /* BMI270初期化待ち時間 */

#define BMI270_ADDRESS_SDO_LOW             (0x68U)
#define BMI270_ADDRESS_SDO_HIGH            (0x69U)
#define BMI270_REG_CHIP_ID                 (0x00U)
#define BMI270_REG_GYRO_DATA               (0x0CU)
#define BMI270_REG_ACCEL_DATA              (0x12U)
#define BMI270_REG_ACCEL_CONFIG            (0x40U)
#define BMI270_REG_ACCEL_RANGE             (0x41U)
#define BMI270_REG_GYRO_CONFIG             (0x42U)
#define BMI270_REG_GYRO_RANGE              (0x43U)
#define BMI270_REG_POWER_CONFIG            (0x7CU)
#define BMI270_REG_POWER_CONTROL           (0x7DU)
#define BMI270_REG_COMMAND                 (0x7EU)
#define BMI270_CHIP_ID                     (0x24U)
#define BMI270_COMMAND_SOFT_RESET          (0xB6U)
#define BMI270_ACCEL_CONFIG_100HZ          (0xA8U)
#define BMI270_ACCEL_RANGE_4G              (0x01U)
#define BMI270_GYRO_CONFIG_100HZ           (0xA8U)
#define BMI270_GYRO_RANGE_500DPS           (0x02U)
#define BMI270_POWER_CONTROL_ACCEL_GYRO    (0x0EU)

LOCAL UB bmi270_address;                                   /**< 検出済みBMI270 7bitアドレス */
LOCAL BOOL bmi270_initialized;                             /**< 初期化完了状態 */

LOCAL fsp_err_t bmi270_write_register(UB register_address, UB value); /* 8bitレジスタ書込み */
LOCAL fsp_err_t bmi270_read_registers(UB register_address, UB * p_data, UW length); /* 連続読出し */
LOCAL H bmi270_little_endian_i16(const UB * p_data);       /* little endian signed 16bit変換 */

/** =================================================================*
 * @brief  BMI270の8bitレジスタへ書込み
 * @param[in] register_address レジスタアドレス
 * @param[in] value 書込み値
 * @return FSPエラーコード
 * ================================================================= */
LOCAL fsp_err_t bmi270_write_register(UB register_address, UB value) {
    UB const data[2] = {register_address, value};
    return sensor_i2c_bus_write(bmi270_address, data, sizeof(data));
}

/** =================================================================*
 * @brief  BMI270レジスタを連続読出し
 * @param[in] register_address レジスタアドレス
 * @param[out] p_data 読出し先
 * @param[in] length 読出し長[byte]
 * @return FSPエラーコード
 * ================================================================= */
LOCAL fsp_err_t bmi270_read_registers(UB register_address, UB * p_data, UW length) {
    return sensor_i2c_bus_write_read(bmi270_address, &register_address, 1U, p_data, length);
}

/** =================================================================*
 * @brief  little endian 16bit値を符号付きへ変換
 * @param[in] p_data 下位byteから始まるデータ
 * @return 符号付き16bit値
 * ================================================================= */
LOCAL H bmi270_little_endian_i16(const UB * p_data) {
    return (H) (UH) (((UH) p_data[1] << 8) | p_data[0]);
}

/** =================================================================*
 * @brief  BMI270のI2Cアドレスを検出しraw accel/gyro出力を開始
 * @details feature engineは使わず、障害物回避に必要なraw 6軸データだけを有効化する。
 * @return FSPエラーコード
 * ================================================================= */
EXPORT fsp_err_t bmi270_init(void) {
    UB chip_id = 0U;
    fsp_err_t err = FSP_ERR_NOT_FOUND;
    UB const candidate_addresses[] = {BMI270_ADDRESS_SDO_LOW, BMI270_ADDRESS_SDO_HIGH};

    bmi270_initialized = FALSE;
    for (UW index = 0U; index < (UW) (sizeof(candidate_addresses) / sizeof(candidate_addresses[0])); index++) {
        bmi270_address = candidate_addresses[index];
        err = bmi270_read_registers(BMI270_REG_CHIP_ID, &chip_id, 1U);
        if ((FSP_SUCCESS == err) && (BMI270_CHIP_ID == chip_id)) {
            break;
        }
    }
    if ((FSP_SUCCESS != err) || (BMI270_CHIP_ID != chip_id)) {
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
 * @brief  BMI270のraw accel/gyroを取得
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

    UB gyro_data[6] = {0U};
    UB accel_data[6] = {0U};
    fsp_err_t err = bmi270_read_registers(BMI270_REG_GYRO_DATA, gyro_data, sizeof(gyro_data));
    if (FSP_SUCCESS == err) {
        err = bmi270_read_registers(BMI270_REG_ACCEL_DATA, accel_data, sizeof(accel_data));
    }
    if (FSP_SUCCESS != err) {
        return err;
    }

    for (UW axis = 0U; axis < 3U; axis++) {
        p_data->gyro[axis] = bmi270_little_endian_i16(&gyro_data[axis * 2U]);
        p_data->accel[axis] = bmi270_little_endian_i16(&accel_data[axis * 2U]);
    }
    return FSP_SUCCESS;
}
