/** =================================================================*
 * @file   sensor_hub.c
 * @brief  ToF 3台とBMI270をまとめるCPU0センサー層実装
 * ================================================================= */
#include "sensor_hub.h"                                    /* センサー共通API */
#include "bmi270.h"                                        /* BMI270 rawデータAPI */
#include "sensor_i2c_bus.h"                                /* I2CバスAPI */
#include "tca9548a.h"                                      /* I2CマルチプレクサAPI */
#include "vl53l1x.h"                                       /* ToF API */
#include "../../cpu0_config.h"                              /* センサー設定値 */

LOCAL BOOL sensor_hub_initialized;                          /**< 全センサー初期化状態 */
LOCAL BOOL sensor_hub_filter_valid[CPU0_SENSOR_TOF_COUNT];  /**< ToFフィルタ初期値状態 */
LOCAL UH sensor_hub_filtered_distance_mm[CPU0_SENSOR_TOF_COUNT]; /**< ToF平滑化値 */

LOCAL UB sensor_hub_tof_channel(cpu0_tof_position_t position); /* ToF位置からTCAチャネル取得 */
LOCAL UB sensor_hub_tof_valid_flag(cpu0_tof_position_t position); /* ToF位置のvalid bit取得 */
LOCAL UW sensor_hub_tof_error_flag(cpu0_tof_position_t position); /* ToF位置のerror bit取得 */
LOCAL fsp_err_t sensor_hub_read_tof(cpu0_tof_position_t position, UH * p_distance_mm); /* 1台読出し */
LOCAL fsp_err_t sensor_hub_read_bmi270(bmi270_raw_data_t * p_raw); /* CH3選択後にIMU読出し */
LOCAL H sensor_hub_scale_accel_mg(H raw);                   /* raw accelをmgへ変換 */
LOCAL H sensor_hub_scale_gyro_dps_x10(H raw);               /* raw gyroを0.1dpsへ変換 */

/** =================================================================*
 * @brief  ToF位置に対応するTCA9548Aチャネルを取得
 * @param[in] position ToF配置
 * @return TCA9548Aチャネル番号
 * ================================================================= */
LOCAL UB sensor_hub_tof_channel(cpu0_tof_position_t position) {
    switch (position) {
    case CPU0_TOF_LEFT:
        return CPU0_TCA9548A_CHANNEL_LEFT;

    case CPU0_TOF_CENTER:
        return CPU0_TCA9548A_CHANNEL_CENTER;

    case CPU0_TOF_RIGHT:
    default:
        return CPU0_TCA9548A_CHANNEL_RIGHT;
    }
}

/** =================================================================*
 * @brief  ToF位置に対応するvalid bitを取得
 * @param[in] position ToF配置
 * @return スナップショットのvalid bit
 * ================================================================= */
LOCAL UB sensor_hub_tof_valid_flag(cpu0_tof_position_t position) {
    switch (position) {
    case CPU0_TOF_LEFT:
        return CPU0_SENSOR_VALID_TOF_LEFT;

    case CPU0_TOF_CENTER:
        return CPU0_SENSOR_VALID_TOF_CENTER;

    case CPU0_TOF_RIGHT:
    default:
        return CPU0_SENSOR_VALID_TOF_RIGHT;
    }
}

/** =================================================================*
 * @brief  ToF位置に対応するerror bitを取得
 * @param[in] position ToF配置
 * @return スナップショットのerror bit
 * ================================================================= */
LOCAL UW sensor_hub_tof_error_flag(cpu0_tof_position_t position) {
    switch (position) {
    case CPU0_TOF_LEFT:
        return CPU0_SENSOR_ERROR_TOF_LEFT;

    case CPU0_TOF_CENTER:
        return CPU0_SENSOR_ERROR_TOF_CENTER;

    case CPU0_TOF_RIGHT:
    default:
        return CPU0_SENSOR_ERROR_TOF_RIGHT;
    }
}

/** =================================================================*
 * @brief  指定位置のToF 1台から距離を取得
 * @param[in] position ToF配置
 * @param[out] p_distance_mm 距離[mm]
 * @return FSPエラーコード
 * ================================================================= */
LOCAL fsp_err_t sensor_hub_read_tof(cpu0_tof_position_t position, UH * p_distance_mm) {
    fsp_err_t const select_err = tca9548a_select_channel(sensor_hub_tof_channel(position));
    if (FSP_SUCCESS != select_err) {
        return select_err;
    }

    return vl53l1x_read_distance(p_distance_mm);
}

/** =================================================================*
 * @brief  BMI270からraw accel/gyroを取得
 * @details 通信直前にTCA9548A CH3だけを排他的に接続する。
 * @param[out] p_raw BMI270 rawデータ
 * @return FSPエラーコード
 * ================================================================= */
LOCAL fsp_err_t sensor_hub_read_bmi270(bmi270_raw_data_t * p_raw) {
    fsp_err_t const select_err = tca9548a_select_channel(CPU0_TCA9548A_CHANNEL_BMI270);
    if (FSP_SUCCESS != select_err) {
        return select_err;
    }

    return bmi270_read_raw(p_raw);
}

/** =================================================================*
 * @brief  BMI270の4g raw accelerationをmgへ変換
 * @param[in] raw BMI270 raw値
 * @return acceleration[mg]
 * ================================================================= */
LOCAL H sensor_hub_scale_accel_mg(H raw) {
    return (H) (((W) raw * 1000) / CPU0_BMI270_ACCEL_LSB_PER_G);
}

/** =================================================================*
 * @brief  BMI270の500dps raw gyroを0.1dpsへ変換
 * @param[in] raw BMI270 raw値
 * @return gyro[0.1dps]
 * ================================================================= */
LOCAL H sensor_hub_scale_gyro_dps_x10(H raw) {
    return (H) (((W) raw * CPU0_BMI270_GYRO_RANGE_DPS_X10) / 32768);
}

/** =================================================================*
 * @brief  I2C、TCA9548A、ToF 3台、BMI270を初期化
 * @return FSPエラーコード
 * ================================================================= */
EXPORT fsp_err_t sensor_hub_init(void) {
    sensor_hub_initialized = FALSE;
    for (UW index = 0U; index < CPU0_SENSOR_TOF_COUNT; index++) {
        sensor_hub_filter_valid[index] = FALSE;
        sensor_hub_filtered_distance_mm[index] = 0U;
    }

    fsp_err_t err = sensor_i2c_bus_init();
    if (FSP_SUCCESS != err) {
        return err;
    }

    err = tca9548a_disable_all();
    if (FSP_SUCCESS != err) {
        sensor_hub_deinit();
        return err;
    }

    for (cpu0_tof_position_t position = CPU0_TOF_LEFT; position <= CPU0_TOF_RIGHT; position++) {
        err = tca9548a_select_channel(sensor_hub_tof_channel(position));
        if (FSP_SUCCESS == err) {
            err = vl53l1x_init();
        }
        if (FSP_SUCCESS != err) {
            sensor_hub_deinit();
            return err;
        }
    }

    /* BMI270の初期化直前にCH3だけを排他的に接続する。 */
    err = tca9548a_select_channel(CPU0_TCA9548A_CHANNEL_BMI270);
    if (FSP_SUCCESS == err) {
        err = bmi270_init();
    }
    if (FSP_SUCCESS != err) {
        sensor_hub_deinit();
        return err;
    }

    sensor_hub_initialized = TRUE;
    return FSP_SUCCESS;
}

/** =================================================================*
 * @brief  センサーとI2Cを停止
 * ================================================================= */
EXPORT void sensor_hub_deinit(void) {
    (void) tca9548a_disable_all();
    sensor_i2c_bus_deinit();
    sensor_hub_initialized = FALSE;
}

/** =================================================================*
 * @brief  ToF 3台とBMI270を1周期分取得
 * @param[out] p_snapshot 取得結果
 * @return 全センサーが有効ならFSP_SUCCESS、それ以外は最初のエラー
 * ================================================================= */
EXPORT fsp_err_t sensor_hub_poll(cpu0_sensor_snapshot_t * p_snapshot) {
    if (NULL == p_snapshot) {
        return FSP_ERR_INVALID_ARGUMENT;
    }
    if (!sensor_hub_initialized) {
        return FSP_ERR_NOT_OPEN;
    }

    *p_snapshot = (cpu0_sensor_snapshot_t){
        .age_ms = 0U,
        .last_error = (W) FSP_SUCCESS,
        .initialized = TRUE,
    };
    fsp_err_t first_error = FSP_SUCCESS;

    for (cpu0_tof_position_t position = CPU0_TOF_LEFT; position <= CPU0_TOF_RIGHT; position++) {
        UH distance_mm = 0U;
        fsp_err_t const err = sensor_hub_read_tof(position, &distance_mm);
        if (FSP_SUCCESS != err) {
            p_snapshot->error_flags |= sensor_hub_tof_error_flag(position);
            p_snapshot->last_error = (W) err;
            if (FSP_SUCCESS == first_error) {
                first_error = err;
            }
            continue;
        }

        UW const index = (UW) position;
        if (sensor_hub_filter_valid[index]) {
            sensor_hub_filtered_distance_mm[index] =
                (UH) (((3U * sensor_hub_filtered_distance_mm[index]) + distance_mm) / 4U);
        } else {
            sensor_hub_filtered_distance_mm[index] = distance_mm;
            sensor_hub_filter_valid[index] = TRUE;
        }
        p_snapshot->tof_distance_mm[index] = sensor_hub_filtered_distance_mm[index];
        p_snapshot->valid_flags |= sensor_hub_tof_valid_flag(position);
    }

    bmi270_raw_data_t raw_imu = {0};
    fsp_err_t const imu_err = sensor_hub_read_bmi270(&raw_imu);
    if (FSP_SUCCESS != imu_err) {
        p_snapshot->error_flags |= CPU0_SENSOR_ERROR_BMI270;
        p_snapshot->last_error = (W) imu_err;
        if (FSP_SUCCESS == first_error) {
            first_error = imu_err;
        }
    } else {
        for (UW axis = 0U; axis < 3U; axis++) {
            p_snapshot->accel_mg[axis] = sensor_hub_scale_accel_mg(raw_imu.accel[axis]);
            p_snapshot->gyro_dps_x10[axis] = sensor_hub_scale_gyro_dps_x10(raw_imu.gyro[axis]);
        }
        p_snapshot->valid_flags |= CPU0_SENSOR_VALID_IMU;
    }

    if (CPU0_SENSOR_VALID_ALL != p_snapshot->valid_flags) {
        return (FSP_SUCCESS == first_error) ? FSP_ERR_INVALID_DATA : first_error;
    }

    return FSP_SUCCESS;
}
