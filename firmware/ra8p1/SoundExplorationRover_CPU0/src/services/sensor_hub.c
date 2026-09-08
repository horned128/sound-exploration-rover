/** =================================================================*
 * @file   sensor_hub.c
 * @brief  ToF 3台とBMI270をまとめるCPU0センサー層実装
 * ================================================================= */
#include "sensor_hub.h"                                    /* センサー共通API */
#include "drivers/bmi270.h"                                /* BMI270 rawデータAPI */
#include "drivers/tca9548a.h"                              /* I2CマルチプレクサAPI */
#include "drivers/vl53l1x.h"                               /* ToF API */
#include "platform/i2c_bus.h"                              /* I2CバスAPI */
#include "config/sensor_config.h"                          /* センサー設定値 */

LOCAL BOOL sensor_hub_initialized;                          /**< 全センサー初期化状態 */
LOCAL BOOL sensor_hub_filter_valid[CPU0_SENSOR_TOF_COUNT];  /**< ToFフィルタ初期値状態 */
LOCAL UH sensor_hub_filtered_distance_mm[CPU0_SENSOR_TOF_COUNT]; /**< ToF平滑化値 */
LOCAL sensor_diagnostics_t sensor_hub_diagnostics;     /**< 最終センサー診断 */

LOCAL UB sensor_hub_tof_channel(tof_position_t position); /* ToF位置からTCAチャネル取得 */
LOCAL UB sensor_hub_tof_valid_flag(tof_position_t position); /* ToF位置のvalid bit取得 */
LOCAL UW sensor_hub_tof_error_flag(tof_position_t position); /* ToF位置のerror bit取得 */
LOCAL sensor_failure_device_t sensor_hub_tof_device(tof_position_t position); /* ToF device取得 */
LOCAL sensor_failure_stage_t sensor_hub_tof_select_stage(tof_position_t position); /* ToF選択stage取得 */
LOCAL sensor_failure_stage_t sensor_hub_tof_init_stage(tof_position_t position); /* ToF初期化stage取得 */
LOCAL sensor_failure_stage_t sensor_hub_tof_read_stage(tof_position_t position); /* ToF読出しstage取得 */
LOCAL void sensor_hub_diagnostics_reset(sensor_diagnostics_t * p_diagnostics); /* 診断初期化 */
LOCAL void sensor_hub_record_failure(sensor_snapshot_t * p_snapshot, sensor_failure_kind_t kind,
                                     sensor_failure_device_t device, sensor_failure_stage_t stage,
                                     B channel, fsp_err_t error); /* 最初の失敗を記録 */
LOCAL sensor_failure_kind_t sensor_hub_transport_failure_kind(fsp_err_t error); /* 転送失敗種別取得 */
LOCAL H sensor_hub_scale_accel_mg(H raw);                   /* raw accelをmgへ変換 */
LOCAL H sensor_hub_scale_gyro_dps_x10(H raw);               /* raw gyroを0.1dpsへ変換 */

/** =================================================================*
 * @brief  ToF位置に対応するTCA9548Aチャネルを取得
 * @param[in] position ToF配置
 * @return TCA9548Aチャネル番号
 * ================================================================= */
LOCAL UB sensor_hub_tof_channel(tof_position_t position) {
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
LOCAL UB sensor_hub_tof_valid_flag(tof_position_t position) {
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
LOCAL UW sensor_hub_tof_error_flag(tof_position_t position) {
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
 * @brief  ToF位置に対応する診断deviceを取得
 * @param[in] position ToF配置
 * @return センサー診断device
 * ================================================================= */
LOCAL sensor_failure_device_t sensor_hub_tof_device(tof_position_t position) {
    switch (position) {
    case CPU0_TOF_LEFT:
        return CPU0_SENSOR_FAILURE_DEVICE_TOF_LEFT;
    case CPU0_TOF_CENTER:
        return CPU0_SENSOR_FAILURE_DEVICE_TOF_CENTER;
    case CPU0_TOF_RIGHT:
    default:
        return CPU0_SENSOR_FAILURE_DEVICE_TOF_RIGHT;
    }
}

/** =================================================================*
 * @brief  ToF位置に対応する診断stageを取得
 * @param[in] position ToF配置
 * @param[in] base 左ToFのstage
 * @return センサー診断stage
 * ================================================================= */
LOCAL sensor_failure_stage_t sensor_hub_tof_stage(tof_position_t position,
                                                        sensor_failure_stage_t left_stage) {
    return (sensor_failure_stage_t) (left_stage + (UW) position * 2U);
}

/** =================================================================*
 * @brief  ToFチャネル選択の診断stageを取得
 * @param[in] position ToF配置
 * @return センサー診断stage
 * ================================================================= */
LOCAL sensor_failure_stage_t sensor_hub_tof_select_stage(tof_position_t position) {
    return sensor_hub_tof_stage(position, CPU0_SENSOR_FAILURE_STAGE_TOF_LEFT_TCA_SELECT);
}

/** =================================================================*
 * @brief  ToF初期化の診断stageを取得
 * @param[in] position ToF配置
 * @return センサー診断stage
 * ================================================================= */
LOCAL sensor_failure_stage_t sensor_hub_tof_init_stage(tof_position_t position) {
    return sensor_hub_tof_stage(position, CPU0_SENSOR_FAILURE_STAGE_TOF_LEFT_INIT);
}

/** =================================================================*
 * @brief  ToF読出しの診断stageを取得
 * @param[in] position ToF配置
 * @return センサー診断stage
 * ================================================================= */
LOCAL sensor_failure_stage_t sensor_hub_tof_read_stage(tof_position_t position) {
    return (sensor_failure_stage_t) (CPU0_SENSOR_FAILURE_STAGE_TOF_LEFT_READ + (UW) position);
}

/** =================================================================*
 * @brief  センサー診断を初期化
 * @param[out] p_diagnostics 初期化先
 * ================================================================= */
LOCAL void sensor_hub_diagnostics_reset(sensor_diagnostics_t * p_diagnostics) {
    if (NULL == p_diagnostics) {
        return;
    }

    *p_diagnostics = (sensor_diagnostics_t){
        .failure_kind = CPU0_SENSOR_FAILURE_NONE,
        .failure_device = CPU0_SENSOR_FAILURE_DEVICE_NONE,
        .failure_stage = CPU0_SENSOR_FAILURE_STAGE_NONE,
        .failure_channel = CPU0_SENSOR_FAILURE_CHANNEL_NONE,
    };
    for (UW index = 0U; index < CPU0_SENSOR_TOF_COUNT; index++) {
        p_diagnostics->tof_range_status[index] = SENSOR_TOF_RANGE_STATUS_UNAVAILABLE;
        p_diagnostics->tof_result[index] = (UB) SENSOR_TOF_RESULT_TRANSPORT_ERROR;
    }
}

/** =================================================================*
 * @brief  最初に発生したセンサー失敗を記録
 * @details 同一周期の最初の失敗を保持し、Live Watchで原因を安定して追跡する。
 * @param[in,out] p_snapshot 診断を書き込むスナップショット
 * @param[in] kind 失敗分類
 * @param[in] device 失敗したデバイス
 * @param[in] stage 失敗した処理段階
 * @param[in] channel TCAチャネル
 * @param[in] error FSPエラーコード
 * ================================================================= */
LOCAL void sensor_hub_record_failure(sensor_snapshot_t * p_snapshot, sensor_failure_kind_t kind,
                                     sensor_failure_device_t device, sensor_failure_stage_t stage,
                                     B channel, fsp_err_t error) {
    if ((NULL == p_snapshot) || (CPU0_SENSOR_FAILURE_NONE != p_snapshot->diagnostics.failure_kind)) {
        return;
    }

    p_snapshot->diagnostics.failure_kind = kind;
    p_snapshot->diagnostics.failure_device = device;
    p_snapshot->diagnostics.failure_stage = stage;
    p_snapshot->diagnostics.failure_channel = channel;
    p_snapshot->last_error = (W) error;
}

/** =================================================================*
 * @brief  I2C転送失敗の診断分類を取得
 * @param[in] error FSPエラーコード
 * @return センサー失敗分類
 * ================================================================= */
LOCAL sensor_failure_kind_t sensor_hub_transport_failure_kind(fsp_err_t error) {
    return (FSP_ERR_TIMEOUT == error) ? CPU0_SENSOR_FAILURE_I2C_TRANSFER_TIMEOUT : CPU0_SENSOR_FAILURE_TRANSPORT;
}

/** =================================================================*
 * @brief  BMI270の4 g生加速度をmgへ変換
 * @param[in] raw BMI270の生値
 * @return acceleration[mg]
 * ================================================================= */
LOCAL H sensor_hub_scale_accel_mg(H raw) {
    return (H) (((W) raw * 1000) / CPU0_BMI270_ACCEL_LSB_PER_G);
}

/** =================================================================*
 * @brief  BMI270の500 dps生角速度を0.1 dpsへ変換
 * @param[in] raw BMI270の生値
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
    sensor_hub_diagnostics_reset(&sensor_hub_diagnostics);
    for (UW index = 0U; index < CPU0_SENSOR_TOF_COUNT; index++) {
        sensor_hub_filter_valid[index] = FALSE;
        sensor_hub_filtered_distance_mm[index] = 0U;
    }

    fsp_err_t err = i2c_bus_init();
    if (FSP_SUCCESS != err) {
        sensor_hub_diagnostics.failure_kind = sensor_hub_transport_failure_kind(err);
        sensor_hub_diagnostics.failure_device = CPU0_SENSOR_FAILURE_DEVICE_I2C_BUS;
        sensor_hub_diagnostics.failure_stage = CPU0_SENSOR_FAILURE_STAGE_I2C_OPEN;
        return err;
    }

    err = tca9548a_disable_all();
    if (FSP_SUCCESS != err) {
        sensor_hub_diagnostics.failure_kind = sensor_hub_transport_failure_kind(err);
        sensor_hub_diagnostics.failure_device = CPU0_SENSOR_FAILURE_DEVICE_TCA9548A;
        sensor_hub_diagnostics.failure_stage = CPU0_SENSOR_FAILURE_STAGE_TCA_DISABLE_ALL;
        sensor_hub_transport_fault_deinit();
        return err;
    }

    for (tof_position_t position = CPU0_TOF_LEFT; position <= CPU0_TOF_RIGHT; position++) {
        UB const channel = sensor_hub_tof_channel(position);
        err = tca9548a_select_channel(channel);
        if (FSP_SUCCESS != err) {
            sensor_hub_diagnostics.failure_kind = sensor_hub_transport_failure_kind(err);
            sensor_hub_diagnostics.failure_device = CPU0_SENSOR_FAILURE_DEVICE_TCA9548A;
            sensor_hub_diagnostics.failure_stage = sensor_hub_tof_select_stage(position);
            sensor_hub_diagnostics.failure_channel = (B) channel;
            sensor_hub_transport_fault_deinit();
            return err;
        }

        vl53l1x_result_t init_result = SENSOR_TOF_RESULT_TRANSPORT_ERROR;
        err = vl53l1x_init(&init_result);
        if (FSP_SUCCESS != err) {
            sensor_hub_diagnostics.failure_kind = (SENSOR_TOF_RESULT_DATA_READY_TIMEOUT == init_result)
                                                      ? CPU0_SENSOR_FAILURE_VL53L1X_DATA_READY_TIMEOUT
                                                      : sensor_hub_transport_failure_kind(err);
            sensor_hub_diagnostics.failure_device = sensor_hub_tof_device(position);
            sensor_hub_diagnostics.failure_stage = sensor_hub_tof_init_stage(position);
            sensor_hub_diagnostics.failure_channel = (B) channel;
            sensor_hub_transport_fault_deinit();
            return err;
        }
    }

    err = tca9548a_select_channel(CPU0_TCA9548A_CHANNEL_BMI270);
    if (FSP_SUCCESS != err) {
        sensor_hub_diagnostics.failure_kind = sensor_hub_transport_failure_kind(err);
        sensor_hub_diagnostics.failure_device = CPU0_SENSOR_FAILURE_DEVICE_TCA9548A;
        sensor_hub_diagnostics.failure_stage = CPU0_SENSOR_FAILURE_STAGE_BMI270_TCA_SELECT;
        sensor_hub_diagnostics.failure_channel = (B) CPU0_TCA9548A_CHANNEL_BMI270;
        sensor_hub_transport_fault_deinit();
        return err;
    }

    err = bmi270_init();
    if (FSP_SUCCESS != err) {
        sensor_hub_diagnostics.failure_kind = sensor_hub_transport_failure_kind(err);
        sensor_hub_diagnostics.failure_device = CPU0_SENSOR_FAILURE_DEVICE_BMI270;
        sensor_hub_diagnostics.failure_stage = CPU0_SENSOR_FAILURE_STAGE_BMI270_INIT;
        sensor_hub_diagnostics.failure_channel = (B) CPU0_TCA9548A_CHANNEL_BMI270;
        sensor_hub_transport_fault_deinit();
        return err;
    }

    sensor_hub_initialized = TRUE;
    return FSP_SUCCESS;
}

/** =================================================================*
 * @brief  正常停止としてセンサーとI2Cを停止
 * ================================================================= */
EXPORT void sensor_hub_deinit(void) {
    if (sensor_hub_initialized) {
        (void) tca9548a_disable_all();
    }
    i2c_bus_deinit();
    sensor_hub_initialized = FALSE;
}

/** =================================================================*
 * @brief  通信障害後にTCA書込みなしでI2Cを停止
 * @details I2C転送失敗後はAbort済みのIICをcloseし、追加のTCA transactionを避ける。
 * ================================================================= */
EXPORT void sensor_hub_transport_fault_deinit(void) {
    i2c_bus_deinit();
    sensor_hub_initialized = FALSE;
}

/** =================================================================*
 * @brief  最終センサー診断を取得
 * @param[out] p_diagnostics 診断取得先
 * ================================================================= */
EXPORT void sensor_hub_diagnostics_get(sensor_diagnostics_t * p_diagnostics) {
    if (NULL != p_diagnostics) {
        *p_diagnostics = sensor_hub_diagnostics;
    }
}

/** =================================================================*
 * @brief  ToF 3台とBMI270を1周期分取得
 * @details 測距値の一時的な無効とVL53L1X data-ready timeoutは個別無効として公開する。
 *          I2C転送障害だけをFSPエラーとして返し、上位のbus recovery対象にする。
 * @param[out] p_snapshot 取得結果
 * @return I2C転送障害なら最初のFSPエラー、それ以外はFSP_SUCCESS
 * ================================================================= */
EXPORT fsp_err_t sensor_hub_poll(sensor_snapshot_t * p_snapshot) {
    if (NULL == p_snapshot) {
        return FSP_ERR_INVALID_ARGUMENT;
    }
    if (!sensor_hub_initialized) {
        return FSP_ERR_NOT_OPEN;
    }

    *p_snapshot = (sensor_snapshot_t){
        .age_ms = 0U,
        .last_error = (W) FSP_SUCCESS,
        .initialized = TRUE,
    };
    sensor_hub_diagnostics_reset(&p_snapshot->diagnostics);
    fsp_err_t first_transport_error = FSP_SUCCESS;

    for (tof_position_t position = CPU0_TOF_LEFT; position <= CPU0_TOF_RIGHT; position++) {
        UW const index = (UW) position;
        UB const channel = sensor_hub_tof_channel(position);
        fsp_err_t const select_err = tca9548a_select_channel(channel);
        if (FSP_SUCCESS != select_err) {
            p_snapshot->error_flags |= CPU0_SENSOR_ERROR_TCA9548A | sensor_hub_tof_error_flag(position);
            sensor_hub_record_failure(p_snapshot, sensor_hub_transport_failure_kind(select_err),
                                      CPU0_SENSOR_FAILURE_DEVICE_TCA9548A, sensor_hub_tof_select_stage(position),
                                      (B) channel, select_err);
            if (FSP_SUCCESS == first_transport_error) {
                first_transport_error = select_err;
            }
            continue;
        }

        vl53l1x_reading_t reading = {0};
        fsp_err_t const read_err = vl53l1x_read_distance(&reading);
        p_snapshot->diagnostics.tof_range_status[index] = reading.range_status;
        p_snapshot->diagnostics.tof_result[index] = (UB) reading.result;
        if (FSP_SUCCESS != read_err) {
            p_snapshot->error_flags |= sensor_hub_tof_error_flag(position);
            if ((SENSOR_TOF_RESULT_RANGE_STATUS_INVALID == reading.result) ||
                (SENSOR_TOF_RESULT_DISTANCE_INVALID == reading.result)) {
                sensor_hub_record_failure(p_snapshot, CPU0_SENSOR_FAILURE_MEASUREMENT_INVALID,
                                          sensor_hub_tof_device(position), sensor_hub_tof_read_stage(position),
                                          (B) channel, read_err);
            } else if (SENSOR_TOF_RESULT_DATA_READY_TIMEOUT == reading.result) {
                sensor_hub_record_failure(p_snapshot, CPU0_SENSOR_FAILURE_VL53L1X_DATA_READY_TIMEOUT,
                                          sensor_hub_tof_device(position), sensor_hub_tof_read_stage(position),
                                          (B) channel, read_err);
            } else {
                sensor_hub_record_failure(p_snapshot, sensor_hub_transport_failure_kind(read_err),
                                          sensor_hub_tof_device(position), sensor_hub_tof_read_stage(position),
                                          (B) channel, read_err);
                if (FSP_SUCCESS == first_transport_error) {
                    first_transport_error = read_err;
                }
            }
            continue;
        }

        if (sensor_hub_filter_valid[index]) {
            sensor_hub_filtered_distance_mm[index] =
                (UH) (((3U * sensor_hub_filtered_distance_mm[index]) + reading.distance_mm) / 4U);
        } else {
            sensor_hub_filtered_distance_mm[index] = reading.distance_mm;
            sensor_hub_filter_valid[index] = TRUE;
        }
        p_snapshot->tof_distance_mm[index] = sensor_hub_filtered_distance_mm[index];
        p_snapshot->valid_flags |= sensor_hub_tof_valid_flag(position);
    }

    bmi270_raw_data_t raw_imu = {0};
    fsp_err_t const bmi_select_err = tca9548a_select_channel(CPU0_TCA9548A_CHANNEL_BMI270);
    fsp_err_t imu_err = bmi_select_err;
    if (FSP_SUCCESS == imu_err) {
        imu_err = bmi270_read_raw(&raw_imu);
    }
    if (FSP_SUCCESS != imu_err) {
        p_snapshot->error_flags |= CPU0_SENSOR_ERROR_BMI270;
        if (FSP_SUCCESS != bmi_select_err) {
            p_snapshot->error_flags |= CPU0_SENSOR_ERROR_TCA9548A;
            sensor_hub_record_failure(p_snapshot, sensor_hub_transport_failure_kind(imu_err),
                                      CPU0_SENSOR_FAILURE_DEVICE_TCA9548A,
                                      CPU0_SENSOR_FAILURE_STAGE_BMI270_TCA_SELECT,
                                      (B) CPU0_TCA9548A_CHANNEL_BMI270, imu_err);
        } else if (FSP_ERR_INVALID_DATA == imu_err) {
            sensor_hub_record_failure(p_snapshot, CPU0_SENSOR_FAILURE_MEASUREMENT_INVALID,
                                      CPU0_SENSOR_FAILURE_DEVICE_BMI270, CPU0_SENSOR_FAILURE_STAGE_BMI270_READ,
                                      (B) CPU0_TCA9548A_CHANNEL_BMI270, imu_err);
        } else {
            sensor_hub_record_failure(p_snapshot, sensor_hub_transport_failure_kind(imu_err),
                                      CPU0_SENSOR_FAILURE_DEVICE_BMI270, CPU0_SENSOR_FAILURE_STAGE_BMI270_READ,
                                      (B) CPU0_TCA9548A_CHANNEL_BMI270, imu_err);
            if (FSP_SUCCESS == first_transport_error) {
                first_transport_error = imu_err;
            }
        }
    } else {
        for (UW axis = 0U; axis < 3U; axis++) {
            p_snapshot->accel_mg[axis] = sensor_hub_scale_accel_mg(raw_imu.accel[axis]);
            p_snapshot->gyro_dps_x10[axis] = sensor_hub_scale_gyro_dps_x10(raw_imu.gyro[axis]);
        }
        p_snapshot->valid_flags |= CPU0_SENSOR_VALID_IMU;
    }

    sensor_hub_diagnostics = p_snapshot->diagnostics;
    return first_transport_error;
}
