/** =================================================================*
 * @file   sensor_hub.h
 * @brief  ToF 3台とBMI270をまとめるCPU0センサー層API
 * ================================================================= */
#ifndef SEROV_CPU0_SENSOR_HUB_H
#define SEROV_CPU0_SENSOR_HUB_H

#include "hal_data.h"                                       /* FSPエラー型 */
#include <tk/tkernel.h>                                     /* μT-Kernel型 */

#define CPU0_SENSOR_TOF_COUNT               (3U)
#define CPU0_SENSOR_FAILURE_CHANNEL_NONE    (-1)

typedef enum e_cpu0_tof_position {
    CPU0_TOF_LEFT = 0U,
    CPU0_TOF_CENTER,
    CPU0_TOF_RIGHT,
} cpu0_tof_position_t;

#define CPU0_SENSOR_VALID_TOF_LEFT           (1U << 0)
#define CPU0_SENSOR_VALID_TOF_CENTER         (1U << 1)
#define CPU0_SENSOR_VALID_TOF_RIGHT          (1U << 2)
#define CPU0_SENSOR_VALID_IMU                (1U << 3)
#define CPU0_SENSOR_VALID_ALL                (CPU0_SENSOR_VALID_TOF_LEFT | CPU0_SENSOR_VALID_TOF_CENTER | \
                                              CPU0_SENSOR_VALID_TOF_RIGHT | CPU0_SENSOR_VALID_IMU)

#define CPU0_SENSOR_ERROR_I2C_INIT           (1U << 0)
#define CPU0_SENSOR_ERROR_TCA9548A           (1U << 1)
#define CPU0_SENSOR_ERROR_TOF_LEFT           (1U << 2)
#define CPU0_SENSOR_ERROR_TOF_CENTER         (1U << 3)
#define CPU0_SENSOR_ERROR_TOF_RIGHT          (1U << 4)
#define CPU0_SENSOR_ERROR_BMI270             (1U << 5)
#define CPU0_SENSOR_ERROR_STALE              (1U << 6)

typedef enum e_cpu0_sensor_failure_kind {
    CPU0_SENSOR_FAILURE_NONE = 0U,
    CPU0_SENSOR_FAILURE_MEASUREMENT_INVALID,
    CPU0_SENSOR_FAILURE_I2C_TRANSFER_TIMEOUT,
    CPU0_SENSOR_FAILURE_VL53L1X_DATA_READY_TIMEOUT,
    CPU0_SENSOR_FAILURE_TRANSPORT,
} cpu0_sensor_failure_kind_t;

typedef enum e_cpu0_sensor_failure_device {
    CPU0_SENSOR_FAILURE_DEVICE_NONE = 0U,
    CPU0_SENSOR_FAILURE_DEVICE_I2C_BUS,
    CPU0_SENSOR_FAILURE_DEVICE_TCA9548A,
    CPU0_SENSOR_FAILURE_DEVICE_TOF_LEFT,
    CPU0_SENSOR_FAILURE_DEVICE_TOF_CENTER,
    CPU0_SENSOR_FAILURE_DEVICE_TOF_RIGHT,
    CPU0_SENSOR_FAILURE_DEVICE_BMI270,
} cpu0_sensor_failure_device_t;

typedef enum e_cpu0_sensor_failure_stage {
    CPU0_SENSOR_FAILURE_STAGE_NONE = 0U,
    CPU0_SENSOR_FAILURE_STAGE_I2C_OPEN,
    CPU0_SENSOR_FAILURE_STAGE_TCA_DISABLE_ALL,
    CPU0_SENSOR_FAILURE_STAGE_TOF_LEFT_TCA_SELECT,
    CPU0_SENSOR_FAILURE_STAGE_TOF_LEFT_INIT,
    CPU0_SENSOR_FAILURE_STAGE_TOF_CENTER_TCA_SELECT,
    CPU0_SENSOR_FAILURE_STAGE_TOF_CENTER_INIT,
    CPU0_SENSOR_FAILURE_STAGE_TOF_RIGHT_TCA_SELECT,
    CPU0_SENSOR_FAILURE_STAGE_TOF_RIGHT_INIT,
    CPU0_SENSOR_FAILURE_STAGE_BMI270_TCA_SELECT,
    CPU0_SENSOR_FAILURE_STAGE_BMI270_INIT,
    CPU0_SENSOR_FAILURE_STAGE_TOF_LEFT_READ,
    CPU0_SENSOR_FAILURE_STAGE_TOF_CENTER_READ,
    CPU0_SENSOR_FAILURE_STAGE_TOF_RIGHT_READ,
    CPU0_SENSOR_FAILURE_STAGE_BMI270_READ,
} cpu0_sensor_failure_stage_t;

typedef struct st_cpu0_sensor_diagnostics {
    UB tof_range_status[CPU0_SENSOR_TOF_COUNT];
    UB tof_result[CPU0_SENSOR_TOF_COUNT];
    cpu0_sensor_failure_kind_t failure_kind;
    cpu0_sensor_failure_device_t failure_device;
    cpu0_sensor_failure_stage_t failure_stage;
    B failure_channel;
} cpu0_sensor_diagnostics_t;

typedef struct st_cpu0_sensor_snapshot {
    UH tof_distance_mm[CPU0_SENSOR_TOF_COUNT];
    H accel_mg[3];
    H gyro_dps_x10[3];
    UW age_ms;
    UW update_count;
    UW error_flags;
    W last_error;
    UB valid_flags;
    BOOL initialized;
    cpu0_sensor_diagnostics_t diagnostics;
} cpu0_sensor_snapshot_t;

EXPORT fsp_err_t sensor_hub_init(void);                     /* I2C、TCA、ToF 3台、BMI270を初期化 */
EXPORT void sensor_hub_deinit(void);                        /* センサーとI2Cを停止 */
EXPORT void sensor_hub_transport_fault_deinit(void);        /* 通信障害後にTCA書込みなしでI2Cを停止 */
EXPORT void sensor_hub_diagnostics_get(cpu0_sensor_diagnostics_t * p_diagnostics); /* 最終診断を取得 */
EXPORT fsp_err_t sensor_hub_poll(cpu0_sensor_snapshot_t * p_snapshot); /* 1周期の全センサー状態を更新 */

#endif /* SEROV_CPU0_SENSOR_HUB_H */
