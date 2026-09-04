/** =================================================================*
 * @file   sensor_hub.h
 * @brief  ToF 3台とBMI270をまとめるCPU0センサー層API
 * ================================================================= */
#ifndef SEROV_CPU0_SENSOR_HUB_H
#define SEROV_CPU0_SENSOR_HUB_H

#include "hal_data.h"                                       /* FSPエラー型 */
#include <tk/tkernel.h>                                     /* μT-Kernel型 */

#define CPU0_SENSOR_TOF_COUNT               (3U)

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
} cpu0_sensor_snapshot_t;

EXPORT fsp_err_t sensor_hub_init(void);                     /* I2C、TCA、ToF 3台、BMI270を初期化 */
EXPORT void sensor_hub_deinit(void);                        /* センサーとI2Cを停止 */
EXPORT fsp_err_t sensor_hub_poll(cpu0_sensor_snapshot_t * p_snapshot); /* 1周期の全センサー取得 */

#endif /* SEROV_CPU0_SENSOR_HUB_H */
