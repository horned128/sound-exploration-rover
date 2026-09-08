/** =================================================================*
 * @file   vl53l1x.h
 * @brief  VL53L1X ToF距離センサーAPI
 * ================================================================= */
#ifndef SEROV_CPU0_VL53L1X_H
#define SEROV_CPU0_VL53L1X_H

#include "hal_data.h"                                       /* FSPエラー型 */
#include <tk/tkernel.h>                                     /* μT-Kernel型 */

typedef enum e_vl53l1x_result {
    VL53L1X_RESULT_VALID = 0U,
    VL53L1X_RESULT_RANGE_STATUS_INVALID,
    VL53L1X_RESULT_DISTANCE_INVALID,
    VL53L1X_RESULT_DATA_READY_TIMEOUT,
    VL53L1X_RESULT_TRANSPORT_ERROR,
} vl53l1x_result_t;

typedef struct st_vl53l1x_reading {
    UH distance_mm;
    UB range_status;
    vl53l1x_result_t result;
} vl53l1x_reading_t;

#define VL53L1X_RANGE_STATUS_UNAVAILABLE    (0xFFU)

EXPORT fsp_err_t vl53l1x_init(vl53l1x_result_t * p_result); /* 初期化して連続測距を開始 */
EXPORT fsp_err_t vl53l1x_read_distance(vl53l1x_reading_t * p_reading); /* 最新測距と診断結果を取得 */

#endif /* SEROV_CPU0_VL53L1X_H */
