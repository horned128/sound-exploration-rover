/** =================================================================*
 * @file   vl53l1x.h
 * @brief  VL53L1X ToF距離センサーAPI
 * ================================================================= */
#ifndef SEROV_CPU0_DRIVER_VL53L1X_H
#define SEROV_CPU0_DRIVER_VL53L1X_H

#include "hal_data.h"                                       /* FSPエラー型 */
#include <tk/tkernel.h>                                     /* μT-Kernel型 */

/**< VL53L1X測距処理の結果 */
typedef enum e_vl53l1x_result {
    VL53L1X_RESULT_VALID = 0U,                              /**< 測距値が有効 */
    VL53L1X_RESULT_RANGE_STATUS_INVALID,                    /**< レンジステータスが異常 */
    VL53L1X_RESULT_DISTANCE_INVALID,                        /**< 距離値が異常 */
    VL53L1X_RESULT_DATA_READY_TIMEOUT,                      /**< データ準備待ちタイムアウト */
    VL53L1X_RESULT_TRANSPORT_ERROR,                         /**< 通信転送エラー */
} vl53l1x_result_t;

/**< VL53L1Xの最新測距値と取得診断結果 */
typedef struct st_vl53l1x_reading {
    UH distance_mm;                                         /**< 測距距離[mm] */
    UB range_status;                                        /**< VL53L1Xレンジステータス */
    vl53l1x_result_t result;                                /**< 測距結果の判定状態 */
} vl53l1x_reading_t;

#define VL53L1X_RANGE_STATUS_UNAVAILABLE   (0xFFU)          /**< VL53L1Xレンジ状態の未取得 */

EXPORT fsp_err_t vl53l1x_init(vl53l1x_result_t * p_result); /* 初期化して連続測距を開始 */
EXPORT fsp_err_t vl53l1x_read_distance(vl53l1x_reading_t * p_reading); /* 最新測距と診断結果を取得 */

#endif /* SEROV_CPU0_DRIVER_VL53L1X_H */
