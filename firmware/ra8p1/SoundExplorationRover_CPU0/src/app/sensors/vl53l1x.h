/** =================================================================*
 * @file   vl53l1x.h
 * @brief  VL53L1X ToF距離センサーAPI
 * ================================================================= */
#ifndef SEROV_CPU0_VL53L1X_H
#define SEROV_CPU0_VL53L1X_H

#include "hal_data.h"                                       /* FSPエラー型 */
#include <tk/tkernel.h>                                     /* μT-Kernel型 */

EXPORT fsp_err_t vl53l1x_init(void);                        /* 初期化して連続測距を開始 */
EXPORT fsp_err_t vl53l1x_read_distance(UH * p_distance_mm); /* 最新の有効距離を取得 */

#endif /* SEROV_CPU0_VL53L1X_H */
