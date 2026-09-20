/** =================================================================*
 * @file   bmi270.h
 * @brief  BMI270加速度・角速度センサーAPI
 * ================================================================= */
#ifndef SEROV_CPU0_DRIVER_BMI270_H
#define SEROV_CPU0_DRIVER_BMI270_H

#include "hal_data.h"                                       /* FSPエラー型 */
#include <tk/tkernel.h>                                     /* μT-Kernel型 */

/**< BMI270から取得した3軸rawセンサー値 */
typedef struct st_bmi270_raw_data {
    H accel[3];                                             /**< 3軸加速度raw値 */
    H gyro[3];                                              /**< 3軸角速度raw値 */
} bmi270_raw_data_t;

EXPORT fsp_err_t bmi270_init(void);                         /* 初期化、raw accel/gyro出力開始 */
EXPORT fsp_err_t bmi270_read_raw(bmi270_raw_data_t * p_data); /* raw accel/gyro取得 */

#endif /* SEROV_CPU0_DRIVER_BMI270_H */
