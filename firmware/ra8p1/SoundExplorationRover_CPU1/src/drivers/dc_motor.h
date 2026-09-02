/** =================================================================*
 * @file   dc_motor.h
 * @brief  DCモーター制御API
 * ================================================================= */
#ifndef SEROV_DC_MOTOR_H
#define SEROV_DC_MOTOR_H

#include "hal_data.h"                                       /* FSP生成のHAL/BSPインスタンスとFSP型 */
#include <tk/tkernel.h>                                     /* μT-Kernel基本型と公開範囲マクロ */

EXPORT fsp_err_t dc_motor_init(void);                              /* 左右BTS7960初期化 */
EXPORT fsp_err_t dc_motor_request_rpm(H left_rpm, H right_rpm); /* 左右目標回転数設定 */
EXPORT fsp_err_t dc_motor_housekeeping_1ms(void);                  /* ソフトスタート更新 */
EXPORT fsp_err_t dc_motor_stop(void);                              /* 左右モーター即時停止 */
EXPORT H dc_motor_left_target_rpm_get(void);                 /* 左モーター目標回転数取得 */
EXPORT H dc_motor_right_target_rpm_get(void);                /* 右モーター目標回転数取得 */

IMPORT volatile H g_drive_left_duty_permille;         /**< 左モーター出力指令（単位: 1/1000） */
IMPORT volatile H g_drive_right_duty_permille;        /**< 右モーター出力指令（単位: 1/1000） */

#endif /* SEROV_DC_MOTOR_H */
