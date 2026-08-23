/** =================================================================*
 * @file   dc_motor.h
 * @brief  DCモーター制御API
 * ================================================================= */
#ifndef SEROV_DC_MOTOR_H
#define SEROV_DC_MOTOR_H

#include "hal_data.h"                                       /* FSP生成のHAL/BSPインスタンスとFSP型 */

fsp_err_t dc_motor_init(void);                              /* 左右BTS7960初期化 */
/* 左右目標回転数設定 */
fsp_err_t dc_motor_request_rpm(int16_t left_rpm, int16_t right_rpm);
fsp_err_t dc_motor_housekeeping_1ms(void);                  /* ソフトスタート更新 */
fsp_err_t dc_motor_stop(void);                              /* 左右モーター即時停止 */
/* 左モーター目標回転数取得 */
int16_t dc_motor_left_target_rpm_get(void);
/* 右モーター目標回転数取得 */
int16_t dc_motor_right_target_rpm_get(void);

extern volatile int16_t g_drive_left_duty_permille;         /**< 左モーター出力指令（単位: 1/1000） */
extern volatile int16_t g_drive_right_duty_permille;        /**< 右モーター出力指令（単位: 1/1000） */

#endif /* SEROV_DC_MOTOR_H */
