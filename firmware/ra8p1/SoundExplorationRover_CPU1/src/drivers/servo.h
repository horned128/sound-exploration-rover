/** =================================================================*
 * @file   servo.h
 * @brief  RCサーボ制御API
 * ================================================================= */
#ifndef SEROV_SERVO_H
#define SEROV_SERVO_H

#include "../cpu1_config.h"                                 /* サーボ数とPWM割り当て */
#include "hal_data.h"                                       /* FSP生成のHAL/BSPインスタンスとFSP型 */
#include <tk/tkernel.h>                                     /* μT-Kernel基本型と公開範囲マクロ */

EXPORT fsp_err_t servo_init(void);                                 /* RCサーボ初期化 */
EXPORT fsp_err_t servo_set_target_deg(UW servo_index, H target_deg); /* サーボ目標角度設定 */
EXPORT fsp_err_t servo_disable(UW servo_index);              /* サーボPWM停止 */
EXPORT H servo_target_deg_get(UW servo_index);         /* サーボ目標角度取得 */

IMPORT volatile UH g_servo_pulse_us[SERVO_COUNT];     /**< 各サーボのパルス幅（Live Watch監視用） */
IMPORT volatile H g_servo_center_trim_us[SERVO_COUNT];/**< 各輪の原点補正（Live Watchで調整可能） */

#endif /* SEROV_SERVO_H */
