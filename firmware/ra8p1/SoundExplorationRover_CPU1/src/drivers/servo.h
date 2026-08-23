/** =================================================================*
 * @file   servo.h
 * @brief  RCサーボ制御API
 * ================================================================= */
#ifndef SEROV_SERVO_H
#define SEROV_SERVO_H

#include "hal_data.h"                                       /* FSP生成のHAL/BSPインスタンスとFSP型 */
#include "../cpu1_config.h"                                 /* サーボ数とPWM割り当て */

fsp_err_t servo_init(void);                                 /* RCサーボ初期化 */
/* サーボ目標角度設定 */
fsp_err_t servo_set_target_deg(uint32_t servo_index, int16_t target_deg);
fsp_err_t servo_disable(uint32_t servo_index);              /* サーボPWM停止 */
/* サーボ目標角度取得 */
int16_t servo_target_deg_get(uint32_t servo_index);

extern volatile uint16_t g_servo_pulse_us[SERVO_COUNT];     /**< 各サーボのパルス幅（Live Watch監視用） */
extern volatile int16_t g_servo_center_trim_us[SERVO_COUNT];/**< 各輪の原点補正（Live Watchで調整可能） */

#endif /* SEROV_SERVO_H */
