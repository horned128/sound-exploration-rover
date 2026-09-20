/** =================================================================*
 * @file   pin_config.h
 * @brief  CPU1のFSP資源と基板上の役割の対応
 * @details ピン多重化はFSP Solutionの設定を正とする。
 * ================================================================= */
#ifndef SEROV_CPU1_CONFIG_PIN_H
#define SEROV_CPU1_CONFIG_PIN_H

#include "hal_data.h"                                       /* FSP生成のピン・PWM資源定義 */

#define CPU1_STATUS_LED_INDEX              (2U)             /**< 状態LEDの番号 */

/* GPT12B/P803, GPT9B/P110, GPT11B/P801, GPT13B/P808 */
#define SERVO_PWM_INSTANCE_FR              (&g_servo_pwm_fr)/**< サーボPWMインスタンスのFR */
#define SERVO_PWM_INSTANCE_FL              (&g_servo_pwm_fl)/**< サーボPWMインスタンスのFL */
#define SERVO_PWM_INSTANCE_RR              (&g_servo_pwm_rr)/**< サーボPWMインスタンスのRR */
#define SERVO_PWM_INSTANCE_RL              (&g_servo_pwm_rl)/**< サーボPWMインスタンスのRL */
#define SERVO_PWM_OUTPUT_FR                (GPT_IO_PIN_GTIOCB) /**< サーボPWM出力のFR */
#define SERVO_PWM_OUTPUT_FL                (GPT_IO_PIN_GTIOCB) /**< サーボPWM出力のFL */
#define SERVO_PWM_OUTPUT_RR                (GPT_IO_PIN_GTIOCB) /**< サーボPWM出力のRR */
#define SERVO_PWM_OUTPUT_RL                (GPT_IO_PIN_GTIOCB) /**< サーボPWM出力のRL */

#define BTS7960_RPWM_INSTANCE              (&g_motor_pwm)   /**< bts7960RPWMのインスタンス */
#define BTS7960_LPWM_INSTANCE              (&g_motor_pwm_lpwm) /**< bts7960LPWMのインスタンス */
/* P811/GPT10B, P810/GPT10A, P602/GPT7B, P603/GPT7A */
#define BTS7960_LEFT_RPWM_OUTPUT           (GPT_IO_PIN_GTIOCB) /**< bts7960左RPWMの出力 */
#define BTS7960_RIGHT_RPWM_OUTPUT          (GPT_IO_PIN_GTIOCA) /**< bts7960右RPWMの出力 */
#define BTS7960_LEFT_LPWM_OUTPUT           (GPT_IO_PIN_GTIOCB) /**< bts7960左LPWMの出力 */
#define BTS7960_RIGHT_LPWM_OUTPUT          (GPT_IO_PIN_GTIOCA) /**< bts7960右LPWMの出力 */
#define BTS7960_ENABLE_PIN                 (ARDUINO_D8_MIKROBUS_INT) /**< bts7960有効化のピン */

/* P011/IRQ16, P809/IRQ20, P006/IRQ11, P413/IRQ18 */
#define ENCODER_LEFT_A_PIN                 (ARDUINO_D2_INT0)/**< エンコーダ左A相のピン */
#define ENCODER_LEFT_B_PIN                 (ARDUINO_D1TX_MIKROBUS_TX) /**< エンコーダ左B相のピン */
#define ENCODER_RIGHT_A_PIN                (PMOD1_IRQ)      /**< エンコーダ右A相のピン */
#define ENCODER_RIGHT_B_PIN                (PMOD1_GPIO2)    /**< エンコーダ右B相のピン */

#endif /* SEROV_CPU1_CONFIG_PIN_H */
