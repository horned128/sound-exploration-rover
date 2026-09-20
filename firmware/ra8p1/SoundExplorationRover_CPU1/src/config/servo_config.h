/** =================================================================*
 * @file   servo_config.h
 * @brief  操舵サーボの可動範囲と校正設定
 * ================================================================= */
#ifndef SEROV_CPU1_CONFIG_SERVO_H
#define SEROV_CPU1_CONFIG_SERVO_H

#include "../../../common/ipc_message.h"                    /* CPU間で共有するサーボ数定義 */

#define SERVO_PWM_PERIOD_US                (20000U)         /**< サーボPWMの周期[us] */
#define SERVO_PULSE_MIN_SAFE_US            (1000U)          /**< サーボパルス最小のsafe[us] */
#define SERVO_PULSE_CENTER_US              (1500U)          /**< サーボパルスの中央[us] */
#define SERVO_PULSE_MAX_SAFE_US            (2000U)          /**< サーボパルス最大のsafe[us] */
#define STEERING_MIN_DEG                   (-45)            /**< 操舵の最小[deg] */
#define STEERING_CENTER_DEG                (0)              /**< 操舵の中央[deg] */
#define STEERING_MAX_DEG                   (45)             /**< 操舵の最大[deg] */
#define STEERING_MIN_PULSE_US              (1200U)          /**< 操舵最小のパルス[us] */
#define STEERING_MAX_PULSE_US              (1800U)          /**< 操舵最大のパルス[us] */
#define SERVO_COUNT                        ACTUATOR_SERVO_COUNT /**< サーボの個数 */

#define SERVO_CENTER_TRIM_US_FR            (0)              /**< サーボ中央補正のFR[us] */
#define SERVO_CENTER_TRIM_US_FL            (0)              /**< サーボ中央補正のFL[us] */
#define SERVO_CENTER_TRIM_US_RR            (0)              /**< サーボ中央補正のRR[us] */
#define SERVO_CENTER_TRIM_US_RL            (0)              /**< サーボ中央補正のRL[us] */
#define SERVO_DIRECTION_FR                 (1)              /**< サーボ方向のFR */
#define SERVO_DIRECTION_FL                 (1)              /**< サーボ方向のFL */
#define SERVO_DIRECTION_RR                 (1)              /**< サーボ方向のRR */
#define SERVO_DIRECTION_RL                 (1)              /**< サーボ方向のRL */

#endif /* SEROV_CPU1_CONFIG_SERVO_H */
