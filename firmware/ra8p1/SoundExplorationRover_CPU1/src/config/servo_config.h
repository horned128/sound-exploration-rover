/** =================================================================*
 * @file   servo_config.h
 * @brief  操舵サーボの可動範囲と校正設定
 * ================================================================= */
#ifndef SEROV_CPU1_CONFIG_SERVO_H
#define SEROV_CPU1_CONFIG_SERVO_H

#include "../../../common/ipc_message.h"                    /* CPU間で共有するサーボ数定義 */

#define SERVO_PWM_PERIOD_US                (20000U)
#define SERVO_PULSE_MIN_SAFE_US            (1000U)
#define SERVO_PULSE_CENTER_US              (1500U)
#define SERVO_PULSE_MAX_SAFE_US            (2000U)
#define STEERING_MIN_DEG                   (-45)
#define STEERING_CENTER_DEG                (0)
#define STEERING_MAX_DEG                   (45)
#define STEERING_MIN_PULSE_US              (1200U)
#define STEERING_MAX_PULSE_US              (1800U)
#define SERVO_COUNT                        ACTUATOR_SERVO_COUNT

#define SERVO_CENTER_TRIM_US_FR            (0)
#define SERVO_CENTER_TRIM_US_FL            (0)
#define SERVO_CENTER_TRIM_US_RR            (0)
#define SERVO_CENTER_TRIM_US_RL            (0)
#define SERVO_DIRECTION_FR                 (1)
#define SERVO_DIRECTION_FL                 (1)
#define SERVO_DIRECTION_RR                 (1)
#define SERVO_DIRECTION_RL                 (1)

#endif /* SEROV_CPU1_CONFIG_SERVO_H */
