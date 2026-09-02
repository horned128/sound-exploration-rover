/** =================================================================*
 * @file   cpu1_config.h
 * @brief  CPU1アクチュエータ設定
 * ================================================================= */
#ifndef SEROV_CPU1_CONFIG_H
#define SEROV_CPU1_CONFIG_H

#include "../../common/ipc_message.h"                       /* CPU間通信で共有するサーボ数 */
#include "hal_data.h"                                       /* FSP生成のHAL/BSPインスタンス、ピン定義、周辺機器設定 */

/* 安全設定と周期設定。CPU0指令タスクは現在50 ms周期で指令を送信する。 */
#define ACTUATOR_COMMAND_TIMEOUT_MS        (1500U)
#define ACTUATOR_LOOP_PERIOD_MS            (1U)

/* 数値が小さいほど高優先度。1 msアクチュエータ処理を状態表示より優先する。 */
#define CPU1_ACTUATOR_TASK_PRIORITY        (4)
#define CPU1_STATUS_TASK_PRIORITY          (12)
#define CPU1_ACTUATOR_TASK_STACK_SIZE      (2048U)
#define CPU1_STATUS_TASK_STACK_SIZE        (512U)
#define CPU1_STATUS_TASK_PERIOD_MS         (10U)
#define CPU1_STATUS_LED_INDEX              (2U)
#define CPU1_STATUS_HEARTBEAT_PERIOD_MS    (500U)
#define CPU1_STATUS_FAULT_BLINK_PERIOD_MS  (50U)

/* 操舵サーボ設定。DS3225MGの180度範囲から安全側の範囲を使用する。 */
#define SERVO_PWM_PERIOD_US                (20000U)
#define SERVO_PULSE_MIN_SAFE_US            (1000U)
#define SERVO_PULSE_CENTER_US              (1500U)
#define SERVO_PULSE_MAX_SAFE_US            (2000U)
#define STEERING_MIN_DEG                   (-45)
#define STEERING_CENTER_DEG                (0)
#define STEERING_MAX_DEG                   (45)
#define STEERING_MIN_PULSE_US              (1200U)
#define STEERING_MAX_PULSE_US              (1800U)

/* 4サーボPWM割り当て。配列順はFR、FL、RR、RL。 */
#define SERVO_COUNT ACTUATOR_SERVO_COUNT
/* FSPインスタンス名を論理輪名へ割り当てる。 */
/* GPT12B / P803 */
#define SERVO_PWM_INSTANCE_FR (&g_servo_pwm_fr)
/* GPT9B / P110 */
#define SERVO_PWM_INSTANCE_FL (&g_servo_pwm_fl)
/* GPT11B / P801 */
#define SERVO_PWM_INSTANCE_RR (&g_servo_pwm_rr)
/* GPT13B / P808 */
#define SERVO_PWM_INSTANCE_RL              (&g_servo_pwm_rl)
#define SERVO_PWM_OUTPUT_FR                (GPT_IO_PIN_GTIOCB)
#define SERVO_PWM_OUTPUT_FL                (GPT_IO_PIN_GTIOCB)
#define SERVO_PWM_OUTPUT_RR                (GPT_IO_PIN_GTIOCB)
#define SERVO_PWM_OUTPUT_RL                (GPT_IO_PIN_GTIOCB)

/* 各輪の操舵補正。1500 usで直進に合わせ、必要なら5～10 usずつ調整する。 */
#define SERVO_CENTER_TRIM_US_FR            (0)
#define SERVO_CENTER_TRIM_US_FL            (0)
#define SERVO_CENTER_TRIM_US_RR            (0)
#define SERVO_CENTER_TRIM_US_RL            (0)
#define SERVO_DIRECTION_FR                 (1)
#define SERVO_DIRECTION_FL                 (1)
#define SERVO_DIRECTION_RR                 (1)
#define SERVO_DIRECTION_RL                 (1)

/* 左右BTS7960の20 kHz PWMおよび代表エンコーダ速度補正設定。 */
#define JGA25_TARGET_RPM_MAX               (300)
#define JGA25_ENCODER_COUNTS_PER_REV       (900U)
#define JGA25_SPEED_SAMPLE_PERIOD_MS       (100U)
#define JGA25_ENCODER_LEFT_FORWARD_SIGN    (-1)
#define JGA25_ENCODER_RIGHT_FORWARD_SIGN   (+1)

#define MOTOR_RPWM_INSTANCE                (&g_motor_pwm)
#define MOTOR_LPWM_INSTANCE                (&g_motor_pwm_lpwm)
/* P811 / GPT10B */
#define MOTOR_LEFT_RPWM_OUTPUT (GPT_IO_PIN_GTIOCB)
/* P810 / GPT10A */
#define MOTOR_RIGHT_RPWM_OUTPUT (GPT_IO_PIN_GTIOCA)
/* P602 / GPT7B */
#define MOTOR_LEFT_LPWM_OUTPUT (GPT_IO_PIN_GTIOCB)
/* P603 / GPT7A */
#define MOTOR_RIGHT_LPWM_OUTPUT (GPT_IO_PIN_GTIOCA)
/*
 * 正のRPMはローバー前進を表す。現在の実機配線では左右ともRPWMが前進である。
 * モーター配線や取付けを変更した場合は、左右個別の符号で吸収する。
 */
#define MOTOR_CHASSIS_FORWARD_SIGN         (+1)
#define MOTOR_LEFT_MOUNT_SIGN              (+1)
#define MOTOR_RIGHT_MOUNT_SIGN             (+1)
#define MOTOR_LEFT_FORWARD_SIGN            (MOTOR_CHASSIS_FORWARD_SIGN * MOTOR_LEFT_MOUNT_SIGN)
#define MOTOR_RIGHT_FORWARD_SIGN           (MOTOR_CHASSIS_FORWARD_SIGN * MOTOR_RIGHT_MOUNT_SIGN)
/* 左右BTS7960のR_EN/L_ENを外部で共通接続し、PD01で一括制御する。 */
#define MOTOR_ENABLE_PIN                   (ARDUINO_D8_MIKROBUS_INT)
#define MOTOR_PWM_MIN_DUTY_PERMILLE        (0)
#define MOTOR_PWM_MAX_DUTY_PERMILLE        (700)
#define MOTOR_PWM_RAMP_PER_MS              (2)
#define MOTOR_PWM_UPDATE_PERIOD_MS         (5U)
/* 右側の無負荷速度差を補正する初期デューティ比。速度フィードバックで追従する。 */
#define MOTOR_LEFT_DUTY_SCALE_PERMILLE               (1000U)
#define MOTOR_RIGHT_DUTY_SCALE_PERMILLE              (750U)
#define MOTOR_SPEED_FEEDBACK_ENABLE                  (1U)
#define MOTOR_SPEED_FEEDBACK_START_DELAY_MS          (150U)
#define MOTOR_SPEED_FEEDBACK_KP_PERMILLE_PER_RPM     (2)
#define MOTOR_SPEED_FEEDBACK_MAX_CORRECTION_PERMILLE (250)

/*
 * エンコーダ入力は左右の代表モーターを1組ずつ測定する。
 * 現在の6輪構成は左右各3台を1台のBTS7960へ並列接続するため、6台を
 * 個別制御できない現状で6組を混ぜて1入力へ接続してはいけない。
 * A/Bは両エッジの四逓倍クアドラチャ計数に使用する。
 */
/* P011 / IRQ16 */
#define JGA25_ENCODER_LEFT_A_PIN (ARDUINO_D2_INT0)
/* P809 / IRQ20 */
#define JGA25_ENCODER_LEFT_B_PIN (ARDUINO_D1TX_MIKROBUS_TX)
/* P006 / IRQ11 */
#define JGA25_ENCODER_RIGHT_A_PIN (PMOD1_IRQ)
/* P413 / IRQ18 */
#define JGA25_ENCODER_RIGHT_B_PIN (PMOD1_GPIO2)

#endif /* SEROV_CPU1_CONFIG_H */
