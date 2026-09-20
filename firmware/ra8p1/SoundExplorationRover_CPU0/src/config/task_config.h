/** =================================================================*
 * @file   task_config.h
 * @brief  CPU0 μT-Kernelタスクのスケジュール設定
 * ================================================================= */
#ifndef SEROV_CPU0_CONFIG_TASK_H
#define SEROV_CPU0_CONFIG_TASK_H

#define CPU0_ACTUATOR_STARTUP_DELAY_MS     (250U)           /**< アクチュエータ起動の遅延[ms] */
#define CPU0_COMMAND_PERIOD_MS             (50U)            /**< 指令の周期[ms] */
#define CPU0_COMMAND_TARGET_TIMEOUT_MS     (500U)           /**< 指令目標の期限[ms] */
#define CPU0_THINK_PERIOD_MS               (50U)            /**< 思考の周期[ms] */
#define CPU0_AUDIO_USB_POLL_MS             (1U)             /**< 音響USBのpoll[ms] */
#define CPU0_AUDIO_USB_RX_SIZE             (512U)           /**< 音響USBRXのサイズ */
#define CPU0_AUDIO_TELEMETRY_PERIOD_MS     (250U)           /**< 音響テレメトリの周期[ms] */
#define CPU0_LED_HEARTBEAT_PERIOD_MS       (1000U)          /**< LEDハートビートの周期[ms] */
#define CPU0_LED_HEARTBEAT_PULSE_MS        (100U)           /**< LEDハートビートのパルス[ms] */
#define CPU0_LED_WAIT_LINK_BLINK_MS        (500U)           /**< LED待ちリンクのblink[ms] */
#define CPU0_LED_LISTEN_BLINK_MS           (1000U)          /**< LED聴取のblink[ms] */
#define CPU0_LED_STEER_BLINK_MS            (125U)           /**< LED操舵のblink[ms] */
#define CPU0_LED_SETTLE_BLINK_MS           (250U)           /**< LED安定待ちのblink[ms] */
#define CPU0_LED_FAULT_PULSE_MS            (100U)           /**< LED異常のパルス[ms] */
#define CPU0_LED_FAULT_GAP_MS              (1000U)          /**< LED異常のgap[ms] */

/* 数値が小さいほど高優先度。IPC keep-aliveを思考処理より優先する。 */
#define CPU0_COMMAND_TASK_PRIORITY         (6)              /**< 指令タスクの優先度 */
#define CPU0_AUDIO_TASK_PRIORITY           (8)              /**< 音響タスクの優先度 */
#define CPU0_SENSOR_TASK_PRIORITY          (9)              /**< センサータスクの優先度 */
#define CPU0_THINK_TASK_PRIORITY           (10)             /**< 思考タスクの優先度 */
#define CPU0_INFER_TASK_PRIORITY           (11)             /**< 推論タスクの優先度 */
#define CPU0_COMMAND_TASK_STACK_SIZE       (2048U)          /**< 指令タスクスタックのサイズ */
#define CPU0_AUDIO_TASK_STACK_SIZE         (4096U)          /**< 音響タスクスタックのサイズ */
#define CPU0_SENSOR_TASK_STACK_SIZE        (2048U)          /**< センサータスクスタックのサイズ */
#define CPU0_THINK_TASK_STACK_SIZE         (4096U)          /**< 思考タスクスタックのサイズ */
#define CPU0_INFER_TASK_STACK_SIZE         (4096U)          /**< 推論タスクスタックのサイズ */

#endif /* SEROV_CPU0_CONFIG_TASK_H */
