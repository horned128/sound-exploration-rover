/** =================================================================*
 * @file   task_config.h
 * @brief  CPU0 μT-Kernelタスクのスケジュール設定
 * ================================================================= */
#ifndef SEROV_CPU0_CONFIG_TASK_H
#define SEROV_CPU0_CONFIG_TASK_H

#define CPU0_ACTUATOR_STARTUP_DELAY_MS     (250U)
#define CPU0_COMMAND_PERIOD_MS             (50U)
#define CPU0_COMMAND_TARGET_TIMEOUT_MS     (500U)
#define CPU0_THINK_PERIOD_MS               (100U)
#define CPU0_AUDIO_USB_POLL_MS             (1U)
#define CPU0_AUDIO_USB_RX_SIZE             (512U)
#define CPU0_AUDIO_TELEMETRY_PERIOD_MS     (250U)
#define CPU0_LED_HEARTBEAT_PERIOD_MS       (1000U)
#define CPU0_LED_HEARTBEAT_PULSE_MS        (100U)
#define CPU0_LED_WAIT_LINK_BLINK_MS        (500U)
#define CPU0_LED_LISTEN_BLINK_MS           (1000U)
#define CPU0_LED_STEER_BLINK_MS            (125U)
#define CPU0_LED_SETTLE_BLINK_MS           (250U)
#define CPU0_LED_FAULT_PULSE_MS            (100U)
#define CPU0_LED_FAULT_GAP_MS              (1000U)

/* 数値が小さいほど高優先度。IPC keep-aliveを思考処理より優先する。 */
#define CPU0_COMMAND_TASK_PRIORITY         (6)
#define CPU0_AUDIO_TASK_PRIORITY           (8)
#define CPU0_SENSOR_TASK_PRIORITY          (9)
#define CPU0_THINK_TASK_PRIORITY           (10)
#define CPU0_INFER_TASK_PRIORITY           (11)
#define CPU0_COMMAND_TASK_STACK_SIZE       (1024U)
#define CPU0_AUDIO_TASK_STACK_SIZE         (2048U)
#define CPU0_SENSOR_TASK_STACK_SIZE        (2048U)
#define CPU0_THINK_TASK_STACK_SIZE         (1024U)
#define CPU0_INFER_TASK_STACK_SIZE         (2048U)

#endif /* SEROV_CPU0_CONFIG_TASK_H */
