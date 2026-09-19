/** =================================================================*
 * @file   task_config.h
 * @brief  CPU1 μT-Kernelタスクのスケジュール設定
 * ================================================================= */
#ifndef SEROV_CPU1_CONFIG_TASK_H
#define SEROV_CPU1_CONFIG_TASK_H

#define ACTUATOR_LOOP_PERIOD_MS            (1U)
#define CPU1_SAFETY_TASK_PRIORITY          (3)
#define CPU1_ACTUATOR_TASK_PRIORITY        (4)
#define CPU1_STATUS_TASK_PRIORITY          (12)
#define CPU1_SAFETY_TASK_STACK_SIZE        (1024U)
#define CPU1_ACTUATOR_TASK_STACK_SIZE      (2048U)
#define CPU1_STATUS_TASK_STACK_SIZE        (512U)
#define CPU1_SAFETY_TASK_PERIOD_MS         (20U)
#define CPU1_STATUS_TASK_PERIOD_MS         (10U)
#define CPU1_STATUS_TELEMETRY_PERIOD_MS    (100U)
#define CPU1_STATUS_HEARTBEAT_PERIOD_MS    (500U)
#define CPU1_STATUS_FAULT_BLINK_PERIOD_MS  (50U)

#endif /* SEROV_CPU1_CONFIG_TASK_H */
