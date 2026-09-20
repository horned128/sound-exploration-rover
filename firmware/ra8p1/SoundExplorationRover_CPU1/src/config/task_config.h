/** =================================================================*
 * @file   task_config.h
 * @brief  CPU1 μT-Kernelタスクのスケジュール設定
 * ================================================================= */
#ifndef SEROV_CPU1_CONFIG_TASK_H
#define SEROV_CPU1_CONFIG_TASK_H

#define ACTUATOR_LOOP_PERIOD_MS            (1U)             /**< アクチュエータ周期の周期[ms] */
#define CPU1_SAFETY_TASK_PRIORITY          (3)              /**< 安全タスクの優先度 */
#define CPU1_ACTUATOR_TASK_PRIORITY        (4)              /**< アクチュエータタスクの優先度 */
#define CPU1_STATUS_TASK_PRIORITY          (12)             /**< 状態タスクの優先度 */
#define CPU1_SAFETY_TASK_STACK_SIZE        (1024U)          /**< 安全タスクスタックのサイズ */
#define CPU1_ACTUATOR_TASK_STACK_SIZE      (2048U)          /**< アクチュエータstackサイズ */
#define CPU1_STATUS_TASK_STACK_SIZE        (512U)           /**< 状態タスクスタックのサイズ */
#define CPU1_SAFETY_TASK_PERIOD_MS         (20U)            /**< 安全タスクの周期[ms] */
#define CPU1_STATUS_TASK_PERIOD_MS         (10U)            /**< 状態タスクの周期[ms] */
#define CPU1_STATUS_TELEMETRY_PERIOD_MS    (100U)           /**< 状態テレメトリの周期[ms] */
#define CPU1_STATUS_HEARTBEAT_PERIOD_MS    (500U)           /**< 状態ハートビートの周期[ms] */
#define CPU1_STATUS_FAULT_BLINK_PERIOD_MS  (50U)            /**< 状態異常点滅の周期[ms] */

#endif /* SEROV_CPU1_CONFIG_TASK_H */
