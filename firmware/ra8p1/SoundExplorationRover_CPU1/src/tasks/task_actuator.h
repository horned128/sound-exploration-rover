/** =================================================================*
 * @file   task_actuator.h
 * @brief  CPU1アクチュエータタスクAPI
 * ================================================================= */
#ifndef SEROV_CPU1_TASK_ACTUATOR_H
#define SEROV_CPU1_TASK_ACTUATOR_H

#include "task_registry.h"                                  /* CPU1異常コード */

EXPORT app_fault_t task_actuator_create(void);              /* アクチュエータ初期化・タスク生成 */
EXPORT app_fault_t task_actuator_start(void);               /* アクチュエータタスク開始 */
EXPORT void task_actuator_delete(void);                     /* アクチュエータタスク解放 */

IMPORT volatile ER g_task_actuator_last_error;              /**< カーネルAPIの最終異常 */
IMPORT volatile UW g_task_actuator_update_count;            /**< 実時間更新の実行回数 */
IMPORT volatile UW g_task_actuator_period_last_ms;          /**< 直近の実更新間隔[ms] */
IMPORT volatile UW g_task_actuator_period_min_ms;           /**< 最小実更新間隔[ms] */
IMPORT volatile UW g_task_actuator_period_max_ms;           /**< 最大実更新間隔[ms] */
IMPORT volatile UW g_task_actuator_late_count;              /**< 1 ms超の更新間隔の回数 */
IMPORT volatile UD g_task_actuator_elapsed_total_ms;        /**< 開始からの実経過時間[ms] */

#endif /* SEROV_CPU1_TASK_ACTUATOR_H */
