/** =================================================================*
 * @file   task_status.h
 * @brief  CPU1状態表示タスクAPI
 * ================================================================= */
#ifndef SEROV_CPU1_TASK_STATUS_H
#define SEROV_CPU1_TASK_STATUS_H

#include "task_registry.h"                                  /* CPU1異常コード */

EXPORT app_fault_t task_status_create(void);                /* 状態表示タスク生成 */
EXPORT app_fault_t task_status_start(void);                 /* 状態表示タスク開始 */
EXPORT void task_status_delete(void);                       /* 状態表示タスク解放 */
EXPORT void task_status_halt(app_fault_t fault);            /* 起動異常表示・停止 */

IMPORT volatile ER g_task_status_last_error;                /**< カーネルAPI異常 */
IMPORT volatile UW g_task_status_update_count;              /**< 更新回数 */
IMPORT volatile UW g_task_status_period_last_ms;            /**< 直近周期[ms] */
IMPORT volatile UW g_task_status_period_min_ms;             /**< 最小周期[ms] */
IMPORT volatile UW g_task_status_period_max_ms;             /**< 最大周期[ms] */
IMPORT volatile UW g_task_status_late_count;                /**< 10 ms超の回数 */
IMPORT volatile UD g_task_status_elapsed_total_ms;          /**< 実経過時間[ms] */
IMPORT volatile UW g_task_status_snapshot_count;            /**< 状態採取回数 */
IMPORT volatile UW g_task_status_snapshot_period_last_ms;   /**< 採取間隔[ms] */
IMPORT volatile UW g_task_status_telemetry_count;           /**< 全語送信完了回数 */
IMPORT volatile UW g_task_status_send_retry_count;          /**< 状態語送信失敗回数 */

#endif /* SEROV_CPU1_TASK_STATUS_H */
