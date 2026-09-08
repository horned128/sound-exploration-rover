/** =================================================================*
 * @file   task_status.h
 * @brief  CPU1状態表示タスクAPI
 * ================================================================= */
#ifndef SEROV_CPU1_TASK_STATUS_H
#define SEROV_CPU1_TASK_STATUS_H

#include "task_registry.h"                                        /* CPU1異常コード */

EXPORT app_fault_t task_status_create(void);          /* 状態表示タスク生成 */
EXPORT app_fault_t task_status_start(void);           /* 状態表示タスク開始 */
EXPORT void task_status_delete(void);                  /* 状態表示タスク解放 */
EXPORT void task_status_halt(app_fault_t fault);           /* 起動異常表示・停止 */

#endif /* SEROV_CPU1_TASK_STATUS_H */
