/** =================================================================*
 * @file   tk_status.h
 * @brief  CPU1状態表示タスクAPI
 * ================================================================= */
#ifndef SEROV_CPU1_TK_STATUS_H
#define SEROV_CPU1_TK_STATUS_H

#include "tk_init.h"                                        /* CPU1異常コード */

EXPORT cpu1_fault_t cpu1_status_task_create(void);          /* 状態表示タスク生成 */
EXPORT cpu1_fault_t cpu1_status_task_start(void);           /* 状態表示タスク開始 */
EXPORT void cpu1_status_task_delete(void);                  /* 状態表示タスク解放 */
EXPORT void cpu1_status_halt(cpu1_fault_t fault);           /* 起動異常表示・停止 */

#endif /* SEROV_CPU1_TK_STATUS_H */
