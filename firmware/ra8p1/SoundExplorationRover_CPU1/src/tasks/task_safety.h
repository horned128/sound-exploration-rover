/** =================================================================*
 * @file   task_safety.h
 * @brief  CPU1独立安全ウォッチドッグタスクAPI（4-7-b）
 * ================================================================= */
#ifndef SEROV_CPU1_TASK_SAFETY_H
#define SEROV_CPU1_TASK_SAFETY_H

#include "task_registry.h"                                        /* CPU1異常コード */

EXPORT app_fault_t task_safety_create(void);                      /* 安全タスク生成 */
EXPORT app_fault_t task_safety_start(void);                       /* 安全タスク開始 */
EXPORT void task_safety_delete(void);                             /* 安全タスク解放 */

IMPORT volatile BOOL g_task_safety_actuator_hang_detected;        /**< 制御タスクハング検知フラグ */
IMPORT volatile BOOL g_task_safety_timeout_detected;              /**< 指令タイムアウト検知フラグ */

#endif /* SEROV_CPU1_TASK_SAFETY_H */
