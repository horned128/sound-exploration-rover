/** =================================================================*
 * @file   tk_actuator.h
 * @brief  CPU1アクチュエータタスクAPI
 * ================================================================= */
#ifndef SEROV_CPU1_TK_ACTUATOR_H
#define SEROV_CPU1_TK_ACTUATOR_H

#include "tk_init.h"                                        /* CPU1異常コード */

EXPORT cpu1_fault_t cpu1_actuator_task_create(void);        /* アクチュエータ初期化・タスク生成 */
EXPORT cpu1_fault_t cpu1_actuator_task_start(void);         /* アクチュエータタスク開始 */
EXPORT void cpu1_actuator_task_delete(void);                /* アクチュエータタスク解放 */

#endif /* SEROV_CPU1_TK_ACTUATOR_H */
