/** =================================================================*
 * @file   task_actuator.h
 * @brief  CPU1アクチュエータタスクAPI
 * ================================================================= */
#ifndef SEROV_CPU1_TASK_ACTUATOR_H
#define SEROV_CPU1_TASK_ACTUATOR_H

#include "task_registry.h"                                        /* CPU1異常コード */

EXPORT app_fault_t task_actuator_create(void);        /* アクチュエータ初期化・タスク生成 */
EXPORT app_fault_t task_actuator_start(void);         /* アクチュエータタスク開始 */
EXPORT void task_actuator_delete(void);                /* アクチュエータタスク解放 */

#endif /* SEROV_CPU1_TASK_ACTUATOR_H */
