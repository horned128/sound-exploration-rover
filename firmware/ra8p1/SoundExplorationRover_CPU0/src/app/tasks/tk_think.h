/** =================================================================*
 * @file   tk_think.h
 * @brief  CPU0思考タスクAPI
 * ================================================================= */
#ifndef SEROV_CPU0_TK_THINK_H
#define SEROV_CPU0_TK_THINK_H

#include <stdint.h>                                        /* 固定幅整数型 */
#include <tk/tkernel.h>                                    /* μT-Kernel型 */
#include "task_common.h"                                   /* CPU0タスク共通異常型 */

typedef enum e_cpu0_think_state {
    CPU0_THINK_STATE_CENTERING = 0,
    CPU0_THINK_STATE_STEERING,
    CPU0_THINK_STATE_CIRCLE,
    CPU0_THINK_STATE_FAULT,
} cpu0_think_state_t;

cpu0_fault_t cpu0_think_task_create(void);                 /* 思考タスクとイベント生成 */
cpu0_fault_t cpu0_think_task_start(void);                  /* 思考タスク開始 */
void cpu0_think_task_delete(void);                         /* 思考タスクとイベント解放 */
ER cpu0_think_report_fault(cpu0_fault_t fault);            /* 他タスクからの異常通知 */
void cpu0_think_halt(cpu0_fault_t fault);                  /* 起動不能時のLED表示 */

extern volatile cpu0_think_state_t g_cpu0_think_state;     /**< 現在の思考状態（Live Watch用） */
extern volatile uint32_t g_cpu0_think_cycle_count;         /**< 思考周期実行回数（Live Watch用） */
extern volatile uint32_t g_cpu0_fault_flags;               /**< CPU0異常ラッチ（Live Watch用） */

#endif /* SEROV_CPU0_TK_THINK_H */
