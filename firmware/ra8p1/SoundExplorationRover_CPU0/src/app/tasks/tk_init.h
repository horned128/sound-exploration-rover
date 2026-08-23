/** =================================================================*
 * @file   tk_init.h
 * @brief  CPU0タスク初期化API
 * ================================================================= */
#ifndef SEROV_CPU0_TK_INIT_H
#define SEROV_CPU0_TK_INIT_H

#include "task_common.h"                                  /* CPU0タスク共通異常型 */

cpu0_fault_t cpu0_tasks_init(void);                        /* 初期化タスクの生成・開始 */

#endif /* SEROV_CPU0_TK_INIT_H */
