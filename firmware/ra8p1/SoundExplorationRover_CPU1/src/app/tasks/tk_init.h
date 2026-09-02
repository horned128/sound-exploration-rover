/** =================================================================*
 * @file   tk_init.h
 * @brief  CPU1タスク群の初期化API
 * ================================================================= */
#ifndef SEROV_CPU1_TK_INIT_H
#define SEROV_CPU1_TK_INIT_H

#include <tk/tkernel.h>                                      /* μT-Kernelの整数型、公開範囲マクロ */

typedef enum e_cpu1_fault {
    CPU1_FAULT_NONE = 0,
    CPU1_FAULT_ACTUATOR_INIT,
    CPU1_FAULT_ACTUATOR_TASK_CREATE,
    CPU1_FAULT_ACTUATOR_TASK_START,
    CPU1_FAULT_STATUS_TASK_CREATE,
    CPU1_FAULT_STATUS_TASK_START,
} cpu1_fault_t;

EXPORT cpu1_fault_t cpu1_tasks_init(void);                  /* CPU1タスク群の生成・開始 */

#endif /* SEROV_CPU1_TK_INIT_H */
