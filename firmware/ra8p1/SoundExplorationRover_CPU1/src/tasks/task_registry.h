/** =================================================================*
 * @file   task_registry.h
 * @brief  CPU1タスク群の初期化API
 * ================================================================= */
#ifndef SEROV_CPU1_TASK_REGISTRY_H
#define SEROV_CPU1_TASK_REGISTRY_H

#include <tk/tkernel.h>                                      /* μT-Kernelの整数型、公開範囲マクロ */

typedef enum e_app_fault {
    APP_FAULT_NONE = 0,
    APP_FAULT_ACTUATOR_INIT,
    APP_FAULT_ACTUATOR_TASK_CREATE,
    APP_FAULT_ACTUATOR_TASK_START,
    APP_FAULT_STATUS_TASK_CREATE,
    APP_FAULT_STATUS_TASK_START,
} app_fault_t;

EXPORT app_fault_t task_registry_init(void);                  /* CPU1タスク群の生成・開始 */

#endif /* SEROV_CPU1_TASK_REGISTRY_H */
