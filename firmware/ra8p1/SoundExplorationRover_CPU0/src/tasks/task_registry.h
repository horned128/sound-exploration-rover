/** =================================================================*
 * @file   task_registry.h
 * @brief  CPU0タスク初期化API
 * ================================================================= */
#ifndef SEROV_CPU0_TASK_REGISTRY_H
#define SEROV_CPU0_TASK_REGISTRY_H

#include "task_common.h"                                    /* CPU0タスク共通異常型 */
#include <tk/tkernel.h>                                     /* μT-Kernel基本型と公開範囲マクロ */

EXPORT app_fault_t task_registry_init(void);                /* 登録済みタスク群の生成・開始 */

#endif /* SEROV_CPU0_TASK_REGISTRY_H */
