/** =================================================================*
 * @file   task_registry.h
 * @brief  CPU1タスク群の初期化API
 * ================================================================= */
#ifndef SEROV_CPU1_TASK_REGISTRY_H
#define SEROV_CPU1_TASK_REGISTRY_H

#include <tk/tkernel.h>                                     /* μT-Kernelの整数型、公開範囲マクロ */

/**< CPU1タスク起動・実行時の異常コード */
typedef enum e_app_fault {
    APP_FAULT_NONE = 0,                                     /**< 異常なし */
    APP_FAULT_ACTUATOR_INIT,                                /**< アクチュエータ初期化失敗 */
    APP_FAULT_ACTUATOR_TASK_CREATE,                         /**< アクチュエータタスク生成失敗 */
    APP_FAULT_ACTUATOR_TASK_START,                          /**< アクチュエータタスク開始失敗 */
    APP_FAULT_STATUS_TASK_CREATE,                           /**< 状態表示タスク生成失敗 */
    APP_FAULT_STATUS_TASK_START,                            /**< 状態表示タスク開始失敗 */
} app_fault_t;

EXPORT app_fault_t task_registry_init(void);                /* CPU1タスク群の生成・開始 */

#endif /* SEROV_CPU1_TASK_REGISTRY_H */
