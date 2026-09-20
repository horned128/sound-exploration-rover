/** =================================================================*
 * @file   task_common.h
 * @brief  CPU0タスク共通状態
 * ================================================================= */
#ifndef SEROV_CPU0_TASK_COMMON_H
#define SEROV_CPU0_TASK_COMMON_H

/**< CPU0タスク・通信初期化時の異常ビット */
typedef enum e_app_fault {
    APP_FAULT_NONE = 0U,                                    /**< 異常なし */
    APP_FAULT_TASK_CREATE = (1U << 0),                      /**< タスク生成失敗 */
    APP_FAULT_TASK_START = (1U << 1),                       /**< タスク開始失敗 */
    APP_FAULT_IPC_INIT = (1U << 2),                         /**< IPC初期化失敗 */
    APP_FAULT_IPC_SEND = (1U << 3),                         /**< IPC送信失敗 */
    APP_FAULT_COMMAND_TARGET_TIMEOUT = (1U << 4),           /**< 指令目標タイムアウト */
    APP_FAULT_TARGET_UPDATE = (1U << 5),                    /**< 目標更新失敗 */
    APP_FAULT_USB_INIT = (1U << 6),                         /**< USB初期化失敗 */
} app_fault_t;

#define APP_FAULT_ALL_MASK                                  /**< CPU0で扱う全異常ビットのマスク */ \
    (APP_FAULT_TASK_CREATE | APP_FAULT_TASK_START | APP_FAULT_IPC_INIT | APP_FAULT_IPC_SEND |                      \
     APP_FAULT_COMMAND_TARGET_TIMEOUT | APP_FAULT_TARGET_UPDATE | APP_FAULT_USB_INIT)

#endif /* SEROV_CPU0_TASK_COMMON_H */
