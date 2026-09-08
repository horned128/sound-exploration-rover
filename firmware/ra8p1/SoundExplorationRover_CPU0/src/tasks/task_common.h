/** =================================================================*
 * @file   task_common.h
 * @brief  CPU0タスク共通状態
 * ================================================================= */
#ifndef SEROV_CPU0_TASK_COMMON_H
#define SEROV_CPU0_TASK_COMMON_H

typedef enum e_app_fault {
    APP_FAULT_NONE = 0U,
    APP_FAULT_TASK_CREATE = (1U << 0),
    APP_FAULT_TASK_START = (1U << 1),
    APP_FAULT_IPC_INIT = (1U << 2),
    APP_FAULT_IPC_SEND = (1U << 3),
    APP_FAULT_COMMAND_TARGET_TIMEOUT = (1U << 4),
    APP_FAULT_TARGET_UPDATE = (1U << 5),
    APP_FAULT_USB_INIT = (1U << 6),
} app_fault_t;

#define APP_FAULT_ALL_MASK                                                                                            \
    (APP_FAULT_TASK_CREATE | APP_FAULT_TASK_START | APP_FAULT_IPC_INIT | APP_FAULT_IPC_SEND |                      \
     APP_FAULT_COMMAND_TARGET_TIMEOUT | APP_FAULT_TARGET_UPDATE | APP_FAULT_USB_INIT)

#endif /* SEROV_CPU0_TASK_COMMON_H */
