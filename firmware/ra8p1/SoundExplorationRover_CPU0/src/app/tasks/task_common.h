/** =================================================================*
 * @file   task_common.h
 * @brief  CPU0タスク共通状態
 * ================================================================= */
#ifndef SEROV_CPU0_TASK_COMMON_H
#define SEROV_CPU0_TASK_COMMON_H

typedef enum e_cpu0_fault {
    CPU0_FAULT_NONE                   = 0U,
    CPU0_FAULT_TASK_CREATE            = (1U << 0),
    CPU0_FAULT_TASK_START             = (1U << 1),
    CPU0_FAULT_IPC_INIT               = (1U << 2),
    CPU0_FAULT_IPC_SEND               = (1U << 3),
    CPU0_FAULT_COMMAND_TARGET_TIMEOUT = (1U << 4),
    CPU0_FAULT_TARGET_UPDATE          = (1U << 5),
    CPU0_FAULT_USB_INIT               = (1U << 6),
} cpu0_fault_t;

#define CPU0_FAULT_ALL_MASK (CPU0_FAULT_TASK_CREATE | \
                             CPU0_FAULT_TASK_START | \
                             CPU0_FAULT_IPC_INIT | \
                             CPU0_FAULT_IPC_SEND | \
                             CPU0_FAULT_COMMAND_TARGET_TIMEOUT | \
                             CPU0_FAULT_TARGET_UPDATE | \
                             CPU0_FAULT_USB_INIT)

#endif /* SEROV_CPU0_TASK_COMMON_H */
