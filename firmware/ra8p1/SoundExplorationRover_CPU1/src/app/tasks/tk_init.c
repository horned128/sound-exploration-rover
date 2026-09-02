/** =================================================================*
 * @file   tk_init.c
 * @brief  CPU1タスク群の初期化
 * ================================================================= */
#include "tk_init.h"                                        /* CPU1タスク初期化API */
#include "tk_actuator.h"                                    /* アクチュエータタスク生成・開始API */
#include "tk_status.h"                                      /* 状態表示タスク生成・開始API */

typedef cpu1_fault_t (*cpu1_task_create_t)(void);
typedef cpu1_fault_t (*cpu1_task_start_t)(void);
typedef void (*cpu1_task_delete_t)(void);

typedef struct st_cpu1_task_registration {
    cpu1_task_create_t create;
    cpu1_task_start_t start;
    cpu1_task_delete_t delete;
} cpu1_task_registration_t;

LOCAL void cpu1_tasks_delete(UW task_count);                /* 登録済みタスク逆順解放 */

/**< CPU1タスクの生成、開始、解放API登録。 */
LOCAL cpu1_task_registration_t const cpu1_task_registry[] = {
    {
        .create = cpu1_actuator_task_create,
        .start = cpu1_actuator_task_start,
        .delete = cpu1_actuator_task_delete,
    },
    {
        .create = cpu1_status_task_create,
        .start = cpu1_status_task_start,
        .delete = cpu1_status_task_delete,
    },
};

#define CPU1_TASK_COUNT                    ((UW) (sizeof(cpu1_task_registry) / sizeof(cpu1_task_registry[0])))

/** =================================================================*
 * @brief  CPU1タスク群の生成・開始
 * @details 全タスクを生成してから登録順に開始し、失敗時は逆順で解放する。
 * @return CPU1異常コード
 * ================================================================= */
EXPORT cpu1_fault_t cpu1_tasks_init(void) {
    for (UW index = 0U; index < CPU1_TASK_COUNT; index++) {
        cpu1_fault_t const fault = cpu1_task_registry[index].create();
        if (CPU1_FAULT_NONE != fault) {
            cpu1_tasks_delete(index + 1U);
            return fault;
        }
    }

    for (UW index = 0U; index < CPU1_TASK_COUNT; index++) {
        cpu1_fault_t const fault = cpu1_task_registry[index].start();
        if (CPU1_FAULT_NONE != fault) {
            cpu1_tasks_delete(CPU1_TASK_COUNT);
            return fault;
        }
    }

    return CPU1_FAULT_NONE;
}

/** =================================================================*
 * @brief  CPU1タスク群の逆順解放
 * @param[in] task_count 解放する登録済みタスク数
 * ================================================================= */
LOCAL void cpu1_tasks_delete(UW task_count) {
    while (task_count > 0U) {
        task_count--;
        cpu1_task_registry[task_count].delete();
    }
}
