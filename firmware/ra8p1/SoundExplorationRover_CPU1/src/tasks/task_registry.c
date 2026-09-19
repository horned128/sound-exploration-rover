/** =================================================================*
 * @file   task_registry.c
 * @brief  CPU1タスク群の初期化
 * ================================================================= */
#include "task_registry.h"                                        /* CPU1タスク初期化API */
#include "task_safety.h"                                      /* 安全ウォッチドッグタスクAPI */
#include "task_actuator.h"                                    /* アクチュエータタスク生成・開始API */
#include "task_status.h"                                      /* 状態表示タスク生成・開始API */

typedef app_fault_t (*task_registry_create_t)(void);
typedef app_fault_t (*task_registry_start_t)(void);
typedef void (*task_registry_delete_t)(void);

typedef struct st_task_registry_registration {
    task_registry_create_t create;
    task_registry_start_t start;
    task_registry_delete_t delete;
} task_registry_registration_t;

LOCAL void task_registry_delete(UW task_count);                /* 登録済みタスク逆順解放 */

/**< CPU1タスクの生成、開始、解放API登録。 */
LOCAL task_registry_registration_t const task_registry_entries[] = {
    {
        .create = task_safety_create,
        .start = task_safety_start,
        .delete = task_safety_delete,
    },
    {
        .create = task_actuator_create,
        .start = task_actuator_start,
        .delete = task_actuator_delete,
    },
    {
        .create = task_status_create,
        .start = task_status_start,
        .delete = task_status_delete,
    },
};

#define CPU1_TASK_COUNT                    ((UW) (sizeof(task_registry_entries) / sizeof(task_registry_entries[0])))

/** =================================================================*
 * @brief  CPU1タスク群の生成・開始
 * @details 全タスクを生成してから登録順に開始し、失敗時は逆順で解放する。
 * @return CPU1異常コード
 * ================================================================= */
EXPORT app_fault_t task_registry_init(void) {
    for (UW index = 0U; index < CPU1_TASK_COUNT; index++) {
        app_fault_t const fault = task_registry_entries[index].create();
        if (APP_FAULT_NONE != fault) {
            task_registry_delete(index + 1U);
            return fault;
        }
    }

    for (UW index = 0U; index < CPU1_TASK_COUNT; index++) {
        app_fault_t const fault = task_registry_entries[index].start();
        if (APP_FAULT_NONE != fault) {
            task_registry_delete(CPU1_TASK_COUNT);
            return fault;
        }
    }

    return APP_FAULT_NONE;
}

/** =================================================================*
 * @brief  CPU1タスク群の逆順解放
 * @param[in] task_count 解放する登録済みタスク数
 * ================================================================= */
LOCAL void task_registry_delete(UW task_count) {
    while (task_count > 0U) {
        task_count--;
        task_registry_entries[task_count].delete();
    }
}
