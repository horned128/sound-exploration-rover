/** =================================================================*
 * @file   tk_init.c
 * @brief  CPU0タスク群の初期化
 * ================================================================= */
#include "tk_init.h"                                        /* CPU0タスク初期化API */
#include "tk_audio.h"                                       /* 音響タスク生成・開始API */
#include "tk_command.h"                                     /* 指令タスク生成・開始API */
#include "tk_think.h"                                       /* 思考タスク生成・開始API */
#include <stddef.h>                                          /* size_t */

typedef cpu0_fault_t (*cpu0_task_create_t)(void);
typedef cpu0_fault_t (*cpu0_task_start_t)(void);
typedef void (*cpu0_task_delete_t)(void);

typedef struct st_cpu0_task_registration {
    cpu0_task_create_t create;
    cpu0_task_start_t start;
    cpu0_task_delete_t delete;
} cpu0_task_registration_t;

static void cpu0_tasks_delete(size_t task_count);           /* 登録済みタスク逆順解放 */

/**< CPU0タスクの生成、開始、解放API登録。deleteはcreate失敗後にも安全に呼べること。 */
static cpu0_task_registration_t const cpu0_task_registry[] = {
    {
        .create = cpu0_think_task_create,
        .start = cpu0_think_task_start,
        .delete = cpu0_think_task_delete,
    },
    {
        .create = cpu0_command_task_create,
        .start = cpu0_command_task_start,
        .delete = cpu0_command_task_delete,
    },
    {
        .create = cpu0_audio_task_create,
        .start = cpu0_audio_task_start,
        .delete = cpu0_audio_task_delete,
    },
};

#define CPU0_TASK_COUNT                    (sizeof(cpu0_task_registry) / sizeof(cpu0_task_registry[0]))

/** =================================================================*
 * @brief  CPU0タスク群の生成・開始
 * @details μT-Kernel初期タスクから呼び出し、全タスクを生成してから登録順に開始する。
 *          失敗時は生成済みリソースを登録の逆順で解放する。
 * @return CPU0異常コード
 * ================================================================= */
cpu0_fault_t cpu0_tasks_init(void) {
    for (size_t index = 0U; index < CPU0_TASK_COUNT; index++) {
        cpu0_fault_t const fault = cpu0_task_registry[index].create();
        if (CPU0_FAULT_NONE != fault) {
            cpu0_tasks_delete(index + 1U);
            return fault;
        }
    }

    for (size_t index = 0U; index < CPU0_TASK_COUNT; index++) {
        cpu0_fault_t const fault = cpu0_task_registry[index].start();
        if (CPU0_FAULT_NONE != fault) {
            cpu0_tasks_delete(CPU0_TASK_COUNT);
            return fault;
        }
    }

    return CPU0_FAULT_NONE;
}

/** =================================================================*
 * @brief  CPU0タスク群の逆順解放
 * @param[in] task_count 解放する登録済みタスク数
 * ================================================================= */
static void cpu0_tasks_delete(size_t task_count) {
    while (task_count > 0U) {
        task_count--;
        cpu0_task_registry[task_count].delete();
    }
}
