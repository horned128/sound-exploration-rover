/** =================================================================*
 * @file   task_registry.c
 * @brief  CPU0タスク群の初期化
 * ================================================================= */
#include "task_registry.h"                                  /* CPU0タスク初期化API */
#include "task_acoustic_link.h"                             /* 音響リンクタスク生成・開始API */
#include "task_command.h"                                   /* 指令タスク生成・開始API */
#include "task_sensor.h"                                    /* センサータスク生成・開始API */
#include "task_think.h"                                     /* 思考タスク生成・開始API */
#include "config/sensor_config.h"                           /* I2Cセンサータスク有効化設定 */

typedef app_fault_t (*task_registry_create_t)(void);
typedef app_fault_t (*task_registry_start_t)(void);
typedef void (*task_registry_delete_t)(void);

/**< CPU0タスクの生成・開始・解放API登録 */
typedef struct st_task_registry_registration {
    task_registry_create_t create;                          /**< タスク生成関数 */
    task_registry_start_t start;                            /**< タスク開始関数 */
    task_registry_delete_t delete;                          /**< タスク解放関数 */
} task_registry_registration_t;

LOCAL void task_registry_delete(UW task_count);             /* 登録済みタスク逆順解放 */

/**< CPU0タスクの生成、開始、解放API登録。deleteはcreate失敗後にも安全に呼べること。 */
LOCAL task_registry_registration_t const task_registry_entries[] = {
#if (CPU0_SENSOR_I2C_ENABLED != 0U)
    {
        .create = task_sensor_create,
        .start = task_sensor_start,
        .delete = task_sensor_delete,
    },
#endif
    {
        .create = task_think_create,
        .start = task_think_start,
        .delete = task_think_delete,
    },
    {
        .create = task_command_create,
        .start = task_command_start,
        .delete = task_command_delete,
    },
    {
        .create = task_acoustic_link_create,
        .start = task_acoustic_link_start,
        .delete = task_acoustic_link_delete,
    },
};

#define CPU0_TASK_COUNT                    /**< CPU0タスク登録数 */ \
    ((UW) (sizeof(task_registry_entries) / sizeof(task_registry_entries[0])))

/** =================================================================*
 * @brief  CPU0タスク群の生成・開始
 * @details μT-Kernel初期タスクから呼び出し、全タスクを生成してから登録順に開始する。
 *          失敗時は生成済みリソースを登録の逆順で解放する。
 * @return CPU0異常コード
 * ================================================================= */
EXPORT app_fault_t task_registry_init(void) {
    for (UW index = 0U; index < CPU0_TASK_COUNT; index++) {
        app_fault_t const fault = task_registry_entries[index].create();
        if (APP_FAULT_NONE != fault) {
            task_registry_delete(index + 1U);
            return fault;
        }
    }

    for (UW index = 0U; index < CPU0_TASK_COUNT; index++) {
        app_fault_t const fault = task_registry_entries[index].start();
        if (APP_FAULT_NONE != fault) {
            task_registry_delete(CPU0_TASK_COUNT);
            return fault;
        }
    }

    return APP_FAULT_NONE;
}

/** =================================================================*
 * @brief  CPU0タスク群の逆順解放
 * @param[in] task_count 解放する登録済みタスク数
 * ================================================================= */
LOCAL void task_registry_delete(UW task_count) {
    while (task_count > 0U) {
        task_count--;
        task_registry_entries[task_count].delete();
    }
}
