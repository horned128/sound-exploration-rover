/** =================================================================*
 * @file   task_think.h
 * @brief  CPU0思考タスクAPI
 * ================================================================= */
#ifndef SEROV_CPU0_TASK_THINK_H
#define SEROV_CPU0_TASK_THINK_H

#include "control/sound_follow_controller.h"                /* 思考状態型 */
#include "control/obstacle_avoidance_controller.h"          /* センサー走行ルール型 */
#include "services/prototype_storage.h"                     /* MRAM保存結果型 */
#include "task_common.h"                                    /* CPU0タスク共通異常型 */
#include <tk/tkernel.h>                                     /* μT-Kernel型 */

/**< UIやUSB診断経路から思考タスクへ渡す現場学習操作 */
typedef enum e_task_think_learning_command {
    TASK_THINK_LEARNING_COMMAND_NONE = 0U,                 /**< 操作なし */
    TASK_THINK_LEARNING_COMMAND_START,                     /**< 見本収集開始 */
    TASK_THINK_LEARNING_COMMAND_COMMIT,                    /**< 見本保存要求 */
    TASK_THINK_LEARNING_COMMAND_CANCEL,                    /**< 未保存見本破棄 */
} task_think_learning_command_t;

EXPORT app_fault_t task_think_create(void);                 /* 思考タスクとイベント生成 */
EXPORT app_fault_t task_think_start(void);                  /* 思考タスク開始 */
EXPORT void task_think_delete(void);                        /* 思考タスクとイベント解放 */
EXPORT ER task_think_report_fault(app_fault_t fault);       /* 他タスクからの異常通知 */
EXPORT ER task_think_clear_fault(app_fault_t fault);        /* 回復確認済み異常の解除通知 */
EXPORT void task_think_halt(app_fault_t fault);             /* 起動不能時のLED表示 */
EXPORT ER task_think_learning_request(task_think_learning_command_t command); /* 現場学習操作要求 */

IMPORT volatile sound_follow_state_t g_task_think_state;    /**< 現在の思考状態（Live Watch用） */
IMPORT volatile UW g_task_think_cycle_count;                /**< 思考周期実行回数（Live Watch用） */
IMPORT volatile UW g_task_think_observation_sequence;       /**< 最終判断観測sequence（Live Watch用） */
IMPORT volatile UW g_task_think_observation_watchdog_ms;    /**< 観測更新停止時間[ms]（Live Watch用） */
IMPORT volatile BOOL g_task_think_link_ready;               /**< 音響リンク判断（Live Watch用） */
IMPORT volatile BOOL g_task_think_new_observation;          /**< 新規観測判断（Live Watch用） */
IMPORT volatile H g_task_think_steering_deg;                /**< 操舵判断値（Live Watch用） */
IMPORT volatile H g_task_think_left_rpm;                    /**< 左RPM判断値（Live Watch用） */
IMPORT volatile H g_task_think_right_rpm;                   /**< 右RPM判断値（Live Watch用） */
IMPORT volatile BOOL g_task_think_actuator_enable;          /**< 出力許可判断（Live Watch用） */
IMPORT volatile BOOL g_task_think_emergency_stop;           /**< 非常停止判断（Live Watch用） */
/**< 採用センサー規則 */
IMPORT volatile obstacle_avoidance_rule_t g_task_think_sensor_rule;
IMPORT volatile UW g_task_think_fault_flags;                /**< CPU0異常ラッチ（Live Watch用） */
IMPORT volatile BOOL g_task_think_learning_mode;            /**< 現場学習モード */
IMPORT volatile UB g_task_think_learning_samples;           /**< 収集済み96次元見本数 */
IMPORT volatile BOOL g_task_think_storage_valid;            /**< 有効なMRAM背景モデル・見本有無 */
/**< 直近MRAM処理結果 */
IMPORT volatile prototype_storage_result_t g_task_think_storage_result;

IMPORT volatile UW g_task_think_sensor_watchdog_ms;         /**< センサー更新停止時間[ms] */
IMPORT volatile BOOL g_task_think_sensor_fresh;             /**< センサー更新期限内 */
IMPORT volatile ER g_task_think_sensor_clock_error;         /**< センサー監視の時刻取得異常 */

#endif /* SEROV_CPU0_TASK_THINK_H */
