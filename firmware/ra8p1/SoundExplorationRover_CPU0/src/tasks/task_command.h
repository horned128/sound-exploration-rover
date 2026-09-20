/** =================================================================*
 * @file   task_command.h
 * @brief  CPU0指令タスクAPI
 * ================================================================= */
#ifndef SEROV_CPU0_TASK_COMMAND_H
#define SEROV_CPU0_TASK_COMMAND_H

#include "../../../common/ipc_message.h"                    /* サーボ数、IPCメッセージ型 */
#include "hal_data.h"                                       /* FSPエラー型 */
#include "task_common.h"                                    /* CPU0タスク共通異常型 */
#include <tk/tkernel.h>                                     /* μT-Kernel型 */

/**< CPU1へ送る左右車輪・サーボの走行目標 */
typedef struct st_rover_motion_target {
    H left_target_rpm;                                      /**< 左車輪目標RPM */
    H right_target_rpm;                                     /**< 右車輪目標RPM */
    H servo_target_deg[ACTUATOR_SERVO_COUNT];               /**< 各サーボ目標角度[deg] */
    BOOL actuator_enable;                                   /**< アクチュエータ出力許可 */
    BOOL emergency_stop;                                    /**< 非常停止指令 */
} rover_motion_target_t;

/**< 指令タスクの最新目標と送信監視状態 */
typedef struct st_task_command_snapshot {
    rover_motion_target_t target;                           /**< 思考タスクから受けた最新目標 */
    rover_motion_target_t last_sent_target;                 /**< CPU1へ最後に送った目標 */
    UW target_age_ms;                                       /**< 最新目標受信からの経過時間[ms] */
    BOOL target_valid;                                      /**< 最新目標の有効状態 */
    BOOL target_stale;                                      /**< 最新目標の期限超過状態 */
} task_command_snapshot_t;

EXPORT app_fault_t task_command_create(void);               /* 指令タスクと共有資源の生成 */
EXPORT app_fault_t task_command_start(void);                /* 指令タスク開始 */
EXPORT void task_command_delete(void);                      /* 指令タスクと共有資源の解放 */
EXPORT ER task_command_set_target(const rover_motion_target_t * p_target); /* 最新目標更新 */
EXPORT ER task_command_snapshot_get(task_command_snapshot_t * p_snapshot); /* 最新指令状態取得 */

IMPORT volatile UW g_task_command_sequence;                 /**< 最終送信シーケンス（Live Watch用） */
IMPORT volatile UW g_task_command_send_count;               /**< 正常送信回数（Live Watch用） */
IMPORT volatile fsp_err_t g_task_command_last_error;        /**< 最終IPCエラー（Live Watch用） */
IMPORT volatile BOOL g_task_command_peer_ready;             /**< CPU1状態受信済み（監視用） */
IMPORT volatile BOOL g_task_command_target_valid;           /**< 思考タスクの目標受信済み（監視用） */

#endif /* SEROV_CPU0_TASK_COMMAND_H */
