/** =================================================================*
 * @file   tk_command.h
 * @brief  CPU0指令タスクAPI
 * ================================================================= */
#ifndef SEROV_CPU0_TK_COMMAND_H
#define SEROV_CPU0_TK_COMMAND_H

#include <stdbool.h>                                       /* bool型 */
#include <stdint.h>                                        /* 固定幅整数型 */
#include <tk/tkernel.h>                                    /* μT-Kernel型 */
#include "hal_data.h"                                     /* FSPエラー型 */
#include "task_common.h"                                  /* CPU0タスク共通異常型 */
#include "../../../../common/ipc_message.h"                /* サーボ数、IPCメッセージ型 */

typedef struct st_rover_motion_target {
    int16_t left_target_rpm;
    int16_t right_target_rpm;
    int16_t servo_target_deg[ACTUATOR_SERVO_COUNT];
    bool actuator_enable;
    bool emergency_stop;
} rover_motion_target_t;

typedef struct st_cpu0_command_snapshot {
    rover_motion_target_t target;
    rover_motion_target_t last_sent_target;
    uint32_t target_age_ms;
    bool target_stale;
} cpu0_command_snapshot_t;

cpu0_fault_t cpu0_command_task_create(void);               /* 指令タスクと共有資源の生成 */
cpu0_fault_t cpu0_command_task_start(void);                /* 指令タスク開始 */
void cpu0_command_task_delete(void);                       /* 指令タスクと共有資源の解放 */
ER cpu0_command_set_target(const rover_motion_target_t * p_target); /* 最新目標更新 */
ER cpu0_command_snapshot_get(
    cpu0_command_snapshot_t * p_snapshot);                /* 最新指令状態取得 */

extern volatile uint32_t g_cpu0_command_sequence;          /**< 最終送信シーケンス（Live Watch用） */
extern volatile uint32_t g_cpu0_command_send_count;        /**< 正常送信回数（Live Watch用） */
extern volatile fsp_err_t g_cpu0_command_last_error;       /**< 最終IPCエラー（Live Watch用） */

#endif /* SEROV_CPU0_TK_COMMAND_H */
