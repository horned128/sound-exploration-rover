/** =================================================================*
 * @file   tk_think.h
 * @brief  CPU0思考タスクAPI
 * ================================================================= */
#ifndef SEROV_CPU0_TK_THINK_H
#define SEROV_CPU0_TK_THINK_H

#include "../control/sound_follow_controller.h"             /* 思考状態型 */
#include "task_common.h"                                    /* CPU0タスク共通異常型 */
#include <tk/tkernel.h>                                     /* μT-Kernel型 */

EXPORT cpu0_fault_t cpu0_think_task_create(void);                  /* 思考タスクとイベント生成 */
EXPORT cpu0_fault_t cpu0_think_task_start(void);                   /* 思考タスク開始 */
EXPORT void cpu0_think_task_delete(void);                          /* 思考タスクとイベント解放 */
EXPORT ER cpu0_think_report_fault(cpu0_fault_t fault);             /* 他タスクからの異常通知 */
EXPORT void cpu0_think_halt(cpu0_fault_t fault);                   /* 起動不能時のLED表示 */

IMPORT volatile cpu0_think_state_t g_cpu0_think_state;      /**< 現在の思考状態（Live Watch用） */
IMPORT volatile UW g_cpu0_think_cycle_count;          /**< 思考周期実行回数（Live Watch用） */
IMPORT volatile UW g_cpu0_think_observation_sequence; /**< 最終判断観測sequence（Live Watch用） */
/**< 観測更新停止時間（Live Watch用） */
IMPORT volatile UW g_cpu0_think_observation_watchdog_ms;
IMPORT volatile BOOL g_cpu0_think_link_ready;               /**< 音響リンク判断（Live Watch用） */
IMPORT volatile BOOL g_cpu0_think_new_observation;          /**< 新規観測判断（Live Watch用） */
IMPORT volatile H g_cpu0_think_steering_deg;          /**< 操舵判断値（Live Watch用） */
IMPORT volatile H g_cpu0_think_left_rpm;              /**< 左RPM判断値（Live Watch用） */
IMPORT volatile H g_cpu0_think_right_rpm;             /**< 右RPM判断値（Live Watch用） */
IMPORT volatile BOOL g_cpu0_think_actuator_enable;          /**< 出力許可判断（Live Watch用） */
IMPORT volatile BOOL g_cpu0_think_emergency_stop;           /**< 非常停止判断（Live Watch用） */
IMPORT volatile UW g_cpu0_fault_flags;                /**< CPU0異常ラッチ（Live Watch用） */

#endif /* SEROV_CPU0_TK_THINK_H */
