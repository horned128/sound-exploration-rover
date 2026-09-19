/** =================================================================*
 * @file   task_safety.c
 * @brief  CPU1独立安全ウォッチドッグタスク実装（4-7-b）
 * @details 制御タスク(task_actuator: 優先度4)より高い優先度3で動作し、
 *          制御タスクのハングまたは指令途絶時に独立して即座に安全停止を行う。
 * ================================================================= */
#include "task_safety.h"
#include "config/task_config.h"
#include "drivers/bts7960.h"
#include "services/actuator_service.h"
#include "task_actuator.h"
#include <tk/tkernel.h>

#define CPU1_SAFETY_ACTUATOR_HANG_TIMEOUT_MS  (100U)

LOCAL void task_safety_entry(INT start_code, void * p_extended_information);

LOCAL ID safety_task_id = 0;
EXPORT volatile BOOL g_task_safety_actuator_hang_detected = FALSE;
EXPORT volatile BOOL g_task_safety_timeout_detected = FALSE;

LOCAL T_CTSK const safety_task_config = {
    .exinf   = NULL,
    .tskatr  = TA_HLNG | TA_RNG3,
    .task    = task_safety_entry,
    .itskpri = CPU1_SAFETY_TASK_PRIORITY,
    .stksz   = CPU1_SAFETY_TASK_STACK_SIZE,
};

/** =================================================================*
 * @brief  安全タスク生成
 * @return CPU1異常コード
 * ================================================================= */
EXPORT app_fault_t task_safety_create(void) {
    g_task_safety_actuator_hang_detected = FALSE;
    g_task_safety_timeout_detected = FALSE;

    ID const task_id = tk_cre_tsk(&safety_task_config);
    if (task_id <= 0) {
        return APP_FAULT_ACTUATOR_TASK_CREATE;
    }
    safety_task_id = task_id;
    return APP_FAULT_NONE;
}

/** =================================================================*
 * @brief  安全タスク開始
 * @return CPU1異常コード
 * ================================================================= */
EXPORT app_fault_t task_safety_start(void) {
    if (safety_task_id <= 0) {
        return APP_FAULT_ACTUATOR_TASK_START;
    }
    ER const err = tk_sta_tsk(safety_task_id, 0);
    if (E_OK != err) {
        task_safety_delete();
        return APP_FAULT_ACTUATOR_TASK_START;
    }
    return APP_FAULT_NONE;
}

/** =================================================================*
 * @brief  安全タスク解放
 * ================================================================= */
EXPORT void task_safety_delete(void) {
    if (safety_task_id > 0) {
        (void) tk_ter_tsk(safety_task_id);
        (void) tk_del_tsk(safety_task_id);
        safety_task_id = 0;
    }
}

/** =================================================================*
 * @brief  独立安全ウォッチドッグタスク本体
 * @param[in] start_code 開始コード
 * @param[in] p_extended_information 拡張情報
 * ================================================================= */
LOCAL void task_safety_entry(INT start_code, void * p_extended_information) {
    (void) start_code;
    (void) p_extended_information;

    UW last_actuator_count = g_task_actuator_update_count;
    UW actuator_stall_ms = 0U;

    /* 起動直後は初期化完了を待機 */
    (void) tk_dly_tsk(100U);

    while (1) {
        (void) tk_dly_tsk(CPU1_SAFETY_TASK_PERIOD_MS);

        UW const current_actuator_count = g_task_actuator_update_count;
        if (current_actuator_count == last_actuator_count) {
            /* 1 ms周期の制御タスクが進んでいない */
            actuator_stall_ms += CPU1_SAFETY_TASK_PERIOD_MS;
            if (actuator_stall_ms >= CPU1_SAFETY_ACTUATOR_HANG_TIMEOUT_MS) {
                /*
                 * 4-7-b: 制御タスク(task_actuator)のハング検知
                 * 高優先度から直接モーター出力を安全停止する。
                 */
                g_task_safety_actuator_hang_detected = TRUE;
                (void) bts7960_stop();
            }
        } else {
            /* 制御タスクは正常に周期実行中 */
            last_actuator_count = current_actuator_count;
            actuator_stall_ms = 0U;
        }

        /* 指令タイムアウトの多重監視 */
        actuator_status_t status = {0};
        actuator_service_status_get(&status);
        if (0U != (status.fault_flags & ACTUATOR_FAULT_COMMAND_TIMEOUT)) {
            g_task_safety_timeout_detected = TRUE;
            (void) bts7960_stop();
        }
    }
}
