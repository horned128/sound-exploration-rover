/** =================================================================*
 * @file   task_command.c
 * @brief  CPU0指令タスク実装
 * ================================================================= */
#include "task_command.h"                                   /* CPU0指令タスクAPI */
#include "config/task_config.h"                             /* 指令周期、優先度、タイムアウト */
#include "ipc/actuator_ipc_client.h"                        /* CPU1へのIPC送信API */
#include "task_think.h"                                     /* 思考タスクへの異常通知 */

#define CPU0_COMMAND_IPC_RECOVERY_STATUS_COUNT (2U)

/**< 最新アクチュエータ目標を保護するμT-Kernel mutex設定 */
LOCAL T_CMTX const command_mutex_config = {
    .mtxatr = TA_INHERIT,
    .ceilpri = 0,
};

LOCAL void task_command_entry(INT stacd, void * exinf);                     /* 指令タスク本体 */
LOCAL void task_command_send_latest(void);                                  /* 最新指令スナップショット送信 */
LOCAL BOOL task_command_sequence_reached(UW actual, UW reference);          /* 24 bit sequence到達判定 */
LOCAL void task_command_recovery_check(const actuator_status_t * p_status); /* IPC回復確認 */

/**< CPU1へIPC指令を送信するタスク設定 */
LOCAL T_CTSK const command_task_config = {
    .exinf = NULL,
    .tskatr = TA_HLNG | TA_RNG3,
    .task = (FP) task_command_entry,
    .itskpri = CPU0_COMMAND_TASK_PRIORITY,
    .stksz = CPU0_COMMAND_TASK_STACK_SIZE,
    .bufptr = NULL,
};

LOCAL ID command_task_id;                                  /**< 指令タスクID */
LOCAL ID command_mutex_id;                                 /**< 最新目標保護mutex ID */
LOCAL BOOL command_task_started;                           /**< 指令タスク開始状態 */
LOCAL BOOL command_ipc_open;                               /**< IPC open状態 */
LOCAL BOOL command_emergency_reset_pending;                /**< CPU1 estopラッチ解除待ち */
LOCAL BOOL command_timeout_reported;                       /**< 目標期限切れ通知済み状態 */
LOCAL BOOL command_target_valid;                           /**< 思考タスクの目標受信済み状態 */
LOCAL UW command_target_age_ms;                            /**< 最新目標の経過時間 */
LOCAL BOOL command_ipc_fault_active;                        /**< IPC送信異常からの回復待ち */
LOCAL BOOL command_recovery_safe_sequence_valid;            /**< 回復用安全指令を送信済み */
LOCAL UW command_recovery_safe_sequence;                    /**< 最初に正常送信した安全指令sequence */
LOCAL UW command_recovery_status_sequence;                  /**< 最後に評価したCPU1状態sequence */
LOCAL UB command_recovery_status_count;                     /**< 正常なCPU1応答の連続確認数 */

/**< 思考タスクが更新する最新アクチュエータ目標 */
LOCAL rover_motion_target_t command_target = {
    .left_target_rpm = 0,
    .right_target_rpm = 0,
    .actuator_enable = FALSE,
    .emergency_stop = TRUE,
};
/**< IPCでCPU1へ最後に送信したアクチュエータ目標 */
LOCAL rover_motion_target_t command_last_sent_target = {
    .left_target_rpm = 0,
    .right_target_rpm = 0,
    .actuator_enable = FALSE,
    .emergency_stop = TRUE,
};

EXPORT volatile UW g_task_command_sequence;                  /**< 最終送信シーケンス */
EXPORT volatile UW g_task_command_send_count;                /**< 正常送信回数 */
EXPORT volatile fsp_err_t g_task_command_last_error;         /**< 最終IPCエラー */
EXPORT volatile BOOL g_task_command_peer_ready;               /**< CPU1状態受信済み */
EXPORT volatile BOOL g_task_command_target_valid;             /**< 思考タスクの目標受信済み */

/** =================================================================*
 * @brief  指令タスクと共有資源生成
 * @return CPU0異常コード
 * ================================================================= */
EXPORT app_fault_t task_command_create(void) {
    command_task_id = 0;
    command_mutex_id = 0;
    command_task_started = FALSE;
    command_ipc_open = FALSE;
    command_emergency_reset_pending = TRUE;
    command_timeout_reported = FALSE;
    command_target_valid = FALSE;
    command_target_age_ms = CPU0_COMMAND_TARGET_TIMEOUT_MS;
    command_ipc_fault_active = FALSE;
    command_recovery_safe_sequence_valid = FALSE;
    command_recovery_safe_sequence = 0U;
    command_recovery_status_sequence = 0U;
    command_recovery_status_count = 0U;
    command_target = (rover_motion_target_t){
        .left_target_rpm = 0,
        .right_target_rpm = 0,
        .actuator_enable = FALSE,
        .emergency_stop = TRUE,
    };
    command_last_sent_target = command_target;
    g_task_command_sequence = 0U;
    g_task_command_send_count = 0U;
    g_task_command_last_error = FSP_SUCCESS;
    g_task_command_peer_ready = FALSE;
    g_task_command_target_valid = FALSE;

    command_mutex_id = tk_cre_mtx(&command_mutex_config);
    if (command_mutex_id <= 0) {
        command_mutex_id = 0;
        return APP_FAULT_TASK_CREATE;
    }

    g_task_command_last_error = actuator_ipc_client_init();
    if (FSP_SUCCESS != g_task_command_last_error) {
        task_command_delete();
        return APP_FAULT_IPC_INIT;
    }
    command_ipc_open = TRUE;

    command_task_id = tk_cre_tsk(&command_task_config);
    if (command_task_id <= 0) {
        command_task_id = 0;
        task_command_delete();
        return APP_FAULT_TASK_CREATE;
    }

    return APP_FAULT_NONE;
}

/** =================================================================*
 * @brief  指令タスク開始
 * @return CPU0異常コード
 * ================================================================= */
EXPORT app_fault_t task_command_start(void) {
    if (command_task_id <= 0) {
        return APP_FAULT_TASK_CREATE;
    }

    ER const err = tk_sta_tsk(command_task_id, 0);
    if (E_OK != err) {
        return APP_FAULT_TASK_START;
    }
    command_task_started = TRUE;
    return APP_FAULT_NONE;
}

/** =================================================================*
 * @brief  指令タスクと共有資源解放
 * ================================================================= */
EXPORT void task_command_delete(void) {
    if (command_task_id > 0) {
        if (command_task_started) {
            (void) tk_ter_tsk(command_task_id);
        }
        (void) tk_del_tsk(command_task_id);
        command_task_id = 0;
        command_task_started = FALSE;
    }

    if (command_ipc_open) {
        (void) actuator_ipc_client_deinit();
        command_ipc_open = FALSE;
    }

    if (command_mutex_id > 0) {
        (void) tk_del_mtx(command_mutex_id);
        command_mutex_id = 0;
    }
}

/** =================================================================*
 * @brief  思考タスク目標更新
 * @param[in] p_target FR/FL/RR/RLと左右モーターを含む目標
 * @return μT-Kernelエラーコード
 * ================================================================= */
EXPORT ER task_command_set_target(const rover_motion_target_t * p_target) {
    if (NULL == p_target) {
        return E_PAR;
    }
    if (command_mutex_id <= 0) {
        return E_NOEXS;
    }

    ER err = tk_loc_mtx(command_mutex_id, TMO_FEVR);
    if (E_OK == err) {
        command_target = *p_target;
        command_target_valid = TRUE;
        command_target_age_ms = 0U;
        command_timeout_reported = FALSE;
        g_task_command_target_valid = TRUE;
        err = tk_unl_mtx(command_mutex_id);
    }

    return err;
}

/** =================================================================*
 * @brief  最新指令状態取得
 * @param[out] p_snapshot 最新指令と経過時間
 * @return μT-Kernelエラーコード
 * ================================================================= */
EXPORT ER task_command_snapshot_get(task_command_snapshot_t * p_snapshot) {
    if (NULL == p_snapshot) {
        return E_PAR;
    }
    if (command_mutex_id <= 0) {
        return E_NOEXS;
    }

    ER err = tk_loc_mtx(command_mutex_id, TMO_POL);
    if (E_OK != err) {
        return err;
    }

    p_snapshot->target = command_target;
    p_snapshot->last_sent_target = command_last_sent_target;
    p_snapshot->target_age_ms = command_target_age_ms;
    p_snapshot->target_valid = command_target_valid;
    p_snapshot->target_stale = command_target_valid &&
                              (command_target_age_ms >= CPU0_COMMAND_TARGET_TIMEOUT_MS);
    err = tk_unl_mtx(command_mutex_id);
    return err;
}

/** =================================================================*
 * @brief  周回する24 bit sequenceが基準へ到達済みか判定
 * @details 差が半周未満ならactualはreferenceと同じか、それより新しい。
 * ================================================================= */
LOCAL BOOL task_command_sequence_reached(UW actual, UW reference) {
    UW const distance = (actual - reference) & ACTUATOR_IPC_SEQUENCE_MASK;
    return distance <= (ACTUATOR_IPC_SEQUENCE_MASK >> 1U);
}

/** =================================================================*
 * @brief  IPC送信異常後の双方向回復確認
 * @details 新しいCPU1状態フレームで、安全指令の適用と永続異常なしを
 *          連続確認してからAPP_FAULT_IPC_SENDだけを解除する。
 * ================================================================= */
LOCAL void task_command_recovery_check(const actuator_status_t * p_status) {
    if (!command_ipc_fault_active || !command_recovery_safe_sequence_valid || (NULL == p_status) ||
        (p_status->sequence_number == command_recovery_status_sequence)) {
        return;
    }
    command_recovery_status_sequence = p_status->sequence_number;

    UH const unexpected_faults =
        p_status->fault_flags & (UH) ~ACTUATOR_FAULT_EMERGENCY_STOP_ACTIVE;
    if ((0U == unexpected_faults) &&
        task_command_sequence_reached(p_status->applied_command_sequence, command_recovery_safe_sequence)) {
        if (command_recovery_status_count < CPU0_COMMAND_IPC_RECOVERY_STATUS_COUNT) {
            command_recovery_status_count++;
        }
    } else {
        command_recovery_status_count = 0U;
    }

    if ((command_recovery_status_count >= CPU0_COMMAND_IPC_RECOVERY_STATUS_COUNT) &&
        (E_OK == task_think_clear_fault(APP_FAULT_IPC_SEND))) {
        command_ipc_fault_active = FALSE;
        command_recovery_safe_sequence_valid = FALSE;
        command_recovery_status_count = 0U;
    }
}

/** =================================================================*
 * @brief  最新指令スナップショット送信
 * @details 4サーボと左右モーターを1つのIPCフレームとして同時commitする。
 * ================================================================= */
LOCAL void task_command_send_latest(void) {
    rover_motion_target_t target;
    actuator_status_t peer_status;
    BOOL target_stale;
    BOOL report_timeout;

    ER const lock_err = tk_loc_mtx(command_mutex_id, TMO_FEVR);
    if (E_OK != lock_err) {
        g_task_command_last_error = actuator_ipc_client_emergency_stop(++g_task_command_sequence);
        (void) task_think_report_fault(APP_FAULT_TARGET_UPDATE);
        return;
    }

    if (command_target_age_ms < CPU0_COMMAND_TARGET_TIMEOUT_MS) {
        command_target_age_ms += CPU0_COMMAND_PERIOD_MS;
        if (command_target_age_ms > CPU0_COMMAND_TARGET_TIMEOUT_MS) {
            command_target_age_ms = CPU0_COMMAND_TARGET_TIMEOUT_MS;
        }
    }

    target_stale = command_target_valid &&
                   (command_target_age_ms >= CPU0_COMMAND_TARGET_TIMEOUT_MS);
    report_timeout = target_stale && !command_timeout_reported;
    if (report_timeout) {
        command_timeout_reported = TRUE;
    }
    target = command_target;
    (void) tk_unl_mtx(command_mutex_id);

    if (report_timeout) {
        (void) task_think_report_fault(APP_FAULT_COMMAND_TARGET_TIMEOUT);
    }
    if (target_stale) {
        target.actuator_enable = FALSE;
        target.emergency_stop = TRUE;
    }

    /* CPU1がIPCをopenして状態フレームを返すまで、FIFOへ指令を積まない。 */
    if (!actuator_ipc_client_status_get(&peer_status)) {
        return;
    }
    g_task_command_peer_ready = TRUE;
    task_command_recovery_check(&peer_status);

    /* IPC復旧中は思考タスクの反映周期に依存せず、CPU1へ安全停止を送り続ける。 */
    if (command_ipc_fault_active) {
        target.left_target_rpm = 0;
        target.right_target_rpm = 0;
        target.actuator_enable = FALSE;
        target.emergency_stop = TRUE;
        for (UW i = 0U; i < ACTUATOR_SERVO_COUNT; i++) {
            target.servo_target_deg[i] = 0;
        }
    }

    BOOL const clear_emergency_latch = command_emergency_reset_pending && !target_stale && !target.emergency_stop;

    actuator_command_t command = actuator_command_make_safe();
    command.left_target_rpm = target.left_target_rpm;
    command.right_target_rpm = target.right_target_rpm;
    command.actuator_enable = (clear_emergency_latch ? FALSE : target.actuator_enable) ? 1U : 0U;
    command.emergency_stop = (clear_emergency_latch ? FALSE : target.emergency_stop) ? 1U : 0U;
    for (UW i = 0U; i < ACTUATOR_SERVO_COUNT; i++) {
        command.servo_target_deg[i] = target.servo_target_deg[i];
    }
    command.sequence_number = ++g_task_command_sequence;

    g_task_command_last_error = actuator_ipc_client_send(&command);
    if (FSP_SUCCESS != g_task_command_last_error) {
        (void) actuator_ipc_client_emergency_stop(++g_task_command_sequence);
        command_ipc_fault_active = TRUE;
        command_recovery_safe_sequence_valid = FALSE;
        command_recovery_status_sequence = peer_status.sequence_number;
        command_recovery_status_count = 0U;
        (void) task_think_report_fault(APP_FAULT_IPC_SEND);
    } else {
        g_task_command_send_count++;
        if (command_ipc_fault_active && !command_recovery_safe_sequence_valid) {
            command_recovery_safe_sequence = command.sequence_number & ACTUATOR_IPC_SEQUENCE_MASK;
            command_recovery_safe_sequence_valid = TRUE;
        }
        ER const sent_lock_err = tk_loc_mtx(command_mutex_id, TMO_FEVR);
        if (E_OK == sent_lock_err) {
            command_last_sent_target.left_target_rpm = command.left_target_rpm;
            command_last_sent_target.right_target_rpm = command.right_target_rpm;
            command_last_sent_target.actuator_enable = 0U != command.actuator_enable;
            command_last_sent_target.emergency_stop = 0U != command.emergency_stop;
            for (UW i = 0U; i < ACTUATOR_SERVO_COUNT; i++) {
                command_last_sent_target.servo_target_deg[i] = command.servo_target_deg[i];
            }
            (void) tk_unl_mtx(command_mutex_id);
        }
        if (clear_emergency_latch) {
            command_emergency_reset_pending = FALSE;
        } else if (target.emergency_stop) {
            command_emergency_reset_pending = TRUE;
        }
    }
}

/** =================================================================*
 * @brief  指令タスク本体
 * ================================================================= */
LOCAL void task_command_entry(INT stacd, void * exinf) {
    (void) stacd;
    (void) exinf;

    (void) tk_dly_tsk(CPU0_ACTUATOR_STARTUP_DELAY_MS);

    while (1) {
        task_command_send_latest();
        (void) tk_dly_tsk(CPU0_COMMAND_PERIOD_MS);
    }
}
