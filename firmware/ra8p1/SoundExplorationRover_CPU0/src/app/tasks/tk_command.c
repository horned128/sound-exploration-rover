/** =================================================================*
 * @file   tk_command.c
 * @brief  CPU0指令タスク実装
 * ================================================================= */
#include "tk_command.h"                                   /* CPU0指令タスクAPI */
#include "tk_think.h"                                     /* 思考タスクへの異常通知 */
#include "../../cpu0_config.h"                            /* 指令周期、優先度、タイムアウト */
#include "../../ipc/actuator_ipc_client.h"                /* CPU1へのIPC送信API */

static T_CMTX const command_mutex_config = {
    .mtxatr = TA_INHERIT,
    .ceilpri = 0,
};

static void cpu0_command_task(INT stacd, void * exinf);    /* 指令タスク本体 */
static void cpu0_command_send_latest(void);                /* 最新指令スナップショット送信 */

static T_CTSK const command_task_config = {
    .exinf = NULL,
    .tskatr = TA_HLNG | TA_RNG3,
    .task = (FP) cpu0_command_task,
    .itskpri = CPU0_COMMAND_TASK_PRIORITY,
    .stksz = CPU0_COMMAND_TASK_STACK_SIZE,
    .bufptr = NULL,
};

static ID command_task_id;                                 /**< 指令タスクID */
static ID command_mutex_id;                                /**< 最新目標保護mutex ID */
static bool command_task_started;                          /**< 指令タスク開始状態 */
static bool command_ipc_open;                              /**< IPC open状態 */
static bool command_emergency_reset_pending;               /**< CPU1 estopラッチ解除待ち */
static bool command_timeout_reported;                      /**< 目標期限切れ通知済み状態 */
static uint32_t command_target_age_ms;                     /**< 最新目標の経過時間 */

static rover_motion_target_t command_target = {
    .left_target_rpm = 0,
    .right_target_rpm = 0,
    .actuator_enable = false,
    .emergency_stop = true,
};
static rover_motion_target_t command_last_sent_target = {
    .left_target_rpm = 0,
    .right_target_rpm = 0,
    .actuator_enable = false,
    .emergency_stop = true,
};

volatile uint32_t g_cpu0_command_sequence;                 /**< 最終送信シーケンス */
volatile uint32_t g_cpu0_command_send_count;               /**< 正常送信回数 */
volatile fsp_err_t g_cpu0_command_last_error;              /**< 最終IPCエラー */

/** =================================================================*
 * @brief  指令タスクと共有資源生成
 * @return CPU0異常コード
 * ================================================================= */
cpu0_fault_t cpu0_command_task_create(void) {
    command_task_id = 0;
    command_mutex_id = 0;
    command_task_started = false;
    command_ipc_open = false;
    command_emergency_reset_pending = true;
    command_timeout_reported = false;
    command_target_age_ms = CPU0_COMMAND_TARGET_TIMEOUT_MS;
    command_target = (rover_motion_target_t) {
        .left_target_rpm = 0,
        .right_target_rpm = 0,
        .actuator_enable = false,
        .emergency_stop = true,
    };
    command_last_sent_target = command_target;
    g_cpu0_command_sequence = 0U;
    g_cpu0_command_send_count = 0U;
    g_cpu0_command_last_error = FSP_SUCCESS;

    command_mutex_id = tk_cre_mtx(&command_mutex_config);
    if (command_mutex_id <= 0) {
        command_mutex_id = 0;
        return CPU0_FAULT_TASK_CREATE;
    }

    g_cpu0_command_last_error = actuator_ipc_client_init();
    if (FSP_SUCCESS != g_cpu0_command_last_error) {
        cpu0_command_task_delete();
        return CPU0_FAULT_IPC_INIT;
    }
    command_ipc_open = true;

    command_task_id = tk_cre_tsk(&command_task_config);
    if (command_task_id <= 0) {
        command_task_id = 0;
        cpu0_command_task_delete();
        return CPU0_FAULT_TASK_CREATE;
    }

    return CPU0_FAULT_NONE;
}

/** =================================================================*
 * @brief  指令タスク開始
 * @return CPU0異常コード
 * ================================================================= */
cpu0_fault_t cpu0_command_task_start(void) {
    if (command_task_id <= 0) {
        return CPU0_FAULT_TASK_CREATE;
    }

    ER const err = tk_sta_tsk(command_task_id, 0);
    if (E_OK != err) {
        return CPU0_FAULT_TASK_START;
    }
    command_task_started = true;
    return CPU0_FAULT_NONE;
}

/** =================================================================*
 * @brief  指令タスクと共有資源解放
 * ================================================================= */
void cpu0_command_task_delete(void) {
    if (command_task_id > 0) {
        if (command_task_started) {
            (void) tk_ter_tsk(command_task_id);
        }
        (void) tk_del_tsk(command_task_id);
        command_task_id = 0;
        command_task_started = false;
    }

    if (command_ipc_open) {
        (void) actuator_ipc_client_deinit();
        command_ipc_open = false;
    }

    if (command_mutex_id > 0) {
        (void) tk_del_mtx(command_mutex_id);
        command_mutex_id = 0;
    }
}

/** =================================================================*
 * @brief  思考タスクが生成した最新目標を更新
 * @param[in] p_target FR/FL/RR/RLと左右モーターを含む目標
 * @return μT-Kernelエラーコード
 * ================================================================= */
ER cpu0_command_set_target(const rover_motion_target_t * p_target) {
    if (NULL == p_target) {
        return E_PAR;
    }
    if (command_mutex_id <= 0) {
        return E_NOEXS;
    }

    ER err = tk_loc_mtx(command_mutex_id, TMO_FEVR);
    if (E_OK == err) {
        command_target = *p_target;
        command_target_age_ms = 0U;
        command_timeout_reported = false;
        err = tk_unl_mtx(command_mutex_id);
    }

    return err;
}

/** =================================================================*
 * @brief  最新指令状態取得
 * @param[out] p_snapshot 最新指令と経過時間
 * @return μT-Kernelエラーコード
 * ================================================================= */
ER cpu0_command_snapshot_get(cpu0_command_snapshot_t * p_snapshot) {
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
    p_snapshot->target_stale =
        command_target_age_ms >= CPU0_COMMAND_TARGET_TIMEOUT_MS;
    err = tk_unl_mtx(command_mutex_id);
    return err;
}

/** =================================================================*
 * @brief  最新指令スナップショット送信
 * @details 4サーボと左右モーターを1つのIPCフレームとして同時commitする。
 * ================================================================= */
static void cpu0_command_send_latest(void) {
    rover_motion_target_t target;
    bool target_stale;
    bool report_timeout;

    ER const lock_err = tk_loc_mtx(command_mutex_id, TMO_FEVR);
    if (E_OK != lock_err) {
        g_cpu0_command_last_error = actuator_ipc_client_emergency_stop(++g_cpu0_command_sequence);
        (void) cpu0_think_report_fault(CPU0_FAULT_TARGET_UPDATE);
        return;
    }

    if (command_target_age_ms < CPU0_COMMAND_TARGET_TIMEOUT_MS) {
        command_target_age_ms += CPU0_COMMAND_PERIOD_MS;
        if (command_target_age_ms > CPU0_COMMAND_TARGET_TIMEOUT_MS) {
            command_target_age_ms = CPU0_COMMAND_TARGET_TIMEOUT_MS;
        }
    }

    target_stale = command_target_age_ms >= CPU0_COMMAND_TARGET_TIMEOUT_MS;
    report_timeout = target_stale && !command_timeout_reported;
    if (report_timeout) {
        command_timeout_reported = true;
    }
    target = command_target;
    (void) tk_unl_mtx(command_mutex_id);

    if (report_timeout) {
        (void) cpu0_think_report_fault(CPU0_FAULT_COMMAND_TARGET_TIMEOUT);
    }
    if (target_stale) {
        target.actuator_enable = false;
        target.emergency_stop = true;
    }

    bool const clear_emergency_latch = command_emergency_reset_pending &&
                                       !target_stale &&
                                       !target.emergency_stop;

    actuator_command_t command = actuator_command_make_safe();
    command.left_target_rpm = target.left_target_rpm;
    command.right_target_rpm = target.right_target_rpm;
    command.actuator_enable = (clear_emergency_latch ? false : target.actuator_enable) ? 1U : 0U;
    command.emergency_stop = (clear_emergency_latch ? false : target.emergency_stop) ? 1U : 0U;
    for (uint32_t i = 0U; i < ACTUATOR_SERVO_COUNT; i++) {
        command.servo_target_deg[i] = target.servo_target_deg[i];
    }
    command.sequence_number = ++g_cpu0_command_sequence;

    g_cpu0_command_last_error = actuator_ipc_client_send(&command);
    if (FSP_SUCCESS != g_cpu0_command_last_error) {
        (void) actuator_ipc_client_emergency_stop(++g_cpu0_command_sequence);
        (void) cpu0_think_report_fault(CPU0_FAULT_IPC_SEND);
    } else {
        g_cpu0_command_send_count++;
        ER const sent_lock_err = tk_loc_mtx(command_mutex_id,
                                             TMO_FEVR);
        if (E_OK == sent_lock_err) {
            command_last_sent_target.left_target_rpm =
                command.left_target_rpm;
            command_last_sent_target.right_target_rpm =
                command.right_target_rpm;
            command_last_sent_target.actuator_enable =
                0U != command.actuator_enable;
            command_last_sent_target.emergency_stop =
                0U != command.emergency_stop;
            for (uint32_t i = 0U; i < ACTUATOR_SERVO_COUNT; i++) {
                command_last_sent_target.servo_target_deg[i] =
                    command.servo_target_deg[i];
            }
            (void) tk_unl_mtx(command_mutex_id);
        }
        if (clear_emergency_latch) {
            command_emergency_reset_pending = false;
        } else if (target.emergency_stop) {
            command_emergency_reset_pending = true;
        }
    }
}

/** =================================================================*
 * @brief  指令タスク本体
 * ================================================================= */
static void cpu0_command_task(INT stacd, void * exinf) {
    (void) stacd;
    (void) exinf;

    (void) tk_dly_tsk(CPU0_ACTUATOR_STARTUP_DELAY_MS);

    while (1) {
        cpu0_command_send_latest();
        (void) tk_dly_tsk(CPU0_COMMAND_PERIOD_MS);
    }
}
