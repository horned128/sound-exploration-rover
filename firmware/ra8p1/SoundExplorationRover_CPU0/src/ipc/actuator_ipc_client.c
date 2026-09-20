/** =================================================================*
 * @file   actuator_ipc_client.c
 * @brief  CPU0-CPU1間IPCクライアント実装
 * ================================================================= */
#include "actuator_ipc_client.h"                            /* CPU0側IPCクライアントAPIとメッセージ型 */
#include "config/ipc_config.h"                              /* CPU0のIPC再送待ち時間 */
#include <tk/tkernel.h>                                     /* μT-Kernelのタスク遅延API */

/**< CPU1へ送るサーボ目標角メッセージID */
LOCAL actuator_ipc_message_id_t const servo_target_message_ids[ACTUATOR_SERVO_COUNT] = {
    ACTUATOR_IPC_COMMAND_FR_TARGET_DEG,
    ACTUATOR_IPC_COMMAND_FL_TARGET_DEG,
    ACTUATOR_IPC_COMMAND_RR_TARGET_DEG,
    ACTUATOR_IPC_COMMAND_RL_TARGET_DEG,
};

LOCAL actuator_status_t g_staging_status;                   /**< 受信中のCPU1状態 */
LOCAL actuator_status_t g_committed_status;                 /**< 確定したCPU1状態 */
LOCAL volatile BOOL g_status_valid;                         /**< CPU1状態受信済み */

/**< IPC送信overflow再試行回数 */
EXPORT volatile UW g_actuator_ipc_client_send_overflow_retry_count;
EXPORT volatile UW g_actuator_ipc_client_last_tx_message_id;/**< 最終IPC送信メッセージID */
/**< 最終IPC送信ワードのエラー */
EXPORT volatile fsp_err_t g_actuator_ipc_client_last_tx_error;

/** =================================================================*
 * @brief  IPCワード送信
 * @param[in] word 送信する32 bitワード
 * @return FSPエラーコード
 * ================================================================= */
LOCAL fsp_err_t actuator_ipc_send_word(UW word) {
    fsp_err_t err = FSP_SUCCESS;

    g_actuator_ipc_client_last_tx_message_id = (UW) actuator_ipc_get_message_id(word);
    for (UW retry_count = 0U; retry_count <= CPU0_IPC_SEND_RETRY_COUNT; retry_count++) {
        err = g_actuator_ipc.p_api->messageSend(g_actuator_ipc.p_ctrl, word);
        if (FSP_ERR_OVERFLOW != err) {
            break;
        }
        if (retry_count < CPU0_IPC_SEND_RETRY_COUNT) {
            g_actuator_ipc_client_send_overflow_retry_count++;
            (void) tk_dly_tsk(CPU0_IPC_RETRY_DELAY_MS);
        }
    }

    g_actuator_ipc_client_last_tx_error = err;
    return err;
}

/** =================================================================*
 * @brief  IPCクライアント初期化
 * @return FSPエラーコード
 * ================================================================= */
EXPORT fsp_err_t actuator_ipc_client_init(void) {
    g_staging_status = (actuator_status_t){0};
    g_committed_status = g_staging_status;
    g_status_valid = FALSE;
    g_actuator_ipc_client_send_overflow_retry_count = 0U;
    g_actuator_ipc_client_last_tx_message_id = 0U;
    g_actuator_ipc_client_last_tx_error = FSP_SUCCESS;
    return g_actuator_ipc.p_api->open(g_actuator_ipc.p_ctrl, g_actuator_ipc.p_cfg);
}

/** =================================================================*
 * @brief  IPCクライアント終了
 * @return FSPエラーコード
 * ================================================================= */
EXPORT fsp_err_t actuator_ipc_client_deinit(void) {
    return g_actuator_ipc.p_api->close(g_actuator_ipc.p_ctrl);
}

/** =================================================================*
 * @brief  アクチュエータ指令送信
 * @param[in] p_command CPU1へ送信する指令
 * @return FSPエラーコード
 * ================================================================= */
EXPORT fsp_err_t actuator_ipc_client_send(const actuator_command_t * p_command) {
    if (NULL == p_command) {
        return FSP_ERR_INVALID_POINTER;
    }

    /* 緊急停止を先に送り、スナップショット確定前にCPU1が停止できるようにする。 */
    fsp_err_t err = actuator_ipc_send_word(
        actuator_ipc_make_control_word(0U != p_command->actuator_enable, 0U != p_command->emergency_stop));

    if (FSP_SUCCESS == err) {
        err = actuator_ipc_send_word(
            actuator_ipc_make_i16_word(ACTUATOR_IPC_COMMAND_LEFT_TARGET_RPM, p_command->left_target_rpm));
    }
    if (FSP_SUCCESS == err) {
        err = actuator_ipc_send_word(
            actuator_ipc_make_i16_word(ACTUATOR_IPC_COMMAND_RIGHT_TARGET_RPM, p_command->right_target_rpm));
    }
    for (UW i = 0U; (FSP_SUCCESS == err) && (i < ACTUATOR_SERVO_COUNT); i++) {
        err = actuator_ipc_send_word(
            actuator_ipc_make_i16_word(servo_target_message_ids[i], p_command->servo_target_deg[i]));
    }
    if (FSP_SUCCESS == err) {
        /* シーケンス番号を完全な指令スナップショットのコミットマーカーとする。 */
        err = actuator_ipc_send_word(actuator_ipc_make_sequence_word(p_command->sequence_number));
    }

    return err;
}

/** =================================================================*
 * @brief  緊急停止指令送信
 * @param[in] sequence_number 緊急停止指令のシーケンス番号
 * @return FSPエラーコード
 * ================================================================= */
EXPORT fsp_err_t actuator_ipc_client_emergency_stop(UW sequence_number) {
    fsp_err_t err = actuator_ipc_send_word(actuator_ipc_make_control_word(FALSE, TRUE));
    if (FSP_SUCCESS == err) {
        err = actuator_ipc_send_word(actuator_ipc_make_sequence_word(sequence_number));
    }

    return err;
}

/** =================================================================*
 * @brief  CPU1実出力状態取得
 * @param[out] p_status 最新状態の格納先
 * @return CPU1から完全な状態を受信済みならtrue
 * ================================================================= */
EXPORT BOOL actuator_ipc_client_status_get(actuator_status_t * p_status) {
    if (NULL == p_status) {
        return FALSE;
    }

    BOOL valid;
    FSP_CRITICAL_SECTION_DEFINE;
    FSP_CRITICAL_SECTION_ENTER;
    valid = g_status_valid;
    if (valid) {
        *p_status = g_committed_status;
    }
    FSP_CRITICAL_SECTION_EXIT;
    return valid;
}

/** =================================================================*
 * @brief  CPU1状態受信コールバック
 * @param[in] p_args FSP IPCコールバック情報
 * ================================================================= */
EXPORT void actuator_ipc_client_callback(ipc_callback_args_t * p_args) {
    if ((NULL == p_args) || (0U == (p_args->event & IPC_EVENT_MESSAGE_RECEIVED))) {
        return;
    }

    UW const payload = actuator_ipc_get_payload(p_args->message);
    switch (actuator_ipc_get_message_id(p_args->message)) {
    case ACTUATOR_IPC_STATUS_FAULT_FLAGS:
        g_staging_status.fault_flags = (UH) payload;
        break;
    case ACTUATOR_IPC_STATUS_LEFT_DUTY:
        g_staging_status.left_duty_permille = actuator_ipc_get_i16_payload(p_args->message);
        break;
    case ACTUATOR_IPC_STATUS_RIGHT_DUTY:
        g_staging_status.right_duty_permille = actuator_ipc_get_i16_payload(p_args->message);
        break;
    case ACTUATOR_IPC_STATUS_LEFT_ENCODER_RPM_X10:
        g_staging_status.left_encoder_rpm_x10 = actuator_ipc_get_i16_payload(p_args->message);
        break;
    case ACTUATOR_IPC_STATUS_RIGHT_ENCODER_RPM_X10:
        g_staging_status.right_encoder_rpm_x10 = actuator_ipc_get_i16_payload(p_args->message);
        break;
    case ACTUATOR_IPC_STATUS_APPLIED_SEQUENCE:
        g_staging_status.applied_command_sequence = payload & ACTUATOR_IPC_SEQUENCE_MASK;
        break;
    case ACTUATOR_IPC_STATUS_LEFT_ENCODER_COUNT:
        g_staging_status.left_encoder_count = payload;
        break;
    case ACTUATOR_IPC_STATUS_RIGHT_ENCODER_COUNT:
        g_staging_status.right_encoder_count = payload;
        break;
    case ACTUATOR_IPC_STATUS_UPTIME_MS:
        g_staging_status.status_uptime_ms = payload;
        break;
    case ACTUATOR_IPC_STATUS_SEQUENCE:
        g_staging_status.sequence_number = payload & ACTUATOR_IPC_SEQUENCE_MASK;
        g_committed_status = g_staging_status;
        g_status_valid = TRUE;
        break;
    default:
        break;
    }
}
