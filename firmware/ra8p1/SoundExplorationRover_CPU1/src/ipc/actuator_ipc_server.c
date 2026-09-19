/** =================================================================*
 * @file   actuator_ipc_server.c
 * @brief  CPU0-CPU1間IPCサーバー実装
 * ================================================================= */
#include "actuator_ipc_server.h"                            /* CPU1側IPCサーバーAPIとメッセージ型 */

LOCAL actuator_command_t g_staging_command;                /**< 受信中の指令 */
LOCAL actuator_command_t g_committed_command;              /**< 適用待ちの確定指令 */
LOCAL volatile BOOL g_command_pending;                     /**< 新しい指令の有無 */
LOCAL volatile BOOL g_rx_fault_pending;                    /**< IPC受信異常の有無 */

/** =================================================================*
 * @brief  CPU1状態IPCワード送信
 * @param[in] word 送信する32 bitワード
 * @return FSPエラーコード
 * ================================================================= */
LOCAL fsp_err_t actuator_ipc_server_send_word(UW word) {
    return g_actuator_ipc.p_api->messageSend(g_actuator_ipc.p_ctrl, word);
}

/** =================================================================*
 * @brief  IPCサーバー初期化
 * @return FSPエラーコード
 * ================================================================= */
EXPORT fsp_err_t actuator_ipc_server_init(void) {
    g_staging_command = actuator_command_make_safe();
    g_committed_command = g_staging_command;
    g_command_pending = FALSE;
    g_rx_fault_pending = FALSE;

    return g_actuator_ipc.p_api->open(g_actuator_ipc.p_ctrl, g_actuator_ipc.p_cfg);
}

/** =================================================================*
 * @brief  IPC指令取得
 * @param[out] p_command 取得した指令の格納先
 * @return 指令が取得できた場合はtrue
 * ================================================================= */
EXPORT BOOL actuator_ipc_server_take_command(actuator_command_t * p_command) {
    if (NULL == p_command) {
        return FALSE;
    }

    BOOL pending;
    FSP_CRITICAL_SECTION_DEFINE;
    FSP_CRITICAL_SECTION_ENTER;
    pending = g_command_pending;
    if (pending) {
        *p_command = g_committed_command;
        g_command_pending = FALSE;
    }
    FSP_CRITICAL_SECTION_EXIT;

    return pending;
}

/** =================================================================*
 * @brief  IPC受信異常取得
 * @return IPC受信異常が保留されている場合はtrue
 * ================================================================= */
EXPORT BOOL actuator_ipc_server_take_rx_fault(void) {
    BOOL pending;
    FSP_CRITICAL_SECTION_DEFINE;
    FSP_CRITICAL_SECTION_ENTER;
    pending = g_rx_fault_pending;
    g_rx_fault_pending = FALSE;
    FSP_CRITICAL_SECTION_EXIT;
    return pending;
}

/** =================================================================*
 * @brief  CPU1実出力状態1ワード送信
 * @details 4段FIFOをあふれさせないよう、状態タスクから1ワードずつ呼び出す。
 * @param[in] p_status 送信するCPU1実出力状態
 * @param[in] word_index 状態内の送信ワード番号
 * @return FSPエラーコード
 * ================================================================= */
EXPORT fsp_err_t actuator_ipc_server_send_status_word(const actuator_status_t * p_status, UB word_index) {
    if (NULL == p_status) {
        return FSP_ERR_INVALID_POINTER;
    }

    switch (word_index) {
    case 0U:
        return actuator_ipc_server_send_word(
            actuator_ipc_make_word(ACTUATOR_IPC_STATUS_FAULT_FLAGS, p_status->fault_flags));
    case 1U:
        return actuator_ipc_server_send_word(
            actuator_ipc_make_i16_word(ACTUATOR_IPC_STATUS_LEFT_DUTY, p_status->left_duty_permille));
    case 2U:
        return actuator_ipc_server_send_word(
            actuator_ipc_make_i16_word(ACTUATOR_IPC_STATUS_RIGHT_DUTY, p_status->right_duty_permille));
    case 3U:
        return actuator_ipc_server_send_word(
            actuator_ipc_make_i16_word(ACTUATOR_IPC_STATUS_LEFT_ENCODER_RPM_X10, p_status->left_encoder_rpm_x10));
    case 4U:
        return actuator_ipc_server_send_word(
            actuator_ipc_make_i16_word(ACTUATOR_IPC_STATUS_RIGHT_ENCODER_RPM_X10, p_status->right_encoder_rpm_x10));
    case 5U:
        return actuator_ipc_server_send_word(actuator_ipc_make_word(
            ACTUATOR_IPC_STATUS_APPLIED_SEQUENCE, p_status->applied_command_sequence & ACTUATOR_IPC_SEQUENCE_MASK));
    case 6U:
        return actuator_ipc_server_send_word(actuator_ipc_make_word(
            ACTUATOR_IPC_STATUS_LEFT_ENCODER_COUNT, p_status->left_encoder_count & ACTUATOR_IPC_PAYLOAD_MASK));
    case 7U:
        return actuator_ipc_server_send_word(actuator_ipc_make_word(
            ACTUATOR_IPC_STATUS_RIGHT_ENCODER_COUNT, p_status->right_encoder_count & ACTUATOR_IPC_PAYLOAD_MASK));
    case 8U:
        return actuator_ipc_server_send_word(actuator_ipc_make_word(
            ACTUATOR_IPC_STATUS_UPTIME_MS, p_status->status_uptime_ms & ACTUATOR_IPC_PAYLOAD_MASK));
    case 9U:
        /* 最後のsequenceワードでCPU0側のスナップショットを確定する。 */
        return actuator_ipc_server_send_word(actuator_ipc_make_status_sequence_word(p_status->sequence_number));
    default:
        return FSP_ERR_INVALID_ARGUMENT;
    }
}

/** =================================================================*
 * @brief  IPC受信コールバック
 * @param[in] p_args FSP IPCコールバック情報
 * ================================================================= */
EXPORT void actuator_ipc_callback(ipc_callback_args_t * p_args) {
    if (NULL == p_args) {
        g_rx_fault_pending = TRUE;
        return;
    }

    if (0U != (p_args->event & (IPC_EVENT_FIFO_ERROR_EMPTY | IPC_EVENT_FIFO_ERROR_FULL))) {
        g_rx_fault_pending = TRUE;
    }

    if (0U == (p_args->event & IPC_EVENT_MESSAGE_RECEIVED)) {
        return;
    }

    UW const payload = actuator_ipc_get_payload(p_args->message);

    switch (actuator_ipc_get_message_id(p_args->message)) {
    case ACTUATOR_IPC_COMMAND_CONTROL:
        g_staging_command.actuator_enable = (0U != (payload & ACTUATOR_CONTROL_ENABLE_MASK)) ? 1U : 0U;
        g_staging_command.emergency_stop = (0U != (payload & ACTUATOR_CONTROL_EMERGENCY_STOP_MASK)) ? 1U : 0U;

        /* 緊急停止はシーケンス番号を待たずに即時コミットする。 */
        if (0U != g_staging_command.emergency_stop) {
            g_committed_command = g_staging_command;
            g_command_pending = TRUE;
        }
        break;

    case ACTUATOR_IPC_COMMAND_LEFT_TARGET_RPM:
        g_staging_command.left_target_rpm = actuator_ipc_get_i16_payload(p_args->message);
        break;

    case ACTUATOR_IPC_COMMAND_RIGHT_TARGET_RPM:
        g_staging_command.right_target_rpm = actuator_ipc_get_i16_payload(p_args->message);
        break;

    case ACTUATOR_IPC_COMMAND_FR_TARGET_DEG:
        g_staging_command.servo_target_deg[0] = actuator_ipc_get_i16_payload(p_args->message);
        break;

    case ACTUATOR_IPC_COMMAND_FL_TARGET_DEG:
        g_staging_command.servo_target_deg[1] = actuator_ipc_get_i16_payload(p_args->message);
        break;

    case ACTUATOR_IPC_COMMAND_RR_TARGET_DEG:
        g_staging_command.servo_target_deg[2] = actuator_ipc_get_i16_payload(p_args->message);
        break;

    case ACTUATOR_IPC_COMMAND_RL_TARGET_DEG:
        g_staging_command.servo_target_deg[3] = actuator_ipc_get_i16_payload(p_args->message);
        break;

    case ACTUATOR_IPC_COMMAND_SEQUENCE:
        g_staging_command.sequence_number = payload & ACTUATOR_IPC_SEQUENCE_MASK;
        g_committed_command = g_staging_command;
        g_command_pending = TRUE;
        break;

    default:
        g_rx_fault_pending = TRUE;
        break;
    }
}
