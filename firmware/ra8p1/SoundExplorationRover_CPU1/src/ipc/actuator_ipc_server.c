/** =================================================================*
 * @file   actuator_ipc_server.c
 * @brief  CPU0-CPU1間IPCサーバー実装
 * ================================================================= */
#include "actuator_ipc_server.h"                            /* CPU1側IPCサーバーAPIとメッセージ型 */

static actuator_command_t g_staging_command;                /**< 受信中の指令 */
static actuator_command_t g_committed_command;              /**< 適用待ちの確定指令 */
static volatile bool g_command_pending;                     /**< 新しい指令の有無 */
static volatile bool g_rx_fault_pending;                    /**< IPC受信異常の有無 */

/** =================================================================*
 * @brief  IPCサーバー初期化
 * @return FSPエラーコード
 * ================================================================= */
fsp_err_t actuator_ipc_server_init(void) {
    g_staging_command = actuator_command_make_safe();
    g_committed_command = g_staging_command;
    g_command_pending = false;
    g_rx_fault_pending = false;

    return g_actuator_ipc.p_api->open(g_actuator_ipc.p_ctrl, g_actuator_ipc.p_cfg);
}

/** =================================================================*
 * @brief  IPC指令取得
 * @param[out] p_command 取得した指令の格納先
 * @return 指令が取得できた場合はtrue
 * ================================================================= */
bool actuator_ipc_server_take_command(actuator_command_t * p_command) {
    if (NULL == p_command) {
        return false;
    }

    bool pending;
    FSP_CRITICAL_SECTION_DEFINE;
    FSP_CRITICAL_SECTION_ENTER;
    pending = g_command_pending;
    if (pending) {
        *p_command = g_committed_command;
        g_command_pending = false;
    }
    FSP_CRITICAL_SECTION_EXIT;

    return pending;
}

/** =================================================================*
 * @brief  IPC受信異常取得
 * @return IPC受信異常が保留されている場合はtrue
 * ================================================================= */
bool actuator_ipc_server_take_rx_fault(void) {
    bool pending;
    FSP_CRITICAL_SECTION_DEFINE;
    FSP_CRITICAL_SECTION_ENTER;
    pending = g_rx_fault_pending;
    g_rx_fault_pending = false;
    FSP_CRITICAL_SECTION_EXIT;
    return pending;
}

/** =================================================================*
 * @brief  IPC受信コールバック
 * @param[in] p_args FSP IPCコールバック情報
 * ================================================================= */
void actuator_ipc_callback(ipc_callback_args_t * p_args) {
    if (NULL == p_args) {
        g_rx_fault_pending = true;
        return;
    }

    if (0U != (p_args->event & (IPC_EVENT_FIFO_ERROR_EMPTY | IPC_EVENT_FIFO_ERROR_FULL))) {
        g_rx_fault_pending = true;
    }

    if (0U == (p_args->event & IPC_EVENT_MESSAGE_RECEIVED)) {
        return;
    }

    uint32_t const payload = actuator_ipc_get_payload(p_args->message);

    switch (actuator_ipc_get_message_id(p_args->message)) {
        case ACTUATOR_IPC_COMMAND_CONTROL:
            g_staging_command.actuator_enable =
                (0U != (payload & ACTUATOR_CONTROL_ENABLE_MASK)) ? 1U : 0U;
            g_staging_command.emergency_stop =
                (0U != (payload & ACTUATOR_CONTROL_EMERGENCY_STOP_MASK)) ? 1U : 0U;

            /* 緊急停止はシーケンス番号を待たずに即時コミットする。 */
            if (0U != g_staging_command.emergency_stop) {
                g_committed_command = g_staging_command;
                g_command_pending = true;
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
            g_command_pending = true;
            break;

        default:
            g_rx_fault_pending = true;
            break;
    }
}
