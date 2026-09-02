/** =================================================================*
 * @file   actuator_ipc_client.c
 * @brief  CPU0-CPU1間IPCクライアント実装
 * ================================================================= */
#include "actuator_ipc_client.h"                            /* CPU0側IPCクライアントAPIとメッセージ型 */
#include "../cpu0_config.h"                                 /* CPU0のIPC再送待ち時間 */
#include <tk/tkernel.h>                                     /* μT-Kernelのタスク遅延API */

/**< CPU1へ送るサーボ目標角メッセージID */
LOCAL actuator_ipc_message_id_t const servo_target_message_ids[ACTUATOR_SERVO_COUNT] = {
    ACTUATOR_IPC_COMMAND_FR_TARGET_DEG,
    ACTUATOR_IPC_COMMAND_FL_TARGET_DEG,
    ACTUATOR_IPC_COMMAND_RR_TARGET_DEG,
    ACTUATOR_IPC_COMMAND_RL_TARGET_DEG,
};

/** =================================================================*
 * @brief  IPCワード送信
 * @param[in] word 送信する32 bitワード
 * @return FSPエラーコード
 * ================================================================= */
LOCAL fsp_err_t actuator_ipc_send_word(UW word) {
    fsp_err_t err;

    do {
        err = g_actuator_ipc.p_api->messageSend(g_actuator_ipc.p_ctrl, word);
        if (FSP_ERR_OVERFLOW == err) {
            tk_dly_tsk(CPU0_IPC_RETRY_DELAY_MS);
        }
    } while (FSP_ERR_OVERFLOW == err);

    return err;
}

/** =================================================================*
 * @brief  IPCクライアント初期化
 * @return FSPエラーコード
 * ================================================================= */
EXPORT fsp_err_t actuator_ipc_client_init(void) {
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
