/** =================================================================*
 * @file   actuator_ipc_client.h
 * @brief  CPU0-CPU1間IPCクライアントAPI
 * ================================================================= */
#ifndef SEROV_ACTUATOR_IPC_CLIENT_H
#define SEROV_ACTUATOR_IPC_CLIENT_H

#include "../../../common/ipc_message.h"                    /* CPU間IPCメッセージ型 */
#include "hal_data.h"                                       /* FSP生成のIPCインスタンスとFSP型 */

EXPORT fsp_err_t actuator_ipc_client_init(void);                   /* IPCクライアント初期化 */
EXPORT fsp_err_t actuator_ipc_client_deinit(void);                 /* IPCクライアント終了 */
EXPORT fsp_err_t actuator_ipc_client_send(const actuator_command_t * p_command); /* アクチュエータ指令送信 */
EXPORT fsp_err_t actuator_ipc_client_emergency_stop(UW sequence_number); /* 緊急停止指令送信 */
EXPORT BOOL actuator_ipc_client_status_get(actuator_status_t * p_status); /* CPU1実出力状態取得 */
EXPORT void actuator_ipc_client_callback(ipc_callback_args_t * p_args); /* CPU1状態受信コールバック */

IMPORT volatile UW g_actuator_ipc_client_send_overflow_retry_count; /**< IPC送信overflow再試行回数（Live Watch用） */
IMPORT volatile UW g_actuator_ipc_client_last_tx_message_id;        /**< 最終IPC送信メッセージID（Live Watch用） */
IMPORT volatile fsp_err_t g_actuator_ipc_client_last_tx_error;      /**< 最終IPC送信ワードのエラー（Live Watch用） */

#endif /* SEROV_ACTUATOR_IPC_CLIENT_H */
