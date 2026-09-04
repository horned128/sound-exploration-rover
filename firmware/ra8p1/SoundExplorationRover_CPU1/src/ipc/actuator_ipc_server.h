/** =================================================================*
 * @file   actuator_ipc_server.h
 * @brief  CPU0-CPU1間IPCサーバーAPI
 * ================================================================= */
#ifndef SEROV_ACTUATOR_IPC_SERVER_H
#define SEROV_ACTUATOR_IPC_SERVER_H

#include "../../../common/ipc_message.h"                    /* CPU間IPCメッセージ型 */
#include "hal_data.h"                                       /* FSP生成のIPCインスタンスとFSP型 */

EXPORT fsp_err_t actuator_ipc_server_init(void);                   /* IPCサーバー初期化 */
EXPORT BOOL actuator_ipc_server_take_command(actuator_command_t * p_command); /* IPC指令取得 */
EXPORT BOOL actuator_ipc_server_take_rx_fault(void);               /* IPC受信異常取得 */
EXPORT fsp_err_t actuator_ipc_server_send_status_word(const actuator_status_t * p_status,
                                                       UB word_index); /* CPU1実出力状態1ワード送信 */

EXPORT void actuator_ipc_callback(ipc_callback_args_t * p_args);   /* IPC受信コールバック */

#endif /* SEROV_ACTUATOR_IPC_SERVER_H */
