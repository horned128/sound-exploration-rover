/** =================================================================*
 * @file   actuator_ipc_client.h
 * @brief  CPU0-CPU1間IPCクライアントAPI
 * ================================================================= */
#ifndef SEROV_ACTUATOR_IPC_CLIENT_H
#define SEROV_ACTUATOR_IPC_CLIENT_H

#include "../../../common/ipc_message.h"                    /* CPU間IPCメッセージ型 */
#include "hal_data.h"                                       /* FSP生成のIPCインスタンスとFSP型 */

fsp_err_t actuator_ipc_client_init(void);                   /* IPCクライアント初期化 */
fsp_err_t actuator_ipc_client_deinit(void);                 /* IPCクライアント終了 */
fsp_err_t actuator_ipc_client_send(const actuator_command_t * p_command); /* アクチュエータ指令送信 */
fsp_err_t actuator_ipc_client_emergency_stop(uint32_t sequence_number); /* 緊急停止指令送信 */

#endif /* SEROV_ACTUATOR_IPC_CLIENT_H */
