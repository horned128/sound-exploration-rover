/** =================================================================*
 * @file   ipc_config.h
 * @brief  CPU0アクチュエータIPC転送の設定値
 * ================================================================= */
#ifndef SEROV_CPU0_CONFIG_IPC_H
#define SEROV_CPU0_CONFIG_IPC_H

#define CPU0_IPC_RETRY_DELAY_MS            (1U)             /**< IPC再試行の遅延[ms] */
#define CPU0_IPC_SEND_RETRY_COUNT          (20U)            /**< IPC送信再試行の個数 */

#endif /* SEROV_CPU0_CONFIG_IPC_H */
