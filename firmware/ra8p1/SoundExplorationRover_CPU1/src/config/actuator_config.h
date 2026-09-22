/** =================================================================*
 * @file   actuator_config.h
 * @brief  CPU1アクチュエータサービスの安全設定
 * ================================================================= */
#ifndef SEROV_CPU1_CONFIG_ACTUATOR_H
#define SEROV_CPU1_CONFIG_ACTUATOR_H

#define ACTUATOR_COMMAND_TIMEOUT_MS        (1500U)          /**< アクチュエータ指令の期限[ms] */
#define ACTUATOR_BOOT_CENTER_HOLD_MS       (1000U)          /**< 電源投入時に操舵0度へ整定するPWM保持時間[ms] */

#endif /* SEROV_CPU1_CONFIG_ACTUATOR_H */
