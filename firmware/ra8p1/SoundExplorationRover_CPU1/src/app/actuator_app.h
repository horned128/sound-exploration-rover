/** =================================================================*
 * @file   actuator_app.h
 * @brief  CPU1アクチュエータアプリケーションAPI
 * ================================================================= */
#ifndef SEROV_ACTUATOR_APP_H
#define SEROV_ACTUATOR_APP_H

#include "hal_data.h"                                       /* FSP生成のHAL/BSPインスタンスとFSP型 */
#include <tk/tkernel.h>                                      /* μT-Kernelの整数型、公開範囲マクロ */

EXPORT fsp_err_t actuator_app_init(void);                   /* CPU1アクチュエータアプリケーション初期化 */
EXPORT void actuator_app_run_1ms(void);                     /* CPU1アクチュエータ1 ms周期処理 */
EXPORT void actuator_app_shutdown(void);                    /* CPU1アクチュエータ安全停止 */

IMPORT volatile fsp_err_t g_actuator_last_error;            /**< 最後に発生したFSPエラー（Live Watch監視用） */
IMPORT volatile UH g_actuator_fault_flags;                  /**< アクチュエータ異常フラグ（Live Watch監視用） */

#endif /* SEROV_ACTUATOR_APP_H */
