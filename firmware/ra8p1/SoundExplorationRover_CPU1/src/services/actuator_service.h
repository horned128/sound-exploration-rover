/** =================================================================*
 * @file   actuator_service.h
 * @brief  CPU1アクチュエータアプリケーションAPI
 * ================================================================= */
#ifndef SEROV_CPU1_SERVICE_ACTUATOR_H
#define SEROV_CPU1_SERVICE_ACTUATOR_H

#include "hal_data.h"                                       /* FSP生成のHAL/BSPインスタンスとFSP型 */
#include "../../../common/ipc_message.h"                    /* CPU1実出力状態 */
#include "config/drive_config.h"                            /* 計測用機能のコンパイル設定 */
#include <tk/tkernel.h>                                     /* μT-Kernelの整数型、公開範囲マクロ */

EXPORT fsp_err_t actuator_service_init(void);               /* CPU1アクチュエータ初期化 */
EXPORT void actuator_service_update(UW elapsed_ms);         /* CPU1アクチュエータ更新 */
EXPORT void actuator_service_shutdown(void);                /* CPU1アクチュエータ安全停止 */
EXPORT void actuator_service_status_get(actuator_status_t * p_status); /* CPU1実出力状態取得 */

IMPORT volatile fsp_err_t g_actuator_service_last_error;    /**< 最終FSPエラー（Live Watch用） */
IMPORT volatile UH g_actuator_service_fault_flags;          /**< アクチュエータ異常フラグ */
IMPORT volatile UW g_actuator_service_applied_sequence;     /**< 最終適用sequence */

#if DRIVE_MEASUREMENT_TEST_ENABLE
IMPORT volatile UB g_drive_measurement_mode;                /**< 0通常/1 duty計測/2強制停止 */
IMPORT volatile UH g_drive_measurement_duty_permille;       /**< 計測デューティ入力[0.1%] */
/**< 上限適用後duty[0.1%] */
IMPORT volatile UH g_drive_measurement_applied_duty_permille;
IMPORT volatile UB g_drive_measurement_status;              /**< 計測モードの状態 */
IMPORT volatile BOOL g_drive_measurement_output_authorized; /**< 通常出力を許可できる状態 */
IMPORT volatile W g_drive_measurement_stop_count_left;      /**< 強制停止直前の左累積count */
IMPORT volatile W g_drive_measurement_stop_count_right;     /**< 強制停止直前の右累積count */
IMPORT volatile BOOL g_drive_measurement_stop_capture_valid;/**< 停止直前countの有効状態 */
#endif

#endif /* SEROV_CPU1_SERVICE_ACTUATOR_H */
