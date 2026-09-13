/** =================================================================*
 * @file   actuator_service.h
 * @brief  CPU1アクチュエータアプリケーションAPI
 * ================================================================= */
#ifndef SEROV_CPU1_SERVICE_ACTUATOR_H
#define SEROV_CPU1_SERVICE_ACTUATOR_H

#include "hal_data.h"                                               /* FSP生成のHAL/BSPインスタンスとFSP型 */
#include "../../../common/ipc_message.h"                            /* CPU1実出力状態 */
#include "config/drive_config.h"                                    /* 計測用機能のコンパイル設定 */
#include <tk/tkernel.h>                                             /* μT-Kernelの整数型、公開範囲マクロ */

EXPORT fsp_err_t actuator_service_init(void);                       /* CPU1アクチュエータアプリケーション初期化 */
EXPORT void actuator_service_update(UW elapsed_ms);                      /* CPU1アクチュエータ実時間更新 */
EXPORT void actuator_service_shutdown(void);                        /* CPU1アクチュエータ安全停止 */
EXPORT void actuator_service_status_get(actuator_status_t * p_status); /* CPU1実出力状態取得 */

IMPORT volatile fsp_err_t g_actuator_service_last_error;            /**< 最後に発生したFSPエラー（Live Watch監視用） */
IMPORT volatile UH g_actuator_service_fault_flags;                  /**< アクチュエータ異常フラグ（Live Watch監視用） */
IMPORT volatile UW g_actuator_service_applied_sequence;             /**< 最終適用指令sequence（Live Watch監視用） */

#if DRIVE_MEASUREMENT_TEST_ENABLE
IMPORT volatile UB g_drive_measurement_mode;                        /**< 0:通常、1:デューティ計測、2:強制停止 */
IMPORT volatile UH g_drive_measurement_duty_permille;               /**< 計測デューティ入力[0.1%] */
IMPORT volatile UH g_drive_measurement_applied_duty_permille;       /**< 上限適用後の計測デューティ[0.1%] */
IMPORT volatile UB g_drive_measurement_status;                      /**< 計測モードの状態 */
IMPORT volatile BOOL g_drive_measurement_output_authorized;         /**< 通常有効指令があり出力可能ならTRUE */
IMPORT volatile W g_drive_measurement_stop_count_left;              /**< 強制停止適用直前の左累積カウント */
IMPORT volatile W g_drive_measurement_stop_count_right;             /**< 強制停止適用直前の右累積カウント */
IMPORT volatile BOOL g_drive_measurement_stop_capture_valid;        /**< 強制停止直前カウントが有効ならTRUE */
#endif

#endif /* SEROV_CPU1_SERVICE_ACTUATOR_H */
