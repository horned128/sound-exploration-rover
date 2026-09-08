/** =================================================================*
 * @file   drive_service.h
 * @brief  車体側RPM、校正、変化率制限のサービスAPI
 * ================================================================= */
#ifndef SEROV_CPU1_SERVICE_DRIVE_H
#define SEROV_CPU1_SERVICE_DRIVE_H

#include "hal_data.h"                                       /* FSP生成のHAL/BSPインスタンスとFSP型 */
#include <tk/tkernel.h>                                     /* μT-Kernel基本型と公開範囲マクロ */

EXPORT fsp_err_t drive_service_init(void);                   /* 車体駆動サービス初期化 */
EXPORT fsp_err_t drive_service_set_target_rpm(H left_rpm, H right_rpm); /* 左右目標回転数設定 */
EXPORT fsp_err_t drive_service_update_1ms(void);             /* 車体駆動1 ms周期更新 */
EXPORT fsp_err_t drive_service_stop(void);                   /* 車体駆動サービス安全停止 */
EXPORT H drive_service_left_target_rpm_get(void);            /* 左モーター目標回転数取得 */
EXPORT H drive_service_right_target_rpm_get(void);           /* 右モーター目標回転数取得 */

IMPORT volatile H g_drive_left_duty_permille;                /**< 左モーターの現在デューティ[0.1%] */
IMPORT volatile H g_drive_right_duty_permille;               /**< 右モーターの現在デューティ[0.1%] */

#endif /* SEROV_CPU1_SERVICE_DRIVE_H */
