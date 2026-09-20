/** =================================================================*
 * @file   bts7960.h
 * @brief  BTS7960の正逆転PWMドライバAPI
 * ================================================================= */
#ifndef SEROV_CPU1_DRIVER_BTS7960_H
#define SEROV_CPU1_DRIVER_BTS7960_H

#include "hal_data.h"                                       /* FSP生成のHAL/BSPインスタンスとFSP型 */
#include <tk/tkernel.h>                                     /* μT-Kernel基本型と公開範囲マクロ */

EXPORT fsp_err_t bts7960_init(void);                        /* BTS7960 PWM資源初期化 */
/* 左右正負出力設定 */
EXPORT fsp_err_t bts7960_set_signed_duty(H left_duty_permille, H right_duty_permille); /* 左右duty設定 */
EXPORT fsp_err_t bts7960_stop(void);                        /* BTS7960出力安全停止 */

#endif /* SEROV_CPU1_DRIVER_BTS7960_H */
