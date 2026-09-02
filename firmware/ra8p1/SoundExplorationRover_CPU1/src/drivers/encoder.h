/** =================================================================*
 * @file   encoder.h
 * @brief  エンコーダ取得API
 * ================================================================= */
#ifndef SEROV_ENCODER_H
#define SEROV_ENCODER_H

#include "hal_data.h"                                       /* FSP生成のHAL/BSPインスタンスとFSP型 */
#include <tk/tkernel.h>                                     /* μT-Kernel基本型と公開範囲マクロ */

EXPORT fsp_err_t encoder_init(void);                               /* 左右エンコーダ初期化 */
EXPORT void encoder_housekeeping_1ms(void);                        /* エンコーダ1 ms周期処理 */
EXPORT W encoder_left_count_get(void);                       /* 左エンコーダカウント取得 */
EXPORT W encoder_right_count_get(void);                      /* 右エンコーダカウント取得 */
EXPORT H encoder_left_rpm_get(void);                         /* 左エンコーダ回転数取得 */
EXPORT H encoder_right_rpm_get(void);                        /* 右エンコーダ回転数取得 */

EXPORT void jga25_encoder_left_a_callback(external_irq_callback_args_t * p_args); /* 左A相割込みコールバック */
EXPORT void jga25_encoder_left_b_callback(external_irq_callback_args_t * p_args); /* 左B相割込みコールバック */
EXPORT void jga25_encoder_right_a_callback(external_irq_callback_args_t * p_args); /* 右A相割込みコールバック */
EXPORT void jga25_encoder_right_b_callback(external_irq_callback_args_t * p_args); /* 右B相割込みコールバック */

IMPORT volatile W g_jga25_left_encoder_count;         /**< 左代表モーターの累積カウント（前進正） */
IMPORT volatile W g_jga25_right_encoder_count;        /**< 右代表モーターの累積カウント（前進正） */
IMPORT volatile W g_jga25_left_rpm_x10;               /**< 左代表モーター推定回転数の10倍（前進正） */
IMPORT volatile W g_jga25_right_rpm_x10;              /**< 右代表モーター推定回転数の10倍（前進正） */

#endif /* SEROV_ENCODER_H */
