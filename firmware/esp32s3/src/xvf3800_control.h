/** =================================================================*
 * @file   xvf3800_control.h
 * @brief  XVF3800制御API
 * ================================================================= */
#ifndef RESPEAKER_XVF3800_CONTROL_H
#define RESPEAKER_XVF3800_CONTROL_H

#include "esp_err.h"                                        /* ESP-IDFエラー型 */
#include <stdbool.h>                                        /* 真偽値 */
#include <stdint.h>                                         /* 固定幅整数型 */

typedef struct {
    uint16_t doa_deg;                                       /**< 到来方向[度] */
    uint16_t speech_detected_raw;                           /**< XVF音声検出生値 */
    uint8_t raw_status;                                     /**< GPO Servicer応答状態 */
    bool doa_valid;                                         /**< 到来方向有効状態 */
    bool used_aec_fallback;                                 /**< AECフォールバック使用状態 */
} xvf3800_doa_result_t;                                     /**< XVF3800到来方向取得結果 */

esp_err_t xvf3800_control_init(void);                       /* I2C制御初期化 */
esp_err_t xvf3800_control_read_doa(xvf3800_doa_result_t * result); /* 到来方向取得 */

#endif /* RESPEAKER_XVF3800_CONTROL_H */
