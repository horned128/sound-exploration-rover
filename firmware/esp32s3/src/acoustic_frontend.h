/** =================================================================*
 * @file   acoustic_frontend.h
 * @brief  音響観測フロントエンドAPI
 * ================================================================= */
#ifndef RESPEAKER_ACOUSTIC_FRONTEND_H
#define RESPEAKER_ACOUSTIC_FRONTEND_H

#include "esp_err.h"                                        /* ESP-IDFエラー型 */

esp_err_t acoustic_frontend_start(void);                    /* 音響観測タスク開始 */

#endif /* RESPEAKER_ACOUSTIC_FRONTEND_H */
