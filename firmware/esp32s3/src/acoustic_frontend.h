/** =================================================================*
 * @file   acoustic_frontend.h
 * @brief  音響観測フロントエンドAPI
 * ================================================================= */
#ifndef RESPEAKER_ACOUSTIC_FRONTEND_H
#define RESPEAKER_ACOUSTIC_FRONTEND_H

#include "esp_err.h"                                        /* ESP-IDFエラー型 */
#include <stdint.h>                                         /* 診断値の固定幅整数型 */

esp_err_t acoustic_frontend_start(void);                    /* 音響観測タスク開始 */

extern volatile uint32_t g_acoustic_frontend_read_count;    /**< XVF3800読出し回数 */
extern volatile uint32_t g_acoustic_frontend_read_error_count; /**< XVF3800読出し失敗回数 */
extern volatile uint32_t g_acoustic_frontend_tx_count;      /**< DoA観測送信成功回数 */
extern volatile uint32_t g_acoustic_frontend_tx_error_count;/**< DoA観測送信失敗回数 */
extern volatile uint32_t g_acoustic_frontend_last_period_ms;/**< 直近DoA読出し周期[ms] */
extern volatile uint32_t g_acoustic_frontend_min_period_ms; /**< 最小DoA読出し周期[ms] */
extern volatile uint32_t g_acoustic_frontend_max_period_ms; /**< 最大DoA読出し周期[ms] */
extern volatile uint32_t g_acoustic_frontend_processing_us; /**< 直近DoA処理時間[us] */
extern volatile uint32_t g_acoustic_frontend_deadline_miss_count; /**< 20 ms期限超過回数 */
extern volatile uint32_t g_acoustic_frontend_last_sample_ms; /**< 直近DoA観測時刻[ms] */
extern volatile uint32_t g_acoustic_frontend_observation_sequence; /**< 直近DoA観測sequence */
extern volatile uint16_t g_acoustic_frontend_raw_doa_deg;   /**< 直近XVF3800 DoA[deg] */
extern volatile uint16_t g_acoustic_frontend_filtered_doa_deg; /**< 循環平均DoA[deg] */
extern volatile uint8_t g_acoustic_frontend_doa_confidence; /**< DoA品質[0..100] */

#endif /* RESPEAKER_ACOUSTIC_FRONTEND_H */
