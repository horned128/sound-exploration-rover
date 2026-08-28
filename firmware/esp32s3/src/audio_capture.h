/** =================================================================*
 * @file   audio_capture.h
 * @brief  音声キャプチャAPI
 * ================================================================= */
#ifndef RESPEAKER_AUDIO_CAPTURE_H
#define RESPEAKER_AUDIO_CAPTURE_H

#include "esp_err.h"                                        /* ESP-IDFエラー型 */
#include <stdbool.h>                                        /* 真偽値 */
#include <stdint.h>                                         /* 固定幅整数型 */

typedef struct {
    int16_t level_dbfs_x100;                                /**< 平均音量[dBFS x100] */
    int16_t peak_dbfs_x100;                                 /**< ピーク音量[dBFS x100] */
    uint32_t frame_count;                                   /**< 累積PCMフレーム数 */
    uint32_t overrun_count;                                 /**< I2S受信オーバーラン数 */
    uint32_t captured_at_ms;                                /**< 最終取得時刻[ms] */
    bool valid;                                             /**< スナップショット有効状態 */
} audio_capture_snapshot_t;                                 /**< 音声キャプチャ状態 */

esp_err_t audio_capture_start(void);                        /* 音声キャプチャタスク開始 */
void audio_capture_get_snapshot(audio_capture_snapshot_t * snapshot); /* 最新音声キャプチャ状態取得 */

#endif /* RESPEAKER_AUDIO_CAPTURE_H */
