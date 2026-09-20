/** =================================================================*
 * @file   acoustic_feature_assembler.c
 * @brief  音響特徴量イベント再組立
 * ================================================================= */
#include "services/acoustic_feature_assembler.h"            /* 特徴量packetと再組立状態 */
#include <string.h>                                         /* memcpy、memset */

/** =================================================================*
 * @brief  音響特徴量再組立状態初期化
 * @param[out] p_assembler 再組立状態
 * ================================================================= */
EXPORT void acoustic_feature_assembler_init(acoustic_feature_assembler_t * p_assembler) {
    if (NULL != p_assembler) {
        memset(p_assembler, 0, sizeof(*p_assembler));
    }
}

/** =================================================================*
 * @brief  特徴量packet追加
 * @details 欠落、重複、イベント混線を検出した場合は不完全パッチを破棄する。
 *          新しいイベントはframe_index 0からだけ開始できる。
 * @param[in,out] p_assembler 再組立状態
 * @param[in] p_feature 2フレーム分の特徴量packet
 * @return 再組立結果
 * ================================================================= */
EXPORT acoustic_feature_assembler_result_t acoustic_feature_assembler_push(
    acoustic_feature_assembler_t * p_assembler, const acoustic_feature_t * p_feature) {
    if ((NULL == p_assembler) || (NULL == p_feature)) {
        return CPU0_ACOUSTIC_FEATURE_FORMAT_ERROR;
    }

    if ((ACOUSTIC_FEATURE_BIN_COUNT != p_feature->n_bins) ||
        (ACOUSTIC_FEATURE_EVENT_FRAME_COUNT != p_feature->frame_count) ||
        (p_feature->frame_index >= ACOUSTIC_FEATURE_EVENT_FRAME_COUNT) ||
        (0U != (p_feature->frame_index % ACOUSTIC_FEATURE_FRAMES_PER_PACKET))) {
        p_assembler->active = FALSE;
        p_assembler->next_frame_index = 0U;
        return CPU0_ACOUSTIC_FEATURE_FORMAT_ERROR;
    }

    acoustic_feature_assembler_result_t result = CPU0_ACOUSTIC_FEATURE_MORE;
    if (0U == p_feature->frame_index) {
        if (p_assembler->active) {
            result = CPU0_ACOUSTIC_FEATURE_RESTARTED;
        }
        p_assembler->patch.event_id = p_feature->event_id;
        p_assembler->next_frame_index = 0U;
        p_assembler->active = TRUE;
    } else if (!p_assembler->active || (p_assembler->patch.event_id != p_feature->event_id) ||
               (p_assembler->next_frame_index != p_feature->frame_index)) {
        p_assembler->active = FALSE;
        p_assembler->next_frame_index = 0U;
        return CPU0_ACOUSTIC_FEATURE_SEQUENCE_ERROR;
    }

    memcpy(&p_assembler->patch.frames[p_feature->frame_index][0], p_feature->mel, ACOUSTIC_FEATURE_DATA_SIZE);
    p_assembler->next_frame_index = (UH) (p_feature->frame_index + ACOUSTIC_FEATURE_FRAMES_PER_PACKET);
    if (ACOUSTIC_FEATURE_EVENT_FRAME_COUNT == p_assembler->next_frame_index) {
        p_assembler->active = FALSE;
        return CPU0_ACOUSTIC_FEATURE_COMPLETE;
    }
    return result;
}
