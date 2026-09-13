/** =================================================================*
 * @file   acoustic_feature_assembler.h
 * @brief  音響特徴量イベント再組立
 * ================================================================= */
#ifndef SEROV_CPU0_ACOUSTIC_FEATURE_ASSEMBLER_H
#define SEROV_CPU0_ACOUSTIC_FEATURE_ASSEMBLER_H

#include "../../../../common/acoustic_protocol.h"           /* 特徴量パケット型と固定サイズ */
#include <tk/tkernel.h>                                     /* μT-Kernel基本型と公開範囲マクロ */

typedef enum e_acoustic_feature_assembler_result {
    CPU0_ACOUSTIC_FEATURE_MORE = 0,
    CPU0_ACOUSTIC_FEATURE_COMPLETE,
    CPU0_ACOUSTIC_FEATURE_RESTARTED,
    CPU0_ACOUSTIC_FEATURE_FORMAT_ERROR,
    CPU0_ACOUSTIC_FEATURE_SEQUENCE_ERROR,
} acoustic_feature_assembler_result_t;

typedef struct st_acoustic_feature_patch {
    UH event_id;
    B frames[ACOUSTIC_FEATURE_EVENT_FRAME_COUNT][ACOUSTIC_FEATURE_BIN_COUNT];
} acoustic_feature_patch_t;

typedef struct st_acoustic_feature_assembler {
    acoustic_feature_patch_t patch;
    UH next_frame_index;
    BOOL active;
} acoustic_feature_assembler_t;

EXPORT void acoustic_feature_assembler_init(acoustic_feature_assembler_t * p_assembler); /* 再組立状態初期化 */
/* 特徴量packet追加 */
EXPORT acoustic_feature_assembler_result_t acoustic_feature_assembler_push(
    acoustic_feature_assembler_t * p_assembler, const acoustic_feature_t * p_feature);

#endif /* SEROV_CPU0_ACOUSTIC_FEATURE_ASSEMBLER_H */
