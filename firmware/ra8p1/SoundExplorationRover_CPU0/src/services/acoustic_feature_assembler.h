/** =================================================================*
 * @file   acoustic_feature_assembler.h
 * @brief  音響特徴量イベント再組立
 * ================================================================= */
#ifndef SEROV_CPU0_ACOUSTIC_FEATURE_ASSEMBLER_H
#define SEROV_CPU0_ACOUSTIC_FEATURE_ASSEMBLER_H

#include "../../../../common/acoustic_protocol.h"           /* 特徴量パケット型と固定サイズ */
#include <tk/tkernel.h>                                     /* μT-Kernel基本型と公開範囲マクロ */

/**< 分割音響特徴量イベントの再組立結果 */
typedef enum e_acoustic_feature_assembler_result {
    CPU0_ACOUSTIC_FEATURE_MORE = 0,                         /**< 追加フレーム待ち */
    CPU0_ACOUSTIC_FEATURE_COMPLETE,                         /**< イベント再組立完了 */
    CPU0_ACOUSTIC_FEATURE_RESTARTED,                        /**< 新イベントで再開始 */
    CPU0_ACOUSTIC_FEATURE_FORMAT_ERROR,                     /**< パケット形式異常 */
    CPU0_ACOUSTIC_FEATURE_SEQUENCE_ERROR,                   /**< フレーム順序異常 */
} acoustic_feature_assembler_result_t;

/**< 1音響イベント分の特徴量フレームを保持するパッチ */
typedef struct st_acoustic_feature_patch {
    UH event_id;                                            /**< 再組立対象の音響イベントID */
    /**< イベントを構成する特徴量フレーム列 */
    B frames[ACOUSTIC_FEATURE_EVENT_FRAME_COUNT][ACOUSTIC_FEATURE_BIN_COUNT];
} acoustic_feature_patch_t;

/**< 分割受信した音響イベントの再組立状態 */
typedef struct st_acoustic_feature_assembler {
    acoustic_feature_patch_t patch;                         /**< 再組立中の特徴量パッチ */
    UH next_frame_index;                                    /**< 次に受信するフレーム番号 */
    BOOL active;                                            /**< 再組立中フラグ */
} acoustic_feature_assembler_t;

EXPORT void acoustic_feature_assembler_init(acoustic_feature_assembler_t * p_assembler); /* 再組立状態初期化 */
EXPORT acoustic_feature_assembler_result_t acoustic_feature_assembler_push(
    acoustic_feature_assembler_t * p_assembler, const acoustic_feature_t * p_feature); /* 特徴量packet追加 */

#endif /* SEROV_CPU0_ACOUSTIC_FEATURE_ASSEMBLER_H */
