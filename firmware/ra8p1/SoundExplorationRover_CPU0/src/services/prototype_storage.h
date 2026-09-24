/** =================================================================*
 * @file   prototype_storage.h
 * @brief  音響プロトタイプのCode MRAM永続化
 * ================================================================= */
#ifndef SEROV_CPU0_SERVICE_PROTOTYPE_STORAGE_H
#define SEROV_CPU0_SERVICE_PROTOTYPE_STORAGE_H

#include "services/background_model.h"                      /* 保存する背景デコーダの次元 */
#include "services/acoustic_identifier.h"                   /* 保存する96次元見本の次元 */
#include <tk/tkernel.h>                                     /* μT-Kernel基本型とリンケージ定義 */

#define CPU0_PROTOTYPE_STORAGE_SAMPLE_COUNT (CPU0_ACOUSTIC_SAMPLE_COUNT) /**< MRAMへ保存する音響見本数 */
#define CPU0_PROTOTYPE_STORAGE_SUMMARY_BYTES (CPU0_ACOUSTIC_SUMMARY_DIMENSION) /**< 要約保存要素数 */

/**< 音響プロトタイプMRAM永続化の処理結果 */
typedef enum e_prototype_storage_result {
    CPU0_PROTOTYPE_STORAGE_OK = 0U,                         /**< 永続化処理が正常完了 */
    CPU0_PROTOTYPE_STORAGE_EMPTY,                           /**< 有効な保存データが空 */
    CPU0_PROTOTYPE_STORAGE_NOT_INITIALIZED,                 /**< 保存サービス未初期化 */
    CPU0_PROTOTYPE_STORAGE_ARGUMENT_ERROR,                  /**< 引数が不正 */
    CPU0_PROTOTYPE_STORAGE_LAYOUT_ERROR,                    /**< 保存レイアウトが不正 */
    CPU0_PROTOTYPE_STORAGE_OPEN_ERROR,                      /**< MRAMオープン失敗 */
    CPU0_PROTOTYPE_STORAGE_CORE_STALL_ERROR,                /**< 相手CPU停止失敗 */
    CPU0_PROTOTYPE_STORAGE_BLANK_ERROR,                     /**< MRAM空き領域異常 */
    CPU0_PROTOTYPE_STORAGE_WRITE_ERROR,                     /**< MRAM書込み失敗 */
    CPU0_PROTOTYPE_STORAGE_VERIFY_ERROR,                    /**< MRAM読出し検証失敗 */
} prototype_storage_result_t;

/**< MRAMへ保存する背景モデルと音響見本のデータ構造 */
typedef struct st_prototype_storage_data {
    UW generation;                                          /**< 保存データの世代番号 */
    UW encoder_seed;                                        /**< 固定乱数エンコーダの再生成用seed */
    /**< MRAMへ保存する背景デコーダ行列 */
    float background_decoder[CPU0_BACKGROUND_MODEL_INPUT_DIMENSION][CPU0_BACKGROUND_MODEL_HIDDEN_DIMENSION];
    float background_mse_threshold;                         /**< 背景異常候補用mean+3sigma */
    float identifier_threshold;                             /**< 見本leave-one-out用mean+3sigma */
    UB sample_count;                                        /**< 有効な192次元見本数（最大5） */
    UB target_peak_bin;                                     /**< 見本群の代表peak bin */
    UB reserved[2];                                         /**< 将来拡張用の予約領域 */
    /**< MRAMへ保存する音響見本の要約列 */
    B samples[CPU0_PROTOTYPE_STORAGE_SAMPLE_COUNT][CPU0_PROTOTYPE_STORAGE_SUMMARY_BYTES];
} prototype_storage_data_t;

EXPORT prototype_storage_result_t prototype_storage_init(void); /* MRAMドライバと保存領域検証 */
EXPORT prototype_storage_result_t prototype_storage_load(prototype_storage_data_t * p_data); /* 最新有効値読込 */
EXPORT prototype_storage_result_t prototype_storage_clear(void); /* 学習開始時にA/B両スロットを初期化 */
EXPORT prototype_storage_result_t prototype_storage_save(prototype_storage_data_t * p_data); /* A/Bスロット保存 */

#endif /* SEROV_CPU0_SERVICE_PROTOTYPE_STORAGE_H */
