/** =================================================================*
 * @file   prototype_storage.h
 * @brief  音響プロトタイプのCode MRAM永続化
 * ================================================================= */
#ifndef SEROV_CPU0_SERVICE_PROTOTYPE_STORAGE_H
#define SEROV_CPU0_SERVICE_PROTOTYPE_STORAGE_H

#include "services/background_model.h"                     /* 保存する背景デコーダの次元 */
#include "services/acoustic_identifier.h"                  /* 保存する96次元見本の次元 */
#include <tk/tkernel.h>                                     /* μT-Kernel基本型とリンケージ定義 */

#define CPU0_PROTOTYPE_STORAGE_SAMPLE_COUNT       (CPU0_ACOUSTIC_SAMPLE_COUNT)
#define CPU0_PROTOTYPE_STORAGE_SUMMARY_BYTES      (CPU0_ACOUSTIC_SUMMARY_DIMENSION)

typedef enum e_prototype_storage_result {
    CPU0_PROTOTYPE_STORAGE_OK = 0U,
    CPU0_PROTOTYPE_STORAGE_EMPTY,
    CPU0_PROTOTYPE_STORAGE_NOT_INITIALIZED,
    CPU0_PROTOTYPE_STORAGE_ARGUMENT_ERROR,
    CPU0_PROTOTYPE_STORAGE_LAYOUT_ERROR,
    CPU0_PROTOTYPE_STORAGE_OPEN_ERROR,
    CPU0_PROTOTYPE_STORAGE_CORE_STALL_ERROR,
    CPU0_PROTOTYPE_STORAGE_BLANK_ERROR,
    CPU0_PROTOTYPE_STORAGE_WRITE_ERROR,
    CPU0_PROTOTYPE_STORAGE_VERIFY_ERROR,
} prototype_storage_result_t;

typedef struct st_prototype_storage_data {
    UW generation;
    UW encoder_seed;                                        /**< 固定乱数エンコーダの再生成用seed */
    float background_decoder[CPU0_BACKGROUND_MODEL_INPUT_DIMENSION][CPU0_BACKGROUND_MODEL_HIDDEN_DIMENSION];
    float background_mse_threshold;                         /**< 背景異常候補用mean+3sigma */
    float identifier_threshold;                             /**< 見本leave-one-out用mean+3sigma */
    UB sample_count;                                        /**< 有効な192次元見本数（最大5） */
    UB target_peak_bin;                                     /**< 見本群から算出した代表ピークbin（0..31） */
    UB reserved[2];
    B samples[CPU0_PROTOTYPE_STORAGE_SAMPLE_COUNT][CPU0_PROTOTYPE_STORAGE_SUMMARY_BYTES];
} prototype_storage_data_t;

EXPORT prototype_storage_result_t prototype_storage_init(void); /* MRAMドライバと保存領域検証 */
EXPORT prototype_storage_result_t prototype_storage_load(prototype_storage_data_t * p_data); /* 最新有効値読込 */
EXPORT prototype_storage_result_t prototype_storage_save(prototype_storage_data_t * p_data); /* A/Bスロット保存 */

#endif /* SEROV_CPU0_SERVICE_PROTOTYPE_STORAGE_H */
