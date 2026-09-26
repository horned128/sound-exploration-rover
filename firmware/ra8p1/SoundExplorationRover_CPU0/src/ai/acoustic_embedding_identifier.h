/** =================================================================*
 * @file   acoustic_embedding_identifier.h
 * @brief  TFLM音響埋め込みCNNによる現場学習照合サービス
 * ================================================================= */
#ifndef SEROV_CPU0_ACOUSTIC_EMBEDDING_IDENTIFIER_H
#define SEROV_CPU0_ACOUSTIC_EMBEDDING_IDENTIFIER_H

#include <tk/tkernel.h>                                     /* μT-Kernel基本型定義 */
#include "services/acoustic_identifier.h"                   /* 音響識別出力型定義 */
#include "ai/acoustic_embedding_model.h"                   /* 音響埋め込みモデル定数 */

#ifdef __cplusplus
extern "C" {
#endif

EXPORT BOOL acoustic_embedding_identifier_init(void);       /* 音響埋め込み推論器初期化 */
EXPORT BOOL acoustic_embedding_identifier_extract(const B * p_mel_80x32, float * p_embedding_64d); /* 特徴量埋め込み抽出 */
/* 抽出埋め込みと現場見本の最近傍コサイン距離照合 */
EXPORT void acoustic_embedding_identifier_classify(
    const float * p_query_64d,
    const float * p_prototypes_64d,
    UW sample_count,
    float threshold,
    acoustic_identifier_summary_output_t * p_output);

#ifdef __cplusplus
}
#endif

#endif /* SEROV_CPU0_ACOUSTIC_EMBEDDING_IDENTIFIER_H */
