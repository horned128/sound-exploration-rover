/** =================================================================*
 * @file   acoustic_embedding_identifier.c
 * @brief  TFLM音響埋め込みCNNによる現場学習照合サービス実装
 * ================================================================= */
#include "config/task_config.h"                             /* タスク優先度・TFLM有効化設定 */
#include "ai/acoustic_embedding_identifier.h"              /* 音響埋め込み照合API */
#if (CPU0_USE_ACOUSTIC_EMBEDDING_TFLM != 0U)
#include "ai/acoustic_tflm_runtime.h"                      /* TFLMランタイムAPI */
#include <string.h>                                         /* メモリ操作関数 */
#include <math.h>                                           /* sqrtf等の数学関数 */

LOCAL BOOL s_initialized = FALSE;                           /**< TFLMランタイム初期化状態 */

/** =================================================================*
 * @brief  音響埋め込み推論器初期化
 * @return 初期化成功ならTRUE、失敗ならFALSE
 * ================================================================= */
EXPORT BOOL acoustic_embedding_identifier_init(void) {
    if (s_initialized) {
        return TRUE;
    }
    INT const err = acoustic_tflm_runtime_init(g_acoustic_embedding_model, ACOUSTIC_EMBEDDING_MODEL_BYTES);
    if (ACOUSTIC_TFLM_OK == err) {
        s_initialized = TRUE;
        return TRUE;
    }
    return FALSE;
}

/** =================================================================*
 * @brief  特徴量フレームからの64次元音響埋め込み抽出
 * @param[in]  p_mel_80x32 80フレーム×32ビンlog-mel入力配列
 * @param[out] p_embedding_64d 抽出された64次元正規化埋め込みベクトル
 * @return 抽出成功ならTRUE、失敗ならFALSE
 * ================================================================= */
EXPORT BOOL acoustic_embedding_identifier_extract(const B * p_mel_80x32, float * p_embedding_64d) {
    if ((NULL == p_mel_80x32) || (NULL == p_embedding_64d)) {
        return FALSE;
    }
    if (!s_initialized) {
        if (!acoustic_embedding_identifier_init()) {
            return FALSE;
        }
    }
    UW const input_bytes = ACOUSTIC_EMBEDDING_INPUT_FRAMES * ACOUSTIC_EMBEDDING_INPUT_BINS;
    INT const err = acoustic_tflm_runtime_invoke(p_mel_80x32, input_bytes,
                                                 p_embedding_64d, ACOUSTIC_EMBEDDING_DIMENSION);
    if (ACOUSTIC_TFLM_OK != err) {
        return FALSE;
    }

    /* 出力のL2ノルム正規化（防衛的） */
    float sum_sq = 0.0f;
    for (UW i = 0U; i < ACOUSTIC_EMBEDDING_DIMENSION; i++) {
        sum_sq += p_embedding_64d[i] * p_embedding_64d[i];
    }
    if (sum_sq > 1.0e-8f) {
        float const inv_norm = 1.0f / sqrtf(sum_sq);
        for (UW i = 0U; i < ACOUSTIC_EMBEDDING_DIMENSION; i++) {
            p_embedding_64d[i] *= inv_norm;
        }
    }
    return TRUE;
}

/** =================================================================*
 * @brief  抽出埋め込みと現場見本の最近傍コサイン距離照合
 * @param[in]  p_query_64d 判定対象の64次元埋め込みベクトル
 * @param[in]  p_prototypes_64d 登録見本埋め込みベクトル配列
 * @param[in]  sample_count 登録見本数
 * @param[in]  threshold 判定しきい値 (0.0以下の場合はデフォルト値を使用)
 * @param[out] p_output 照合結果出力構造体
 * ================================================================= */
EXPORT void acoustic_embedding_identifier_classify(
    const float * p_query_64d,
    const float * p_prototypes_64d,
    UW sample_count,
    float threshold,
    acoustic_identifier_summary_output_t * p_output)
{
    if (NULL == p_output) {
        return;
    }
    memset(p_output, 0, sizeof(*p_output));
    p_output->threshold = (threshold > 0.0f) ? threshold : ACOUSTIC_EMBEDDING_SAFE_THRESHOLD;
    p_output->status = CPU0_ACOUSTIC_IDENTIFIER_SUMMARY_INVALID;

    if ((NULL == p_query_64d) || (NULL == p_prototypes_64d) || (0U == sample_count)) {
        p_output->status = CPU0_ACOUSTIC_IDENTIFIER_SUMMARY_NOT_READY;
        return;
    }

    float min_distance = 2.0f;
    UW best_sample = 0U;

    for (UW s = 0U; s < sample_count; s++) {
        const float * p_sample = &p_prototypes_64d[s * ACOUSTIC_EMBEDDING_DIMENSION];
        float dot = 0.0f;
        for (UW i = 0U; i < ACOUSTIC_EMBEDDING_DIMENSION; i++) {
            dot += p_query_64d[i] * p_sample[i];
        }
        float const dist = 1.0f - dot;
        if (dist < min_distance) {
            min_distance = dist;
            best_sample = s;
        }
    }

    p_output->minimum_cosine_distance = min_distance;
    p_output->sample_index = best_sample;
    p_output->active_frame_count = ACOUSTIC_EMBEDDING_INPUT_FRAMES;

    if (min_distance <= p_output->threshold) {
        p_output->status = CPU0_ACOUSTIC_IDENTIFIER_SUMMARY_TARGET;
    } else {
        p_output->status = CPU0_ACOUSTIC_IDENTIFIER_SUMMARY_NOT_TARGET;
    }
}

#else /* (CPU0_USE_ACOUSTIC_EMBEDDING_TFLM == 0U) */

/** =================================================================*
 * @brief  音響埋め込み推論器初期化スタブ
 * @return 常にFALSE (無効化状態)
 * ================================================================= */
EXPORT BOOL acoustic_embedding_identifier_init(void) {
    return FALSE;
}

/** =================================================================*
 * @brief  特徴量フレームからの64次元音響埋め込み抽出スタブ
 * @param[in]  p_mel_80x32 未使用
 * @param[out] p_embedding_64d 未使用
 * @return 常にFALSE (無効化状態)
 * ================================================================= */
EXPORT BOOL acoustic_embedding_identifier_extract(const B * p_mel_80x32, float * p_embedding_64d) {
    (void) p_mel_80x32;
    (void) p_embedding_64d;
    return FALSE;
}

/** =================================================================*
 * @brief  抽出埋め込みと現場見本の最近傍コサイン距離照合スタブ
 * @param[in]  p_query_64d 未使用
 * @param[in]  p_prototypes_64d 未使用
 * @param[in]  sample_count 未使用
 * @param[in]  threshold 未使用
 * @param[out] p_output 照合結果出力構造体 (NOT_READYを出力)
 * ================================================================= */
EXPORT void acoustic_embedding_identifier_classify(
    const float * p_query_64d,
    const float * p_prototypes_64d,
    UW sample_count,
    float threshold,
    acoustic_identifier_summary_output_t * p_output)
{
    (void) p_query_64d;
    (void) p_prototypes_64d;
    (void) sample_count;
    (void) threshold;
    if (NULL != p_output) {
        p_output->status = CPU0_ACOUSTIC_IDENTIFIER_SUMMARY_NOT_READY;
        p_output->minimum_cosine_distance = 2.0f;
        p_output->threshold = ACOUSTIC_EMBEDDING_SAFE_THRESHOLD;
        p_output->sample_index = 0U;
        p_output->active_frame_count = 0U;
    }
}

#endif /* CPU0_USE_ACOUSTIC_EMBEDDING_TFLM != 0U */
