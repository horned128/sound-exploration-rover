/** =================================================================*
 * @file   acoustic_identifier.c
 * @brief  音響埋め込みの最近傍識別
 * ================================================================= */
#include "services/acoustic_identifier.h"                  /* 音響埋め込みとプロトタイプ型 */

/** =================================================================*
 * @brief  64次元int8埋め込み間の二乗L2距離算出
 * @details 同一TFLite出力量子化の埋め込み同士を比較するため、scaleとzero pointは差分で相殺される。
 * @param[in] p_embedding 識別対象の64次元埋め込み
 * @param[in] p_prototype 比較対象の64次元プロトタイプ
 * @return 二乗L2距離。引数がNULLの場合はUW最大値
 * ================================================================= */
EXPORT UW acoustic_identifier_squared_l2(const B * p_embedding, const B * p_prototype) {
    if ((NULL == p_embedding) || (NULL == p_prototype)) {
        return ~(UW) 0U;
    }

    UW squared_distance = 0U;
    for (UW index = 0U; index < CPU0_ACOUSTIC_EMBEDDING_DIMENSION; index++) {
        W const difference = (W) p_embedding[index] - (W) p_prototype[index];
        squared_distance += (UW) (difference * difference);
    }
    return squared_distance;
}

/** =================================================================*
 * @brief  最近傍プロトタイプによる音響クラス識別
 * @details 有効な最近傍を決定してから、そのプロトタイプ固有の二乗距離しきい値で受理する。
 *          同距離の場合は配列の先頭を選び、再現可能な結果にする。
 * @param[in] p_embedding 識別対象の64次元埋め込み
 * @param[in] p_prototypes 比較するプロトタイプ配列
 * @param[in] prototype_count プロトタイプ数
 * @param[out] p_output 識別結果
 * ================================================================= */
EXPORT void acoustic_identifier_classify(const B * p_embedding,
                                         const acoustic_identifier_prototype_t * p_prototypes,
                                         UW prototype_count,
                                         acoustic_identifier_output_t * p_output) {
    if (NULL == p_output) {
        return;
    }

    p_output->squared_distance = ~(UW) 0U;
    p_output->prototype_index = ~(UW) 0U;
    p_output->class_id = CPU0_ACOUSTIC_CLASS_INVALID;
    p_output->matched = FALSE;

    if ((NULL == p_embedding) || (NULL == p_prototypes)) {
        return;
    }

    const acoustic_identifier_prototype_t * p_nearest = NULL;
    for (UW index = 0U; index < prototype_count; index++) {
        if (FALSE == p_prototypes[index].valid) {
            continue;
        }

        UW const distance = acoustic_identifier_squared_l2(p_embedding, p_prototypes[index].embedding);
        if (distance < p_output->squared_distance) {
            p_output->squared_distance = distance;
            p_output->prototype_index = index;
            p_output->class_id = p_prototypes[index].class_id;
            p_nearest = &p_prototypes[index];
        }
    }

    if ((NULL != p_nearest) && (p_output->squared_distance <= p_nearest->max_squared_distance)) {
        p_output->matched = TRUE;
    }
}
