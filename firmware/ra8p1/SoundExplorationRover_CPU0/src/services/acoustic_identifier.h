/** =================================================================*
 * @file   acoustic_identifier.h
 * @brief  音響埋め込みの最近傍識別
 * ================================================================= */
#ifndef SEROV_CPU0_ACOUSTIC_IDENTIFIER_H
#define SEROV_CPU0_ACOUSTIC_IDENTIFIER_H

#include <tk/tkernel.h>                                     /* μT-Kernel基本型と公開範囲マクロ */

#define CPU0_ACOUSTIC_EMBEDDING_DIMENSION  (64U)
#define CPU0_ACOUSTIC_CLASS_INVALID        (0xFFU)

typedef struct st_acoustic_identifier_prototype {
    B embedding[CPU0_ACOUSTIC_EMBEDDING_DIMENSION];
    UW max_squared_distance;
    UB class_id;
    BOOL valid;
} acoustic_identifier_prototype_t;

typedef struct st_acoustic_identifier_output {
    UW squared_distance;
    UW prototype_index;
    UB class_id;
    BOOL matched;
} acoustic_identifier_output_t;

/* 64次元int8埋め込み間の二乗L2距離算出 */
EXPORT UW acoustic_identifier_squared_l2(const B * p_embedding, const B * p_prototype);
/* 有効プロトタイプから最近傍を選択し、距離しきい値で受理 */
EXPORT void acoustic_identifier_classify(const B * p_embedding,
                                         const acoustic_identifier_prototype_t * p_prototypes,
                                         UW prototype_count,
                                         acoustic_identifier_output_t * p_output);

#endif /* SEROV_CPU0_ACOUSTIC_IDENTIFIER_H */
