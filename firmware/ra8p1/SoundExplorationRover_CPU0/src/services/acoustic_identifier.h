/** =================================================================*
 * @file   acoustic_identifier.h
 * @brief  能動フレーム要約による現場音響照合
 * ================================================================= */
#ifndef SEROV_CPU0_ACOUSTIC_IDENTIFIER_H
#define SEROV_CPU0_ACOUSTIC_IDENTIFIER_H

#include <tk/tkernel.h>                                     /* μT-Kernel基本型と公開範囲マクロ */

/* 特徴量要約の固定契約 */
#define CPU0_ACOUSTIC_FEATURE_BIN_COUNT       (32U)
#define CPU0_ACOUSTIC_EVENT_FRAME_COUNT       (80U)
#define CPU0_ACOUSTIC_SLOT_FRAME_COUNT        (40U)
#define CPU0_ACOUSTIC_SLOT_COUNT              (2U)
#define CPU0_ACOUSTIC_SUMMARY_DIMENSION       (192U)
#define CPU0_ACOUSTIC_SAMPLE_COUNT             (5U)
#define CPU0_ACOUSTIC_MIN_ACTIVE_FRAME_COUNT  (10U)
#define CPU0_ACOUSTIC_ACTIVE_STDDEV_LSB       (12.0F)
#define CPU0_ACOUSTIC_IDENTIFIER_THRESHOLD_MIN (0.08F)
#define CPU0_ACOUSTIC_IDENTIFIER_THRESHOLD_MAX (0.22F)
#define CPU0_ACOUSTIC_IDENTIFIER_SIGMA_SCALE   (1.5F)

typedef enum e_acoustic_identifier_summary_status {
    CPU0_ACOUSTIC_IDENTIFIER_SUMMARY_INVALID = 0U,
    CPU0_ACOUSTIC_IDENTIFIER_SUMMARY_INDETERMINATE,
    CPU0_ACOUSTIC_IDENTIFIER_SUMMARY_NOT_READY,
    CPU0_ACOUSTIC_IDENTIFIER_SUMMARY_NOT_TARGET,
    CPU0_ACOUSTIC_IDENTIFIER_SUMMARY_TARGET,
} acoustic_identifier_summary_status_t;

typedef struct st_acoustic_identifier_summary_output {
    UB active_frame_count;
    UW sample_index;
    float minimum_cosine_distance;
    float threshold;
    acoustic_identifier_summary_status_t status;
} acoustic_identifier_summary_output_t;

/* 32binフレームの母標準偏差が12 LSBを厳密に超える場合だけ能動とする。 */
EXPORT BOOL acoustic_identifier_frame_is_active(const B * p_frame);
/* 2スロット分割により前半・後半各96次元を結合した192次元int8要約を生成する。 */
EXPORT BOOL acoustic_identifier_summary_create(const B * p_frames,
                                                UW frame_count,
                                                B * p_summary,
                                                UB * p_active_frame_count);
/* 見本群の各bin最大パワーから代表ピーク周波数binを特定する。 */
EXPORT UB acoustic_identifier_find_peak_bin(const B * p_samples, UW sample_count);
/* 代表ピーク周波数binおよび暗騒音帯域に基づく32bin重みベクトルを構築する。 */
EXPORT void acoustic_identifier_build_weights(UB peak_bin, float * p_weights);
/* 192次元int8要約同士の重み付きcosine距離（1-cosine）を算出する。 */
EXPORT BOOL acoustic_identifier_weighted_cosine_distance(const B * p_left,
                                                         const B * p_right,
                                                         const float * p_bin_weights,
                                                         float * p_distance);
/* 192次元int8要約同士の均等重みcosine距離（1-cosine）を算出する。 */
EXPORT BOOL acoustic_identifier_cosine_distance(const B * p_left, const B * p_right, float * p_distance);
/* 個別見本の重み付きleave-one-out最近傍距離から受理しきい値を作る。 */
EXPORT BOOL acoustic_identifier_leave_one_out_threshold(const B * p_samples,
                                                         UW sample_count,
                                                         const float * p_bin_weights,
                                                         float * p_threshold);
/* 要約済みの入力を個別見本へ重み付き照合する。 */
EXPORT void acoustic_identifier_summary_classify(const B * p_summary,
                                                  BOOL summary_valid,
                                                  UB active_frame_count,
                                                  const B * p_samples,
                                                  UW sample_count,
                                                  const float * p_bin_weights,
                                                  float threshold,
                                                  acoustic_identifier_summary_output_t * p_output);

#endif /* SEROV_CPU0_ACOUSTIC_IDENTIFIER_H */
