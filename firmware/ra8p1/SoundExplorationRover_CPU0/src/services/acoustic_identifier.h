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
#define CPU0_ACOUSTIC_SUMMARY_DIMENSION       (96U)
#define CPU0_ACOUSTIC_SAMPLE_COUNT             (5U)
#define CPU0_ACOUSTIC_MIN_ACTIVE_FRAME_COUNT  (10U)
#define CPU0_ACOUSTIC_ACTIVE_STDDEV_LSB       (12.0F)

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
/* 能動フレームのmean[32], std[32], max[32]を96次元int8へ要約する。 */
EXPORT BOOL acoustic_identifier_summary_create(const B * p_frames,
                                                UW frame_count,
                                                B * p_summary,
                                                UB * p_active_frame_count);
/* 零ベクトルは距離を返さずFALSEにし、判定不能と扱えるようにする。 */
EXPORT BOOL acoustic_identifier_cosine_distance(const B * p_left, const B * p_right, float * p_distance);
/* 見本ごとの最近傍距離からmean + 3 sigmaを計算する。 */
EXPORT BOOL acoustic_identifier_leave_one_out_threshold(const B * p_samples,
                                                         UW sample_count,
                                                         float * p_threshold);
/* 要約済みの入力を個別見本へ照合する。見本不足と能動不足は非対象と混同しない。 */
EXPORT void acoustic_identifier_summary_classify(const B * p_summary,
                                                  BOOL summary_valid,
                                                  UB active_frame_count,
                                                  const B * p_samples,
                                                  UW sample_count,
                                                  float threshold,
                                                  acoustic_identifier_summary_output_t * p_output);

#endif /* SEROV_CPU0_ACOUSTIC_IDENTIFIER_H */
