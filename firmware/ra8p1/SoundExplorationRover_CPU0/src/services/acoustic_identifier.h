/** =================================================================*
 * @file   acoustic_identifier.h
 * @brief  能動フレーム要約による現場音響照合
 * ================================================================= */
#ifndef SEROV_CPU0_ACOUSTIC_IDENTIFIER_H
#define SEROV_CPU0_ACOUSTIC_IDENTIFIER_H

#include <tk/tkernel.h>                                     /* μT-Kernel基本型と公開範囲マクロ */

/* 特徴量要約の固定契約 */
#define CPU0_ACOUSTIC_FEATURE_BIN_COUNT    (32U)            /**< 音響特徴量の周波数bin数 */
#define CPU0_ACOUSTIC_EVENT_FRAME_COUNT    (80U)            /**< 音響イベントの収集フレーム数 */
#define CPU0_ACOUSTIC_SLOT_FRAME_COUNT     (40U)            /**< 音響スロットの保持フレーム数 */
#define CPU0_ACOUSTIC_SLOT_COUNT           (2U)             /**< 音響特徴量スロット数 */
#define CPU0_ACOUSTIC_SUMMARY_DIMENSION    (192U)           /**< 音響要約ベクトルの次元数 */
#define CPU0_ACOUSTIC_SAMPLE_COUNT         (5U)             /**< 現場学習で保持する音響見本数 */
#define CPU0_ACOUSTIC_MIN_ACTIVE_FRAME_COUNT (10U)          /**< 識別に必要な有効フレーム最小数 */
#define CPU0_ACOUSTIC_ACTIVE_STDDEV_LSB    (12.0F)          /**< 有効フレーム判定σ[LSB] */
#define CPU0_ACOUSTIC_IDENTIFIER_THRESHOLD_MIN (0.08F)      /**< 音響識別しきい値の下限 */
#define CPU0_ACOUSTIC_IDENTIFIER_THRESHOLD_MAX (0.20F)      /**< 音響識別しきい値の上限 */
#define CPU0_ACOUSTIC_IDENTIFIER_SIGMA_SCALE (1.5F)         /**< 音響識別しきい値の標準偏差倍率 */
#define CPU0_ACOUSTIC_IDENTIFIER_PEAK_TOLERANCE_BINS (3U)   /**< TARGETを許す代表ピークbinの差 */

/**< 音響要約と現場見本照合の判定状態 */
typedef enum e_acoustic_identifier_summary_status {
    CPU0_ACOUSTIC_IDENTIFIER_SUMMARY_INVALID = 0U,          /**< 要約が無効 */
    CPU0_ACOUSTIC_IDENTIFIER_SUMMARY_INDETERMINATE,         /**< 判定材料不足で不確定 */
    CPU0_ACOUSTIC_IDENTIFIER_SUMMARY_NOT_READY,             /**< 見本照合の準備未完了 */
    CPU0_ACOUSTIC_IDENTIFIER_SUMMARY_NOT_TARGET,            /**< 見本と一致しない */
    CPU0_ACOUSTIC_IDENTIFIER_SUMMARY_TARGET,                /**< 見本と一致 */
} acoustic_identifier_summary_status_t;

/**< 音響要約と現場見本照合の判定結果 */
typedef struct st_acoustic_identifier_summary_output {
    UB active_frame_count;                                  /**< 要約に使用した有効フレーム数 */
    UW sample_index;                                        /**< 最も近かった見本の番号 */
    float minimum_cosine_distance;                          /**< 最小cosine距離 */
    float threshold;                                        /**< 判定に使用した受理しきい値 */
    acoustic_identifier_summary_status_t status;            /**< 見本照合の判定状態 */
} acoustic_identifier_summary_output_t;

/* 32binフレームの母標準偏差が12 LSBを厳密に超える場合だけ能動とする。 */
EXPORT BOOL acoustic_identifier_frame_is_active(const B * p_frame); /* 音響フレーム有効判定 */
EXPORT BOOL acoustic_identifier_summary_create(const B * p_frames,
                                                UW frame_count,
                                                B * p_summary,
                                                UB * p_active_frame_count); /* 192次元int8要約生成 */
/* 見本群の各bin最大パワーから代表ピーク周波数binを特定する。 */
EXPORT UB acoustic_identifier_find_peak_bin(const B * p_samples, UW sample_count); /* peak bin取得 */
/* 代表ピーク周波数binおよび暗騒音帯域に基づく32bin重みベクトルを構築する。 */
EXPORT void acoustic_identifier_build_weights(UB peak_bin, float * p_weights); /* 音響特徴量bin重み生成 */
EXPORT BOOL acoustic_identifier_weighted_cosine_distance(const B * p_left,
                                                         const B * p_right,
                                                         const float * p_bin_weights,
                                                         float * p_distance); /* 重み付きcosine距離算出 */
EXPORT BOOL acoustic_identifier_cosine_distance(const B * p_left, const B * p_right,
                                               float * p_distance); /* cosine距離 */
EXPORT BOOL acoustic_identifier_leave_one_out_threshold(const B * p_samples,
                                                         UW sample_count,
                                                         const float * p_bin_weights,
                                                         float * p_threshold); /* 見本照合しきい値算出 */
EXPORT void acoustic_identifier_summary_classify(
    const B * p_summary,
    BOOL summary_valid,
    UB active_frame_count,
    const B * p_samples,
    UW sample_count,
    const float * p_bin_weights,
    float threshold,
    acoustic_identifier_summary_output_t * p_output); /* 音響要約照合 */

#endif /* SEROV_CPU0_ACOUSTIC_IDENTIFIER_H */
