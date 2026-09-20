/** =================================================================*
 * @file   acoustic_identifier.c
 * @brief  能動フレーム要約による現場音響照合
 * ================================================================= */
#include "services/acoustic_identifier.h"                   /* 能動フレーム要約と照合型 */
#include <math.h>                                           /* フレーム内標準偏差 */
#include <string.h>                                         /* 要約初期化 */

LOCAL B acoustic_identifier_int8_round(float value, B minimum); /* 仕様どおりのint8丸め */

/** =================================================================*
 * @brief  float値を最も近いint8へ半端をゼロから遠ざけて丸める
 * ================================================================= */
LOCAL B acoustic_identifier_int8_round(float value, B minimum) {
    W rounded = 0;
    if (value >= 0.0F) {
        rounded = (W) (value + 0.5F);
    } else {
        rounded = (W) (value - 0.5F);
    }
    if (rounded > 127) {
        rounded = 127;
    }
    if (rounded < (W) minimum) {
        rounded = (W) minimum;
    }
    return (B) rounded;
}

/** =================================================================*
 * @brief  32binフレームの母標準偏差で能動性を判定
 * @details 閾値と等しい場合は非能動。`std > 12`を固定契約とする。
 * ================================================================= */
EXPORT BOOL acoustic_identifier_frame_is_active(const B * p_frame) {
    if (NULL == p_frame) {
        return FALSE;
    }

    W sum = 0;
    for (UW index = 0U; index < CPU0_ACOUSTIC_FEATURE_BIN_COUNT; index++) {
        sum += (W) p_frame[index];
    }
    float const mean = (float) sum / (float) CPU0_ACOUSTIC_FEATURE_BIN_COUNT;
    float squared_sum = 0.0F;
    for (UW index = 0U; index < CPU0_ACOUSTIC_FEATURE_BIN_COUNT; index++) {
        float const difference = (float) p_frame[index] - mean;
        squared_sum += difference * difference;
    }
    float const stddev = sqrtf(squared_sum / (float) CPU0_ACOUSTIC_FEATURE_BIN_COUNT);
    return stddev > CPU0_ACOUSTIC_ACTIVE_STDDEV_LSB;
}

/** =================================================================*
 * @brief  単一スロット（40フレーム）の能動フレームを96次元int8へ要約
 * @details 能動フレームが0件の場合は全ゼロ（0）で埋める。
 * ================================================================= */
LOCAL void acoustic_identifier_slot_summary(const B * p_frames,
                                              UW frame_count,
                                              B * p_slot_summary,
                                              UW * p_active_count) {
    W sums[CPU0_ACOUSTIC_FEATURE_BIN_COUNT] = {0};
    float squared_sums[CPU0_ACOUSTIC_FEATURE_BIN_COUNT] = {0.0F};
    B maxima[CPU0_ACOUSTIC_FEATURE_BIN_COUNT];
    memset(maxima, 0x80, sizeof(maxima));
    UW active = 0U;

    for (UW frame_index = 0U; frame_index < frame_count; frame_index++) {
        const B * const p_frame = &p_frames[frame_index * CPU0_ACOUSTIC_FEATURE_BIN_COUNT];
        if (FALSE == acoustic_identifier_frame_is_active(p_frame)) {
            continue;
        }
        active++;
        for (UW bin = 0U; bin < CPU0_ACOUSTIC_FEATURE_BIN_COUNT; bin++) {
            float const value = (float) p_frame[bin];
            sums[bin] += (W) p_frame[bin];
            squared_sums[bin] += value * value;
            if (p_frame[bin] > maxima[bin]) {
                maxima[bin] = p_frame[bin];
            }
        }
    }

    *p_active_count = active;
    if (0U == active) {
        memset(p_slot_summary, 0, CPU0_ACOUSTIC_FEATURE_BIN_COUNT * 3U);
        return;
    }

    for (UW bin = 0U; bin < CPU0_ACOUSTIC_FEATURE_BIN_COUNT; bin++) {
        float const mean = (float) sums[bin] / (float) active;
        float const variance = (squared_sums[bin] / (float) active) - (mean * mean);
        p_slot_summary[bin] = acoustic_identifier_int8_round(mean, -128);
        p_slot_summary[CPU0_ACOUSTIC_FEATURE_BIN_COUNT + bin] =
            acoustic_identifier_int8_round(sqrtf((variance > 0.0F) ? variance : 0.0F), 0);
        p_slot_summary[(2U * CPU0_ACOUSTIC_FEATURE_BIN_COUNT) + bin] = maxima[bin];
    }
}

/** =================================================================*
 * @brief  80フレームを2スロット（前半40・後半40）に分割し192次元int8要約へ変換
 * ================================================================= */
EXPORT BOOL acoustic_identifier_summary_create(const B * p_frames,
                                                UW frame_count,
                                                B * p_summary,
                                                UB * p_active_frame_count) {
    if ((NULL == p_frames) || (NULL == p_summary) || (NULL == p_active_frame_count) ||
        (frame_count < CPU0_ACOUSTIC_EVENT_FRAME_COUNT)) {
        return FALSE;
    }

    UW slot1_active = 0U;
    UW slot2_active = 0U;
    acoustic_identifier_slot_summary(&p_frames[0], CPU0_ACOUSTIC_SLOT_FRAME_COUNT,
                                     &p_summary[0], &slot1_active);
    acoustic_identifier_slot_summary(&p_frames[CPU0_ACOUSTIC_SLOT_FRAME_COUNT * CPU0_ACOUSTIC_FEATURE_BIN_COUNT],
                                     CPU0_ACOUSTIC_SLOT_FRAME_COUNT,
                                     &p_summary[CPU0_ACOUSTIC_FEATURE_BIN_COUNT * 3U], &slot2_active);

    UW const total_active = slot1_active + slot2_active;
    *p_active_frame_count = (total_active > 255U) ? 255U : (UB) total_active;

    if (total_active < CPU0_ACOUSTIC_MIN_ACTIVE_FRAME_COUNT) {
        memset(p_summary, 0, CPU0_ACOUSTIC_SUMMARY_DIMENSION);
        return FALSE;
    }
    return TRUE;
}

/** =================================================================*
 * @brief  見本群の各bin最大パワーから代表ピーク周波数binを特定する
 * ================================================================= */
EXPORT UB acoustic_identifier_find_peak_bin(const B * p_samples, UW sample_count) {
    if ((NULL == p_samples) || (0U == sample_count)) {
        return 0U;
    }

    W bin_sums[CPU0_ACOUSTIC_FEATURE_BIN_COUNT] = {0};
    for (UW sample = 0U; sample < sample_count; sample++) {
        const B * const p_sample = &p_samples[sample * CPU0_ACOUSTIC_SUMMARY_DIMENSION];
        /* Slot 1 max: index 64..95, Slot 2 max: index 160..191 */
        const B * const p_max1 = &p_sample[2U * CPU0_ACOUSTIC_FEATURE_BIN_COUNT];
        const B * const p_max2 = &p_sample[5U * CPU0_ACOUSTIC_FEATURE_BIN_COUNT];
        for (UW bin = 0U; bin < CPU0_ACOUSTIC_FEATURE_BIN_COUNT; bin++) {
            B const m1 = p_max1[bin];
            B const m2 = p_max2[bin];
            bin_sums[bin] += (W) ((m1 > m2) ? m1 : m2);
        }
    }

    W max_sum = -32768;
    UB peak_bin = 0U;
    for (UW bin = 0U; bin < CPU0_ACOUSTIC_FEATURE_BIN_COUNT; bin++) {
        if (bin_sums[bin] > max_sum) {
            max_sum = bin_sums[bin];
            peak_bin = (UB) bin;
        }
    }
    return peak_bin;
}

/** =================================================================*
 * @brief  代表ピーク周波数binおよび暗騒音帯域に基づく32bin重みベクトルを構築
 * ================================================================= */
EXPORT void acoustic_identifier_build_weights(UB peak_bin, float * p_weights) {
    if (NULL == p_weights) {
        return;
    }

    for (UW bin = 0U; bin < CPU0_ACOUSTIC_FEATURE_BIN_COUNT; bin++) {
        p_weights[bin] = (bin < 3U) ? 0.5F : 1.0F;
    }

    /* ピーク近傍強調（暗騒音抑制より優先） */
    const struct {
        W offset;
        float weight;
    } k_peak_emphasis[] = {
        {-2, 1.3F},
        {-1, 1.8F},
        { 0, 2.5F},
        { 1, 1.8F},
        { 2, 1.3F},
    };

    for (UW idx = 0U; idx < (sizeof(k_peak_emphasis) / sizeof(k_peak_emphasis[0])); idx++) {
        W const b = (W) peak_bin + k_peak_emphasis[idx].offset;
        if ((b >= 0) && (b < (W) CPU0_ACOUSTIC_FEATURE_BIN_COUNT)) {
            p_weights[b] = k_peak_emphasis[idx].weight;
        }
    }
}

/** =================================================================*
 * @brief  192次元int8要約同士の重み付きcosine距離（1-cosine）を算出
 * ================================================================= */
EXPORT BOOL acoustic_identifier_weighted_cosine_distance(const B * p_left,
                                                         const B * p_right,
                                                         const float * p_bin_weights,
                                                         float * p_distance) {
    if ((NULL == p_left) || (NULL == p_right) || (NULL == p_distance)) {
        return FALSE;
    }

    float dot = 0.0F;
    float left_norm_squared = 0.0F;
    float right_norm_squared = 0.0F;
    for (UW index = 0U; index < CPU0_ACOUSTIC_SUMMARY_DIMENSION; index++) {
        float const w = (NULL != p_bin_weights) ?
                            p_bin_weights[index % CPU0_ACOUSTIC_FEATURE_BIN_COUNT] : 1.0F;
        float const left = (float) p_left[index];
        float const right = (float) p_right[index];
        dot += w * left * right;
        left_norm_squared += w * left * left;
        right_norm_squared += w * right * right;
    }
    if ((left_norm_squared <= 0.0F) || (right_norm_squared <= 0.0F)) {
        return FALSE;
    }
    float cosine = dot / sqrtf(left_norm_squared * right_norm_squared);
    if (cosine > 1.0F) {
        cosine = 1.0F;
    }
    if (cosine < -1.0F) {
        cosine = -1.0F;
    }
    *p_distance = 1.0F - cosine;
    return TRUE;
}

/** =================================================================*
 * @brief  192次元int8要約同士の均等重みcosine距離を算出
 * ================================================================= */
EXPORT BOOL acoustic_identifier_cosine_distance(const B * p_left, const B * p_right, float * p_distance) {
    return acoustic_identifier_weighted_cosine_distance(p_left, p_right, NULL, p_distance);
}

/** =================================================================*
 * @brief  個別見本のleave-one-out最近傍距離から受理しきい値を作る
 * ================================================================= */
EXPORT BOOL acoustic_identifier_leave_one_out_threshold(const B * p_samples,
                                                         UW sample_count,
                                                         const float * p_bin_weights,
                                                         float * p_threshold) {
    if ((NULL == p_samples) || (NULL == p_threshold) || (sample_count < 2U)) {
        return FALSE;
    }

    float auto_weights[CPU0_ACOUSTIC_FEATURE_BIN_COUNT];
    const float * p_effective_weights = p_bin_weights;
    if (NULL == p_effective_weights) {
        UB const peak_bin = acoustic_identifier_find_peak_bin(p_samples, sample_count);
        acoustic_identifier_build_weights(peak_bin, auto_weights);
        p_effective_weights = auto_weights;
    }

    float nearest[CPU0_ACOUSTIC_SAMPLE_COUNT] = {0.0F};
    if (sample_count > CPU0_ACOUSTIC_SAMPLE_COUNT) {
        return FALSE;
    }
    for (UW sample = 0U; sample < sample_count; sample++) {
        BOOL found = FALSE;
        float minimum = 0.0F;
        const B * const p_current = &p_samples[sample * CPU0_ACOUSTIC_SUMMARY_DIMENSION];
        for (UW other = 0U; other < sample_count; other++) {
            if (sample == other) {
                continue;
            }
            float distance = 0.0F;
            const B * const p_other = &p_samples[other * CPU0_ACOUSTIC_SUMMARY_DIMENSION];
            if (!acoustic_identifier_weighted_cosine_distance(p_current, p_other,
                                                               p_effective_weights, &distance)) {
                continue;
            }
            if ((!found) || (distance < minimum)) {
                minimum = distance;
                found = TRUE;
            }
        }
        if (!found) {
            return FALSE;
        }
        nearest[sample] = minimum;
    }

    float mean = 0.0F;
    for (UW sample = 0U; sample < sample_count; sample++) {
        mean += nearest[sample];
    }
    mean /= (float) sample_count;
    float variance = 0.0F;
    for (UW sample = 0U; sample < sample_count; sample++) {
        float const difference = nearest[sample] - mean;
        variance += difference * difference;
    }
    float threshold = mean + (CPU0_ACOUSTIC_IDENTIFIER_SIGMA_SCALE * sqrtf(variance / (float) sample_count));
    if (threshold < CPU0_ACOUSTIC_IDENTIFIER_THRESHOLD_MIN) {
        threshold = CPU0_ACOUSTIC_IDENTIFIER_THRESHOLD_MIN;
    } else if (threshold > CPU0_ACOUSTIC_IDENTIFIER_THRESHOLD_MAX) {
        threshold = CPU0_ACOUSTIC_IDENTIFIER_THRESHOLD_MAX;
    }
    *p_threshold = threshold;
    return TRUE;
}

/** =================================================================*
 * @brief  有効な要約を個別見本へ重み付き照合する
 * ================================================================= */
EXPORT void acoustic_identifier_summary_classify(const B * p_summary,
                                                  BOOL summary_valid,
                                                  UB active_frame_count,
                                                  const B * p_samples,
                                                  UW sample_count,
                                                  const float * p_bin_weights,
                                                  float threshold,
                                                  acoustic_identifier_summary_output_t * p_output) {
    if (NULL == p_output) {
        return;
    }
    p_output->active_frame_count = active_frame_count;
    p_output->sample_index = ~(UW) 0U;
    p_output->minimum_cosine_distance = -1.0F;
    p_output->threshold = threshold;
    p_output->status = CPU0_ACOUSTIC_IDENTIFIER_SUMMARY_INVALID;
    if (!summary_valid) {
        p_output->status = CPU0_ACOUSTIC_IDENTIFIER_SUMMARY_INDETERMINATE;
        return;
    }
    if ((NULL == p_summary) || (NULL == p_samples) || (sample_count < 2U) ||
        (sample_count > CPU0_ACOUSTIC_SAMPLE_COUNT) || (threshold < 0.0F)) {
        p_output->status = CPU0_ACOUSTIC_IDENTIFIER_SUMMARY_NOT_READY;
        return;
    }

    float auto_weights[CPU0_ACOUSTIC_FEATURE_BIN_COUNT];
    const float * p_effective_weights = p_bin_weights;
    if (NULL == p_effective_weights) {
        UB const peak_bin = acoustic_identifier_find_peak_bin(p_samples, sample_count);
        acoustic_identifier_build_weights(peak_bin, auto_weights);
        p_effective_weights = auto_weights;
    }

    BOOL found = FALSE;
    for (UW sample = 0U; sample < sample_count; sample++) {
        float distance = 0.0F;
        const B * const p_sample = &p_samples[sample * CPU0_ACOUSTIC_SUMMARY_DIMENSION];
        if (!acoustic_identifier_weighted_cosine_distance(p_summary, p_sample,
                                                           p_effective_weights, &distance)) {
            continue;
        }
        if ((!found) || (distance < p_output->minimum_cosine_distance)) {
            p_output->minimum_cosine_distance = distance;
            p_output->sample_index = sample;
            found = TRUE;
        }
    }
    if (!found) {
        p_output->status = CPU0_ACOUSTIC_IDENTIFIER_SUMMARY_NOT_READY;
        return;
    }
    p_output->status = (p_output->minimum_cosine_distance <= threshold) ?
                           CPU0_ACOUSTIC_IDENTIFIER_SUMMARY_TARGET :
                           CPU0_ACOUSTIC_IDENTIFIER_SUMMARY_NOT_TARGET;
}
