/** =================================================================*
 * @file   acoustic_identifier.c
 * @brief  能動フレーム要約による現場音響照合
 * ================================================================= */
#include "services/acoustic_identifier.h"                  /* 能動フレーム要約と照合型 */

#include <math.h>                                            /* sqrtf: フレーム内母標準偏差 */
#include <string.h>                                          /* memset: 要約初期化 */

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
 * @brief  能動フレームを96次元int8のmean/std/max要約へ変換
 * ================================================================= */
EXPORT BOOL acoustic_identifier_summary_create(const B * p_frames,
                                                UW frame_count,
                                                B * p_summary,
                                                UB * p_active_frame_count) {
    if ((NULL == p_frames) || (NULL == p_summary) || (NULL == p_active_frame_count)) {
        return FALSE;
    }

    W sums[CPU0_ACOUSTIC_FEATURE_BIN_COUNT] = {0};
    float squared_sums[CPU0_ACOUSTIC_FEATURE_BIN_COUNT] = {0.0F};
    B maxima[CPU0_ACOUSTIC_FEATURE_BIN_COUNT];
    memset(maxima, 0x80, sizeof(maxima));
    UW active_count = 0U;

    for (UW frame_index = 0U; frame_index < frame_count; frame_index++) {
        const B * const p_frame = &p_frames[frame_index * CPU0_ACOUSTIC_FEATURE_BIN_COUNT];
        if (FALSE == acoustic_identifier_frame_is_active(p_frame)) {
            continue;
        }
        active_count++;
        for (UW bin = 0U; bin < CPU0_ACOUSTIC_FEATURE_BIN_COUNT; bin++) {
            float const value = (float) p_frame[bin];
            sums[bin] += (W) p_frame[bin];
            squared_sums[bin] += value * value;
            if (p_frame[bin] > maxima[bin]) {
                maxima[bin] = p_frame[bin];
            }
        }
    }

    *p_active_frame_count = (active_count > 255U) ? 255U : (UB) active_count;
    if (active_count < CPU0_ACOUSTIC_MIN_ACTIVE_FRAME_COUNT) {
        return FALSE;
    }

    for (UW bin = 0U; bin < CPU0_ACOUSTIC_FEATURE_BIN_COUNT; bin++) {
        float const mean = (float) sums[bin] / (float) active_count;
        float const variance = (squared_sums[bin] / (float) active_count) - (mean * mean);
        p_summary[bin] = acoustic_identifier_int8_round(mean, -128);
        p_summary[CPU0_ACOUSTIC_FEATURE_BIN_COUNT + bin] =
            acoustic_identifier_int8_round(sqrtf((variance > 0.0F) ? variance : 0.0F), 0);
        p_summary[(2U * CPU0_ACOUSTIC_FEATURE_BIN_COUNT) + bin] = maxima[bin];
    }
    return TRUE;
}

/** =================================================================*
 * @brief  96次元int8要約同士のcosine距離を算出
 * ================================================================= */
EXPORT BOOL acoustic_identifier_cosine_distance(const B * p_left, const B * p_right, float * p_distance) {
    if ((NULL == p_left) || (NULL == p_right) || (NULL == p_distance)) {
        return FALSE;
    }

    float dot = 0.0F;
    float left_norm_squared = 0.0F;
    float right_norm_squared = 0.0F;
    for (UW index = 0U; index < CPU0_ACOUSTIC_SUMMARY_DIMENSION; index++) {
        float const left = (float) p_left[index];
        float const right = (float) p_right[index];
        dot += left * right;
        left_norm_squared += left * left;
        right_norm_squared += right * right;
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
 * @brief  個別見本のleave-one-out最近傍距離から受理しきい値を作る
 * ================================================================= */
EXPORT BOOL acoustic_identifier_leave_one_out_threshold(const B * p_samples,
                                                         UW sample_count,
                                                         float * p_threshold) {
    if ((NULL == p_samples) || (NULL == p_threshold) || (sample_count < 2U)) {
        return FALSE;
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
            if (!acoustic_identifier_cosine_distance(p_current, p_other, &distance)) {
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
    *p_threshold = mean + (3.0F * sqrtf(variance / (float) sample_count));
    return TRUE;
}

/** =================================================================*
 * @brief  有効な要約を個別見本へ照合する
 * ================================================================= */
EXPORT void acoustic_identifier_summary_classify(const B * p_summary,
                                                  BOOL summary_valid,
                                                  UB active_frame_count,
                                                  const B * p_samples,
                                                  UW sample_count,
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

    BOOL found = FALSE;
    for (UW sample = 0U; sample < sample_count; sample++) {
        float distance = 0.0F;
        const B * const p_sample = &p_samples[sample * CPU0_ACOUSTIC_SUMMARY_DIMENSION];
        if (!acoustic_identifier_cosine_distance(p_summary, p_sample, &distance)) {
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
