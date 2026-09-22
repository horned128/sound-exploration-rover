/** =================================================================*
 * @file   sound_source_localizer.c
 * @brief  重み付き方位線交点によるbearing-only音源位置推定
 * ================================================================= */
#include "services/sound_source_localizer.h"                /* 公開入出力 */
#include "config/control_config.h"                          /* 幾何・到着判定値 */
#include <math.h>                                           /* 三角関数、有限値判定 */

#define SOUND_LOCALIZER_PI                 (3.14159265358979323846F)
#define SOUND_LOCALIZER_RAD_TO_DEG(rad)    ((rad) * (180.0F / SOUND_LOCALIZER_PI))
#define SOUND_LOCALIZER_DEG_TO_RAD(deg)    ((deg) * (SOUND_LOCALIZER_PI / 180.0F))
#define SOUND_LOCALIZER_EPSILON            (1.0e-6F)

typedef struct st_sound_bearing_observation {
    float x_mm;
    float y_mm;
    float bearing_rad;
    float weight;
    UW timestamp_ms;
    UW sequence;
} sound_bearing_observation_t;

typedef struct st_sound_source_localizer_context {
    sound_bearing_observation_t observations[CPU0_SOUND_LOCALIZATION_MAX_OBSERVATIONS];
    UB observation_count;
    BOOL sequence_valid;
    UW last_sequence;
    BOOL source_valid;
    float source_x_mm;
    float source_y_mm;
    UB source_confidence;
    UW last_sound_ms;
    sound_arrival_state_t arrival_state;
    UW arrival_started_ms;
    UB arrival_confirm_count;
} sound_source_localizer_context_t;

LOCAL sound_source_localizer_context_t localizer;

LOCAL float sound_source_angle_normalize(float angle_rad) {
    while (angle_rad > SOUND_LOCALIZER_PI) {
        angle_rad -= 2.0F * SOUND_LOCALIZER_PI;
    }
    while (angle_rad < -SOUND_LOCALIZER_PI) {
        angle_rad += 2.0F * SOUND_LOCALIZER_PI;
    }
    return angle_rad;
}

LOCAL UH sound_source_u16_round_clamp(float value) {
    if (!(value > 0.0F)) {
        return 0U;
    }
    if (value >= 65535.0F) {
        return 65535U;
    }
    return (UH) roundf(value);
}

LOCAL UB sound_source_u8_round_clamp(float value) {
    if (!(value > 0.0F)) {
        return 0U;
    }
    if (value >= 100.0F) {
        return 100U;
    }
    return (UB) roundf(value);
}

LOCAL void sound_source_observations_expire(UW now_ms) {
    UB destination = 0U;
    for (UB source = 0U; source < localizer.observation_count; source++) {
        if ((now_ms - localizer.observations[source].timestamp_ms) <=
            CPU0_SOUND_LOCALIZATION_OBSERVATION_MAX_AGE_MS) {
            if (destination != source) {
                localizer.observations[destination] = localizer.observations[source];
            }
            destination++;
        }
    }
    localizer.observation_count = destination;
}

LOCAL void sound_source_observation_push(const sound_source_localizer_input_t * p_input) {
    if (!p_input->new_observation || !p_input->sound_valid || !p_input->pose_valid ||
        (p_input->doa_confidence < CPU0_SOUND_LOCALIZATION_MIN_CONFIDENCE)) {
        return;
    }
    if (localizer.sequence_valid && (p_input->observation_sequence == localizer.last_sequence)) {
        return;
    }
    localizer.sequence_valid = TRUE;
    localizer.last_sequence = p_input->observation_sequence;
    localizer.last_sound_ms = p_input->now_ms;

    if (localizer.observation_count >= CPU0_SOUND_LOCALIZATION_MAX_OBSERVATIONS) {
        for (UB index = 1U; index < localizer.observation_count; index++) {
            localizer.observations[index - 1U] = localizer.observations[index];
        }
        localizer.observation_count--;
    }

    sound_bearing_observation_t * const p_observation =
        &localizer.observations[localizer.observation_count++];
    p_observation->x_mm = (float) p_input->pose.x_mm;
    p_observation->y_mm = (float) p_input->pose.y_mm;
    p_observation->bearing_rad = sound_source_angle_normalize(
        ((float) p_input->pose.theta_mrad / 1000.0F) -
        SOUND_LOCALIZER_DEG_TO_RAD((float) p_input->relative_doa_deg));
    p_observation->weight = (float) p_input->doa_confidence / 100.0F;
    p_observation->timestamp_ms = p_input->now_ms;
    p_observation->sequence = p_input->observation_sequence;
}

LOCAL BOOL sound_source_estimate(float * p_x_mm, float * p_y_mm, float * p_residual_mm,
                                 float * p_baseline_mm, float * p_crossing_deg,
                                 UB * p_confidence) {
    if (localizer.observation_count < 2U) {
        return FALSE;
    }

    float a_xx = 0.0F;
    float a_xy = 0.0F;
    float a_yy = 0.0F;
    float b_x = 0.0F;
    float b_y = 0.0F;
    float weight_sum = 0.0F;
    float confidence_sum = 0.0F;
    float baseline_mm = 0.0F;
    float crossing_rad = 0.0F;

    for (UB first = 0U; first < localizer.observation_count; first++) {
        sound_bearing_observation_t const * const observation = &localizer.observations[first];
        float const nx = -sinf(observation->bearing_rad);
        float const ny = cosf(observation->bearing_rad);
        float const projection = (nx * observation->x_mm) + (ny * observation->y_mm);
        a_xx += observation->weight * nx * nx;
        a_xy += observation->weight * nx * ny;
        a_yy += observation->weight * ny * ny;
        b_x += observation->weight * nx * projection;
        b_y += observation->weight * ny * projection;
        weight_sum += observation->weight;
        confidence_sum += observation->weight * observation->weight * 100.0F;

        for (UB second = (UB) (first + 1U); second < localizer.observation_count; second++) {
            float const dx = localizer.observations[second].x_mm - observation->x_mm;
            float const dy = localizer.observations[second].y_mm - observation->y_mm;
            float const pair_baseline_mm = sqrtf((dx * dx) + (dy * dy));
            if (pair_baseline_mm > baseline_mm) {
                baseline_mm = pair_baseline_mm;
            }
            float angle = fabsf(sound_source_angle_normalize(
                localizer.observations[second].bearing_rad - observation->bearing_rad));
            if (angle > (SOUND_LOCALIZER_PI * 0.5F)) {
                angle = SOUND_LOCALIZER_PI - angle;
            }
            if (angle > crossing_rad) {
                crossing_rad = angle;
            }
        }
    }

    float const crossing_deg = SOUND_LOCALIZER_RAD_TO_DEG(crossing_rad);
    float const determinant = (a_xx * a_yy) - (a_xy * a_xy);
    float const trace = a_xx + a_yy;
    *p_baseline_mm = baseline_mm;
    *p_crossing_deg = crossing_deg;
    if ((baseline_mm < (float) CPU0_SOUND_LOCALIZATION_MIN_BASELINE_MM) ||
        (crossing_deg < (float) CPU0_SOUND_LOCALIZATION_MIN_CROSSING_DEG) ||
        (determinant <= SOUND_LOCALIZER_EPSILON) ||
        ((determinant / (trace * trace)) < 0.005F) || (weight_sum <= SOUND_LOCALIZER_EPSILON)) {
        return FALSE;
    }

    float const x_mm = ((b_x * a_yy) - (a_xy * b_y)) / determinant;
    float const y_mm = ((a_xx * b_y) - (a_xy * b_x)) / determinant;
    if (!isfinite(x_mm) || !isfinite(y_mm)) {
        return FALSE;
    }

    float residual_sum = 0.0F;
    UB forward_count = 0U;
    for (UB index = 0U; index < localizer.observation_count; index++) {
        sound_bearing_observation_t const * const observation = &localizer.observations[index];
        float const dx = x_mm - observation->x_mm;
        float const dy = y_mm - observation->y_mm;
        float const nx = -sinf(observation->bearing_rad);
        float const ny = cosf(observation->bearing_rad);
        float const residual = (nx * dx) + (ny * dy);
        residual_sum += observation->weight * residual * residual;
        if (((cosf(observation->bearing_rad) * dx) + (sinf(observation->bearing_rad) * dy)) > 0.0F) {
            forward_count++;
        }
    }
    float const residual_mm = sqrtf(residual_sum / weight_sum);
    if (!isfinite(residual_mm) || (residual_mm > (float) CPU0_SOUND_LOCALIZATION_MAX_RESIDUAL_MM) ||
        (forward_count * 3U < localizer.observation_count * 2U)) {
        return FALSE;
    }

    float const average_doa_confidence = confidence_sum / weight_sum;
    float const crossing_factor = fminf(crossing_deg / 45.0F, 1.0F);
    float const baseline_factor = fminf(baseline_mm / 600.0F, 1.0F);
    float const residual_factor = fmaxf(0.0F, 1.0F -
        (residual_mm / (float) CPU0_SOUND_LOCALIZATION_MAX_RESIDUAL_MM));
    float const quality = average_doa_confidence *
        (0.50F + (0.20F * crossing_factor) + (0.15F * baseline_factor) + (0.15F * residual_factor));

    *p_x_mm = x_mm;
    *p_y_mm = y_mm;
    *p_residual_mm = residual_mm;
    *p_confidence = sound_source_u8_round_clamp(quality);
    return TRUE;
}

EXPORT void sound_source_localizer_init(void) {
    localizer = (sound_source_localizer_context_t){
        .arrival_state = CPU0_SOUND_ARRIVAL_SEARCH,
    };
}

EXPORT void sound_source_localizer_step(const sound_source_localizer_input_t * p_input,
                                        sound_source_localizer_output_t * p_output) {
    if ((NULL == p_input) || (NULL == p_output)) {
        return;
    }
    *p_output = (sound_source_localizer_output_t){0};
    sound_source_observations_expire(p_input->now_ms);
    sound_source_observation_push(p_input);

    float estimate_x_mm = 0.0F;
    float estimate_y_mm = 0.0F;
    float residual_mm = 0.0F;
    float baseline_mm = 0.0F;
    float crossing_deg = 0.0F;
    UB confidence = 0U;
    BOOL geometry_valid = sound_source_estimate(&estimate_x_mm, &estimate_y_mm, &residual_mm,
                                                 &baseline_mm, &crossing_deg, &confidence);
    if (geometry_valid) {
        float const current_dx = estimate_x_mm - (float) p_input->pose.x_mm;
        float const current_dy = estimate_y_mm - (float) p_input->pose.y_mm;
        geometry_valid = sqrtf((current_dx * current_dx) + (current_dy * current_dy)) <=
                         (float) CPU0_SOUND_LOCALIZATION_MAX_RANGE_MM;
    }
    p_output->localization_geometry_valid = geometry_valid;
    p_output->localization_residual_mm = sound_source_u16_round_clamp(residual_mm);
    p_output->baseline_mm = sound_source_u16_round_clamp(baseline_mm);
    p_output->bearing_crossing_angle_deg = sound_source_u16_round_clamp(crossing_deg);
    p_output->observation_count = localizer.observation_count;

    if (geometry_valid) {
        float shift_mm = 0.0F;
        if (localizer.source_valid) {
            float const dx = estimate_x_mm - localizer.source_x_mm;
            float const dy = estimate_y_mm - localizer.source_y_mm;
            shift_mm = sqrtf((dx * dx) + (dy * dy));
        }
        p_output->source_position_shift_mm = sound_source_u16_round_clamp(shift_mm);
        if (!localizer.source_valid || (shift_mm <= (float) CPU0_SOUND_LOCALIZATION_MAX_JUMP_MM)) {
            localizer.source_x_mm = estimate_x_mm;
            localizer.source_y_mm = estimate_y_mm;
            localizer.source_confidence = confidence;
            localizer.source_valid = TRUE;
        } else {
            p_output->localization_geometry_valid = FALSE;
        }
    }

    BOOL const target_fresh = localizer.source_valid &&
        ((p_input->now_ms - localizer.last_sound_ms) <= CPU0_SOUND_LOCALIZATION_HOLD_MS);
    if (localizer.source_valid) {
        float const dx = localizer.source_x_mm - (float) p_input->pose.x_mm;
        float const dy = localizer.source_y_mm - (float) p_input->pose.y_mm;
        float const range_mm = sqrtf((dx * dx) + (dy * dy));
        float const relative_bearing_rad = sound_source_angle_normalize(
            ((float) p_input->pose.theta_mrad / 1000.0F) - atan2f(dy, dx));
        p_output->source_x_mm = (W) roundf(localizer.source_x_mm);
        p_output->source_y_mm = (W) roundf(localizer.source_y_mm);
        p_output->source_range_mm = (UW) roundf(range_mm);
        p_output->source_bearing_deg = (H) roundf(SOUND_LOCALIZER_RAD_TO_DEG(relative_bearing_rad));
        p_output->source_confidence = localizer.source_confidence;
        p_output->source_position_valid = TRUE;
        /* 以前の交点を保持しているだけの状態を「現在の音源方位」として
         * 可視化・操舵へ渡さない。今回の方位線の幾何が不成立なら、位置推定は
         * 参考履歴にとどめ、リアルタイムDoAと混同しない。 */
        p_output->navigation_target_valid = target_fresh && p_input->pose_valid &&
            p_output->localization_geometry_valid &&
            (range_mm <= (float) CPU0_SOUND_LOCALIZATION_MAX_RANGE_MM);
    }

    H const bearing_abs = (p_output->source_bearing_deg < 0) ?
        (H) -p_output->source_bearing_deg : p_output->source_bearing_deg;
    BOOL const bearing_supported = (bearing_abs <= (H) CPU0_SOUND_ARRIVAL_BEARING_DEG) ||
        (p_output->source_confidence >= CPU0_SOUND_ARRIVAL_STRONG_CONFIDENCE);
    p_output->arrival_candidate = p_output->navigation_target_valid &&
        p_output->localization_geometry_valid && p_input->sound_valid &&
        (p_output->source_range_mm <= CPU0_SOUND_ARRIVAL_RANGE_MM) &&
        (p_output->source_confidence >= CPU0_SOUND_ARRIVAL_MIN_CONFIDENCE) && bearing_supported;

    if (!target_fresh) {
        localizer.arrival_state = CPU0_SOUND_ARRIVAL_SEARCH;
        localizer.arrival_confirm_count = 0U;
    } else if (CPU0_SOUND_ARRIVAL_ARRIVED != localizer.arrival_state) {
        if (CPU0_SOUND_ARRIVAL_VERIFY == localizer.arrival_state) {
            if (p_input->new_observation && !p_output->arrival_candidate) {
                localizer.arrival_state = CPU0_SOUND_ARRIVAL_TRACKING;
                localizer.arrival_confirm_count = 0U;
            } else if (p_input->new_observation && p_output->arrival_candidate &&
                       (localizer.arrival_confirm_count < 255U)) {
                localizer.arrival_confirm_count++;
            }
            if (p_output->arrival_candidate &&
                (localizer.arrival_confirm_count >= CPU0_SOUND_ARRIVAL_CONFIRM_COUNT) &&
                ((p_input->now_ms - localizer.arrival_started_ms) >= CPU0_SOUND_ARRIVAL_VERIFY_MS)) {
                localizer.arrival_state = CPU0_SOUND_ARRIVAL_ARRIVED;
            }
        } else if (p_input->new_observation && p_output->arrival_candidate) {
            localizer.arrival_state = CPU0_SOUND_ARRIVAL_VERIFY;
            localizer.arrival_started_ms = p_input->now_ms;
            localizer.arrival_confirm_count = 1U;
        } else {
            localizer.arrival_state = CPU0_SOUND_ARRIVAL_TRACKING;
        }
    }

    p_output->arrival_state = localizer.arrival_state;
    p_output->arrival_confirm_count = localizer.arrival_confirm_count;
}
