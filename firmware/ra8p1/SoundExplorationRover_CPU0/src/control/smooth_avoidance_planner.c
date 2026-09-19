/** =================================================================*
 * @file   smooth_avoidance_planner.c
 * @brief  曲率制限付き滑らか障害物回避プランナ実装（ポテンシャル法純関数）
 * ================================================================= */
#include "smooth_avoidance_planner.h"
#include <math.h>

#ifndef M_PI
#define M_PI (3.14159265358979323846f)
#endif

#define DEG_TO_RAD(d) ((d) * (float)(M_PI / 180.0f))
#define RAD_TO_DEG(r) ((r) * (float)(180.0f / M_PI))

static float clampf(float val, float min_val, float max_val) {
    if (val < min_val) {
        return min_val;
    }
    if (val > max_val) {
        return max_val;
    }
    return val;
}

static float rate_limit(float target, float current, float max_change) {
    float const delta = target - current;
    if (delta > max_change) {
        return current + max_change;
    }
    if (delta < -max_change) {
        return current - max_change;
    }
    return target;
}

void smooth_avoidance_plan(const smooth_avoidance_input_t * p_input,
                           smooth_avoidance_output_t * p_output) {
    if ((NULL == p_input) || (NULL == p_output)) {
        return;
    }

    float const dt = (p_input->dt_sec > 0.001f) ? p_input->dt_sec : 0.05f;
    float const max_steer_step = SMOOTH_PLANNER_MAX_STEER_RATE_DPS * dt;
    float const max_accel_step = SMOOTH_PLANNER_MAX_ACCEL_PER_SEC * dt;

    /* 有効ToFチャネルの確認と幾何クリアランス計算 */
    bool any_valid = false;
    bool hard_stop_triggered = false;
    float d[3];

    /* 各ToFのオフセット (PLAN.md 0-D-1) */
    static const float sensor_x[3] = {
        SMOOTH_PLANNER_TOF_LEFT_X_MM,
        SMOOTH_PLANNER_TOF_CENTER_X_MM,
        SMOOTH_PLANNER_TOF_RIGHT_X_MM
    };
    static const float sensor_y[3] = {
        SMOOTH_PLANNER_TOF_LEFT_Y_MM,
        SMOOTH_PLANNER_TOF_CENTER_Y_MM,
        SMOOTH_PLANNER_TOF_RIGHT_Y_MM
    };

    for (int i = 0; i < 3; i++) {
        if (p_input->tof_valid[i]) {
            any_valid = true;
            d[i] = p_input->tof_distance_mm[i];
            /* 250mm未満の近接障害物は即時ハード停止 (I1) */
            if (d[i] < SMOOTH_PLANNER_HARD_STOP_MM) {
                hard_stop_triggered = true;
            }
        } else {
            /* 無効時は遠方障害物なしとみなす (A-3: 0mmを入れない) */
            d[i] = SMOOTH_PLANNER_FAR_DISTANCE_MM;
        }
    }

    /* 全センサ無効時は安全停止 (S6) */
    if (!any_valid) {
        hard_stop_triggered = true;
    }

    /* 幾何クリアランス記録 (車体中心原点からの距離) */
    p_output->left_clearance_mm = sqrtf((sensor_x[0] + d[0]) * (sensor_x[0] + d[0]) + sensor_y[0] * sensor_y[0]);
    p_output->center_clearance_mm = d[1];
    p_output->right_clearance_mm = sqrtf((sensor_x[2] + d[2]) * (sensor_x[2] + d[2]) + sensor_y[2] * sensor_y[2]);

    if (hard_stop_triggered) {
        p_output->is_blocked = true;
        p_output->speed_scale = 0.0f;
        /* 停止時も操舵角は急変させずレート制限を適用して維持 */
        p_output->steering_deg = rate_limit(p_input->current_steering_deg,
                                            p_input->current_steering_deg, max_steer_step);
        return;
    }

    p_output->is_blocked = false;

    /* 1. 引力ベクトル (Goal Attraction) */
    /* 目標方位角 [-180, +180] */
    float const target_rad = DEG_TO_RAD(p_input->target_heading_deg);
    /* 後方（|angle| > 90°）の場合でも前進しながら旋回する (S8) */
    float att_x = cosf(target_rad);
    float att_y = sinf(target_rad);
    if (att_x < 0.1f) {
        /* 後方または真横目標時も前進ベクトルを確保 */
        att_x = 0.1f;
        att_y = (target_rad >= 0.0f) ? 1.0f : -1.0f;
    }

    /* 2. 斥力ベクトル (Obstacle Repulsion) */
    float rep_x = 0.0f;
    float rep_y = 0.0f;
    float const d_inf = SMOOTH_PLANNER_INFLUENCE_DISTANCE_MM;
    float const d_min = SMOOTH_PLANNER_HARD_STOP_MM;

    /* 左ToF (y = -90mm): 近づくと右(+y)へ押す */
    if (p_input->tof_valid[0] && (d[0] < d_inf)) {
        float const strength = (d_inf - d[0]) / (d_inf - d_min);
        rep_y += strength * 1.5f;   /* 右への斥力 */
        rep_x -= strength * 0.5f;   /* 減速方向 */
    }

    /* 右ToF (y = +90mm): 近づくと左(-y)へ押す */
    if (p_input->tof_valid[2] && (d[2] < d_inf)) {
        float const strength = (d_inf - d[2]) / (d_inf - d_min);
        rep_y -= strength * 1.5f;   /* 左への斥力 */
        rep_x -= strength * 0.5f;   /* 減速方向 */
    }

    /* 中央ToF (y = 0mm): 正面障害物は空いている側へ大きく旋回を誘起 */
    if (p_input->tof_valid[1] && (d[1] < d_inf)) {
        float const strength = (d_inf - d[1]) / (d_inf - d_min);
        rep_x -= strength * 1.8f;   /* 前方からの強い減速・後押し */

        /* 左右クリアランス差による側方回避モーメント */
        float const left_d = p_input->tof_valid[0] ? d[0] : SMOOTH_PLANNER_FAR_DISTANCE_MM;
        float const right_d = p_input->tof_valid[2] ? d[2] : SMOOTH_PLANNER_FAR_DISTANCE_MM;
        float side_bias = 0.0f;

        if (fabsf(left_d - right_d) > 20.0f) {
            /* 空いている側へ旋回 (右が広いなら+y、左が広いなら-y) */
            side_bias = (right_d > left_d) ? 1.0f : -1.0f;
        } else {
            /* 左右ほぼ均等の正面壁: 目標方位の側、または直近操舵方向へ回避 */
            if (fabsf(p_input->target_heading_deg) > 5.0f) {
                side_bias = (p_input->target_heading_deg > 0.0f) ? 1.0f : -1.0f;
            } else if (fabsf(p_input->current_steering_deg) > 1.0f) {
                side_bias = (p_input->current_steering_deg > 0.0f) ? 1.0f : -1.0f;
            } else {
                side_bias = 1.0f; /* 既定で右回避 */
            }
        }
        rep_y += strength * 1.5f * side_bias;
    }

    /* 3. 力の合成 */
    float const total_x = att_x + rep_x;
    float const total_y = att_y + rep_y;

    /* 目標操舵角の算出 */
    float target_steer_deg = 0.0f;
    if (total_x > 0.01f) {
        target_steer_deg = RAD_TO_DEG(atan2f(total_y, total_x));
    } else {
        /* 後方へ押されている場合は最大横旋回 */
        target_steer_deg = (total_y >= 0.0f) ? SMOOTH_PLANNER_MAX_STEERING_DEG : -SMOOTH_PLANNER_MAX_STEERING_DEG;
    }

    /* クランプ: [-45, +45] (I2) */
    target_steer_deg = clampf(target_steer_deg,
                              -SMOOTH_PLANNER_MAX_STEERING_DEG,
                              SMOOTH_PLANNER_MAX_STEERING_DEG);

    /* レート制限 (I3) */
    p_output->steering_deg = rate_limit(target_steer_deg,
                                        p_input->current_steering_deg,
                                        max_steer_step);

    /* 4. 速度スケール算出 */
    /* 前方最小距離による基本速度 */
    float min_front_d = SMOOTH_PLANNER_FAR_DISTANCE_MM;
    for (int i = 0; i < 3; i++) {
        if (p_input->tof_valid[i] && (d[i] < min_front_d)) {
            min_front_d = d[i];
        }
    }

    float target_speed = 1.0f;
    if (min_front_d >= SMOOTH_PLANNER_RECOVER_CLEAR_MM) {
        target_speed = 1.0f;
    } else {
        /* 250mm〜650mmで線形減速 */
        float const ratio = (min_front_d - SMOOTH_PLANNER_HARD_STOP_MM) /
                            (SMOOTH_PLANNER_RECOVER_CLEAR_MM - SMOOTH_PLANNER_HARD_STOP_MM);
        target_speed = SMOOTH_PLANNER_MIN_SPEED_SCALE +
                       (1.0f - SMOOTH_PLANNER_MIN_SPEED_SCALE) * clampf(ratio, 0.0f, 1.0f);
    }

    /* 急旋回時の減速 (スリップ防止) */
    float const abs_steer = fabsf(p_output->steering_deg);
    if (abs_steer > 20.0f) {
        float const turn_slowdown = 1.0f - 0.35f * ((abs_steer - 20.0f) / 25.0f);
        target_speed *= turn_slowdown;
    }

    /* 走行時デッドゾーン回避: 停止意図でない限り0を通さない (4-3-2, I9) */
    target_speed = clampf(target_speed, SMOOTH_PLANNER_MIN_SPEED_SCALE, 1.0f);

    /* 速度レート制限 (I4) */
    p_output->speed_scale = rate_limit(target_speed,
                                       p_input->current_speed_scale,
                                       max_accel_step);
}
