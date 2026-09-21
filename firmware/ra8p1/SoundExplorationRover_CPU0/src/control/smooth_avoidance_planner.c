/** =================================================================*
 * @file   smooth_avoidance_planner.c
 * @brief  曲率制限付き滑らか障害物回避プランナ実装（ポテンシャル法純関数）
 * ================================================================= */
#include "smooth_avoidance_planner.h"                       /* 滑らか回避プランナAPI */
#include <math.h>                                           /* 三角関数・平方根 */

#ifndef M_PI
#define M_PI                               (3.14159265358979323846f) /**< 滑らか回避の角度計算用円周率 */
#endif

#define DEG_TO_RAD(d) ((d) * (float)(M_PI / 180.0f))        /**< 回避操舵角をラジアンへ変換 */
#define RAD_TO_DEG(r) ((r) * (float)(180.0f / M_PI))        /**< 回避計算結果を操舵角へ変換 */

/**< ToF各センサーの車体中心基準Xオフセット[mm] */
LOCAL float const smooth_planner_sensor_x[3] = {
    SMOOTH_PLANNER_TOF_LEFT_X_MM,
    SMOOTH_PLANNER_TOF_CENTER_X_MM,
    SMOOTH_PLANNER_TOF_RIGHT_X_MM,
};
/**< ToF各センサーの車体中心基準Yオフセット[mm] */
LOCAL float const smooth_planner_sensor_y[3] = {
    SMOOTH_PLANNER_TOF_LEFT_Y_MM,
    SMOOTH_PLANNER_TOF_CENTER_Y_MM,
    SMOOTH_PLANNER_TOF_RIGHT_Y_MM,
};

/** =================================================================*
 * @brief  浮動小数点値を範囲内へクランプ
 * @param[in] val 入力値
 * @param[in] min_val 下限
 * @param[in] max_val 上限
 * @return クランプ後の値
 * ================================================================= */
LOCAL float smooth_planner_clampf(float val, float min_val, float max_val) {
    if (val < min_val) {
        return min_val;
    }
    if (val > max_val) {
        return max_val;
    }
    return val;
}

/** =================================================================*
 * @brief  変化量を制限して目標値へ近づける
 * @param[in] target 目標値
 * @param[in] current 現在値
 * @param[in] max_change 1周期の最大変化量
 * @return 制限適用後の値
 * ================================================================= */
LOCAL float smooth_planner_rate_limit(float target, float current, float max_change) {
    float const delta = target - current;
    if (delta > max_change) {
        return current + max_change;
    }
    if (delta < -max_change) {
        return current - max_change;
    }
    return target;
}

/** =================================================================*
 * @brief  滑らかな操舵角と速度スケールを算出
 * @details ToFの斥力と目標方位の引力を合成し、操舵角・速度の変化率を制限する。
 *          全チャネル無効時だけ安全停止し、近接距離だけでは停止しない。
 * @param[in] p_input 観測値と直前の出力
 * @param[out] p_output 回避計画結果
 * ================================================================= */
EXPORT void smooth_avoidance_plan(const smooth_avoidance_input_t * p_input,
                                  smooth_avoidance_output_t * p_output) {
    if ((NULL == p_input) || (NULL == p_output)) {
        return;
    }

    float const dt = (p_input->dt_sec > 0.001f) ? p_input->dt_sec : 0.05f;
    float const max_steer_step = SMOOTH_PLANNER_MAX_STEER_RATE_DPS * dt;
    float const max_accel_step = SMOOTH_PLANNER_MAX_ACCEL_PER_SEC * dt;

    /* 有効ToFチャネルの確認と幾何クリアランス計算 */
    BOOL any_valid = FALSE;
    float d[3];

    for (UW i = 0U; i < 3U; i++) {
        if (p_input->tof_valid[i]) {
            any_valid = TRUE;
            d[i] = p_input->tof_distance_mm[i];
        } else {
            /* 無効時は遠方障害物なしとみなす (A-3: 0mmを入れない) */
            d[i] = SMOOTH_PLANNER_FAR_DISTANCE_MM;
        }
    }

    /* 全センサ無効時は安全停止 (S6) */
    if (!any_valid) {
        p_output->is_blocked = TRUE;
        p_output->speed_scale = 0.0f;
        /* 停止時も操舵角は急変させずレート制限を適用して維持 */
        p_output->steering_deg = smooth_planner_rate_limit(p_input->current_steering_deg,
                                                           p_input->current_steering_deg,
                                                           max_steer_step);
        return;
    }

    /* 幾何クリアランス記録 (車体中心原点からの距離) */
    p_output->left_clearance_mm =
        sqrtf((smooth_planner_sensor_x[0] + d[0]) * (smooth_planner_sensor_x[0] + d[0]) +
              smooth_planner_sensor_y[0] * smooth_planner_sensor_y[0]);
    p_output->center_clearance_mm = d[1];
    p_output->right_clearance_mm =
        sqrtf((smooth_planner_sensor_x[2] + d[2]) * (smooth_planner_sensor_x[2] + d[2]) +
              smooth_planner_sensor_y[2] * smooth_planner_sensor_y[2]);

    p_output->is_blocked = FALSE;

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
    /* 近接しても停止させず、斥力だけを最大値へ飽和させる。 */
    float const d_min = 0.0f;

    /* 左ToF (y = -90mm): 近づくと右(+y)へ押す */
    if (p_input->tof_valid[0] && (d[0] < d_inf)) {
        float const strength = (d_inf - d[0]) / (d_inf - d_min);
        /* 左側障害物は右方向へ押し、前方成分で減速する。 */
        rep_y += strength * 1.5f;
        rep_x -= strength * 0.5f;
    }

    /* 右ToF (y = +90mm): 近づくと左(-y)へ押す */
    if (p_input->tof_valid[2] && (d[2] < d_inf)) {
        float const strength = (d_inf - d[2]) / (d_inf - d_min);
        /* 右側障害物は左方向へ押し、前方成分で減速する。 */
        rep_y -= strength * 1.5f;
        rep_x -= strength * 0.5f;
    }

    /* 中央ToF (y = 0mm): 正面障害物は空いている側へ大きく旋回を誘起 */
    if (p_input->tof_valid[1] && (d[1] < d_inf)) {
        float const strength = (d_inf - d[1]) / (d_inf - d_min);
        /* 正面障害物は強い減速成分として加える。 */
        rep_x -= strength * 1.8f;

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
                /* 左右差がない場合は、既定の右回避を選ぶ。 */
                side_bias = 1.0f;
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
    target_steer_deg = smooth_planner_clampf(target_steer_deg,
                                             -SMOOTH_PLANNER_MAX_STEERING_DEG,
                                             SMOOTH_PLANNER_MAX_STEERING_DEG);

    /* レート制限 (I3) */
    p_output->steering_deg = smooth_planner_rate_limit(target_steer_deg,
                                                       p_input->current_steering_deg,
                                                       max_steer_step);

    /* 4. 速度スケール算出 */
    /* 前方最小距離による基本速度 */
    float min_front_d = SMOOTH_PLANNER_FAR_DISTANCE_MM;
    for (UW i = 0U; i < 3U; i++) {
        if (p_input->tof_valid[i] && (d[i] < min_front_d)) {
            min_front_d = d[i];
        }
    }

    float target_speed = 1.0f;
    if (min_front_d >= SMOOTH_PLANNER_RECOVER_CLEAR_MM) {
        target_speed = 1.0f;
    } else {
        /* 0mm〜650mmで線形減速し、最小走行速度を維持する。 */
        float const ratio = min_front_d / SMOOTH_PLANNER_RECOVER_CLEAR_MM;
        target_speed = SMOOTH_PLANNER_MIN_SPEED_SCALE +
                       (1.0f - SMOOTH_PLANNER_MIN_SPEED_SCALE) *
                       smooth_planner_clampf(ratio, 0.0f, 1.0f);
    }

    /* 急旋回時の減速 (スリップ防止) */
    float const abs_steer = fabsf(p_output->steering_deg);
    if (abs_steer > 20.0f) {
        float const turn_slowdown = 1.0f - 0.35f * ((abs_steer - 20.0f) / 25.0f);
        target_speed *= turn_slowdown;
    }

    /* 走行時デッドゾーン回避: 停止意図でない限り0を通さない (4-3-2, I9) */
    target_speed = smooth_planner_clampf(target_speed, SMOOTH_PLANNER_MIN_SPEED_SCALE, 1.0f);

    /* 速度レート制限 (I4) */
    p_output->speed_scale = smooth_planner_rate_limit(target_speed,
                                                      p_input->current_speed_scale,
                                                      max_accel_step);
}
