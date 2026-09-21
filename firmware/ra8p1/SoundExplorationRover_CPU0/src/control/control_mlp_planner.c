/** =================================================================*
 * @file   control_mlp_planner.c
 * @brief  TFLM int8制御MLPによる障害物回避プランナ実装
 * ================================================================= */
#include "control_mlp_planner.h"                            /* プランナ公開API */
#include "control_mlp_model.h"                              /* int8モデル定数 */
#include "config/control_config.h"                          /* センサー軸・制御定数 */
#include "ai/tflm_runtime.h"                                /* TFLMランタイムAPI */
#include <math.h>                                           /* 三角関数・丸め */
#include <string.h>                                         /* メモリ操作 */

#ifndef M_PI
#define M_PI                               (3.14159265358979323846f)
#endif

#define DEG_TO_RAD(d)                      ((d) * (float)(M_PI / 180.0f))

LOCAL BOOL s_initialized = FALSE;                           /**< TFLM推論器準備完了フラグ */
LOCAL tflm_runtime_info_t s_model_info;                     /**< モデル入出力量子化情報 */
LOCAL float s_current_steering_deg = 0.0f;                  /**< 直前の決定操舵角 [deg] */
LOCAL float s_current_speed_scale = 0.0f;                   /**< 直前の決定速度スケール [0.0, 1.0] */

/** =================================================================*
 * @brief  浮動小数点クランプ関数
 * ================================================================= */
LOCAL float control_mlp_clampf(float val, float min_val, float max_val) {
    if (val < min_val) {
        return min_val;
    }
    if (val > max_val) {
        return max_val;
    }
    return val;
}

/** =================================================================*
 * @brief  変化量制限（スルーレートリミッタ）
 * ================================================================= */
LOCAL float control_mlp_rate_limit(float target, float current, float max_change) {
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
 * @brief  制御MLPプランナ初期化
 * @return 成功時TRUE、初期化失敗時FALSE
 * ================================================================= */
EXPORT BOOL control_mlp_planner_init(void) {
    s_initialized = FALSE;
    s_current_steering_deg = 0.0f;
    s_current_speed_scale = 0.0f;

    INT const init_err = tflm_runtime_init(g_control_mlp_model, g_control_mlp_model_len);
    if (TFLM_RUNTIME_OK != init_err) {
        return FALSE;
    }

    INT const info_err = tflm_runtime_get_info(&s_model_info);
    if ((TFLM_RUNTIME_OK != info_err) ||
        (s_model_info.input_bytes != CONTROL_MLP_INPUT_DIMENSION) ||
        (s_model_info.output_bytes != CONTROL_MLP_OUTPUT_DIMENSION) ||
        !(s_model_info.input_scale > 0.0f) ||
        !(s_model_info.output_scale > 0.0f)) {
        tflm_runtime_reset();
        return FALSE;
    }

    s_initialized = TRUE;
    return TRUE;
}

/** =================================================================*
 * @brief  プランナ状態リセット
 * ================================================================= */
EXPORT void control_mlp_planner_reset(void) {
    s_initialized = FALSE;
    s_current_steering_deg = 0.0f;
    s_current_speed_scale = 0.0f;
    tflm_runtime_reset();
}

/** =================================================================*
 * @brief  初期化状態取得
 * ================================================================= */
EXPORT BOOL control_mlp_planner_is_ready(void) {
    return s_initialized;
}

/** =================================================================*
 * @brief  1周期（通常100ms）のMLP回避計画ステップ
 * @param[in] p_snapshot センサースナップショット
 * @param[in] target_heading_deg 目標方位角 [-180, +180]
 * @param[out] p_output 算出結果
 * ================================================================= */
EXPORT void control_mlp_planner_step(const sensor_snapshot_t * p_snapshot,
                                     float target_heading_deg,
                                     control_mlp_output_t * p_output) {
    if (NULL == p_output) {
        return;
    }

    /* デフォルトは安全停止・フォールバック要求 */
    p_output->steering_deg = s_current_steering_deg;
    p_output->speed_scale = 0.0f;
    p_output->is_blocked = TRUE;
    p_output->emergency_stop = FALSE;
    p_output->fallback_required = TRUE;

    if (!s_initialized || (NULL == p_snapshot) || !p_snapshot->initialized) {
        return;
    }

    /* ToF 3台の有効状態取得 */
    BOOL tof_valid[CPU0_SENSOR_TOF_COUNT];
    tof_valid[CPU0_TOF_LEFT]   = (0U != (p_snapshot->valid_flags & CPU0_SENSOR_VALID_TOF_LEFT));
    tof_valid[CPU0_TOF_CENTER] = (0U != (p_snapshot->valid_flags & CPU0_SENSOR_VALID_TOF_CENTER));
    tof_valid[CPU0_TOF_RIGHT]  = (0U != (p_snapshot->valid_flags & CPU0_SENSOR_VALID_TOF_RIGHT));

    /* 全ToF無効なら即座にフォールバック */
    if (!tof_valid[CPU0_TOF_LEFT] && !tof_valid[CPU0_TOF_CENTER] && !tof_valid[CPU0_TOF_RIGHT]) {
        return;
    }

    /* 近接距離による即時停止はここでは行わない。正面衝突候補の連続確認と
     * 回避旋回は、ToF 3眼を同時に扱うルールベース制御へ委譲する。 */
    float min_tof_mm = CONTROL_MLP_FAR_DISTANCE_MM;
    for (UW i = 0U; i < CPU0_SENSOR_TOF_COUNT; i++) {
        if (tof_valid[i]) {
            float const dist_mm = (float) p_snapshot->tof_distance_mm[i];
            if (dist_mm < min_tof_mm) {
                min_tof_mm = dist_mm;
            }
        }
    }

    /* 10次元入力特徴量ベクトルの構築 */
    float input_features[CONTROL_MLP_INPUT_DIMENSION];

    /* [0..2]: ToF L, C, R 距離（0〜4000mmを0.0〜1.0へ正規化、無効時は1.0） */
    for (UW i = 0U; i < CPU0_SENSOR_TOF_COUNT; i++) {
        if (tof_valid[i]) {
            input_features[i] = control_mlp_clampf(
                (float) p_snapshot->tof_distance_mm[i] / CONTROL_MLP_FAR_DISTANCE_MM, 0.0f, 1.0f);
            input_features[3U + i] = 1.0f; /* [3..5]: 有効フラグ 1.0 */
        } else {
            input_features[i] = 1.0f;
            input_features[3U + i] = 0.0f; /* [3..5]: 有効フラグ 0.0 */
        }
    }

    /* [6..7]: 目標音源方位 sin(theta), cos(theta) */
    float const target_rad = DEG_TO_RAD(target_heading_deg);
    input_features[6U] = sinf(target_rad);
    input_features[7U] = cosf(target_rad);

    /* [8]: 現在の速度スケール [0.0, 1.0] */
    input_features[8U] = control_mlp_clampf(s_current_speed_scale, 0.0f, 1.0f);

    /* [9]: ヨーレート（正規化: dps / 50.0 を [-1.0, 1.0] へクランプ） */
    float const yaw_rate_dps = (float) p_snapshot->gyro_dps_x10[CPU0_SENSOR_YAW_AXIS] / 10.0f;
    input_features[9U] = control_mlp_clampf(yaw_rate_dps / 50.0f, -1.0f, 1.0f);

    /* int8 量子化 */
    B quantized_input[CONTROL_MLP_INPUT_DIMENSION];
    for (UW i = 0U; i < CONTROL_MLP_INPUT_DIMENSION; i++) {
        float const q_val = roundf(input_features[i] / s_model_info.input_scale) +
                            (float) s_model_info.input_zero_point;
        quantized_input[i] = (B) control_mlp_clampf(q_val, -128.0f, 127.0f);
    }

    /* int8 推論実行 */
    B quantized_output[CONTROL_MLP_OUTPUT_DIMENSION];
    INT const invoke_err = tflm_runtime_invoke(quantized_input, sizeof(quantized_input),
                                              quantized_output, sizeof(quantized_output));
    if (TFLM_RUNTIME_OK != invoke_err) {
        p_output->fallback_required = TRUE;
        return;
    }

    /* int8 出力の物理量逆量子化 */
    float const steer_norm = ((float) quantized_output[0] - (float) s_model_info.output_zero_point) *
                             s_model_info.output_scale;
    float const speed_norm = ((float) quantized_output[1] - (float) s_model_info.output_zero_point) *
                             s_model_info.output_scale;

    float raw_steer_deg = control_mlp_clampf(steer_norm * CONTROL_MLP_MAX_STEERING_DEG,
                                             -CONTROL_MLP_MAX_STEERING_DEG,
                                             CONTROL_MLP_MAX_STEERING_DEG);
    float raw_speed = control_mlp_clampf(speed_norm, 0.0f, 1.0f);

    /* 開けた直進空間での量子化残留不感帯（850mm以上かつ目標直進時） */
    if ((min_tof_mm >= 850.0f) && (fabsf(target_heading_deg) < 1.0f) && (fabsf(raw_steer_deg) < 4.0f)) {
        raw_steer_deg = 0.0f;
    }

    /* スルーレート制限適用（100ms周期） */
    float const max_steer_step = CONTROL_MLP_MAX_STEER_RATE_DPS * CONTROL_MLP_STEP_DT_SEC;
    float const max_accel_step = CONTROL_MLP_MAX_ACCEL_PER_SEC * CONTROL_MLP_STEP_DT_SEC;

    s_current_steering_deg = control_mlp_rate_limit(raw_steer_deg, s_current_steering_deg, max_steer_step);
    s_current_speed_scale  = control_mlp_rate_limit(raw_speed, s_current_speed_scale, max_accel_step);

    p_output->steering_deg      = s_current_steering_deg;
    p_output->speed_scale       = s_current_speed_scale;
    p_output->is_blocked        = (s_current_speed_scale <= 0.01f);
    p_output->emergency_stop    = FALSE;
    p_output->fallback_required = FALSE;
}
