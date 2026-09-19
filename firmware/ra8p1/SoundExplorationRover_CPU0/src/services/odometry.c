/** =================================================================*
 * @file   odometry.c
 * @brief  車輪エンコーダとジャイロによるオドメトリ計算・位置推定実装
 * ================================================================= */
#include "services/odometry.h"
#include "../../../common/ipc_message.h"
#include "hal_data.h"
#include <math.h>
#include <stdlib.h>

#ifndef M_PI
#define M_PI                               (3.14159265358979323846f)
#endif

#define DEG_TO_RAD(d)                      ((d) * (M_PI / 180.0f))
#define RAD_TO_DEG(r)                      ((r) * (180.0f / M_PI))

LOCAL odometry_context_t s_odometry_ctx;
LOCAL odometry_pose_t    s_latest_pose;
LOCAL BOOL               s_service_initialized = FALSE;

/** =================================================================*
 * @brief  24 bit値の符号拡張付き剰余差分計算
 * @param[in] curr 最新24 bit値
 * @param[in] prev 前回24 bit値
 * @return 符号付き差分
 * ================================================================= */
Inline W odometry_diff_24(UW curr, UW prev) {
    W const diff = (W) (((curr - prev) & 0x00FFFFFFUL) << 8);
    return diff >> 8;
}

/** =================================================================*
 * @brief  オドメトリコンテキスト初期化（純関数）
 * @param[out] p_ctx 初期化対象コンテキスト
 * ================================================================= */
EXPORT void odometry_init(odometry_context_t * p_ctx) {
    if (NULL == p_ctx) {
        return;
    }
    p_ctx->x_mm = 0.0f;
    p_ctx->y_mm = 0.0f;
    p_ctx->theta_rad = 0.0f;
    p_ctx->total_distance_mm = 0.0f;
    p_ctx->linear_speed_mm_s = 0.0f;
    p_ctx->angular_speed_rad_s = 0.0f;
    p_ctx->gyro_bias_dps = 0.0f;

    p_ctx->prev_left_count = 0U;
    p_ctx->prev_right_count = 0U;
    p_ctx->prev_uptime_ms = 0U;
    p_ctx->left_unwrapped = 0;
    p_ctx->right_unwrapped = 0;

    p_ctx->initialized = FALSE;
    p_ctx->gyro_calibrated = FALSE;
}

/** =================================================================*
 * @brief  オドメトリ実時間更新（純関数）
 * @param[in,out] p_ctx オドメトリコンテキスト
 * @param[in] left_count_24  左エンコーダ24 bit累積カウント
 * @param[in] right_count_24 右エンコーダ24 bit累積カウント
 * @param[in] uptime_ms_24   CPU1稼働時間24 bit[ms]
 * @param[in] gyro_z_dps_x10 BMI270 Z軸角速度[0.1 dps]
 * @param[in] left_rpm_x10   左車輪実測RPM[0.1 rpm]
 * @param[in] right_rpm_x10  右車輪実測RPM[0.1 rpm]
 * ================================================================= */
EXPORT void odometry_update(odometry_context_t * p_ctx,
                            UW left_count_24,
                            UW right_count_24,
                            UW uptime_ms_24,
                            H gyro_z_dps_x10,
                            H left_rpm_x10,
                            H right_rpm_x10) {
    if (NULL == p_ctx) {
        return;
    }

    if (!p_ctx->initialized) {
        p_ctx->prev_left_count = left_count_24 & ACTUATOR_IPC_PAYLOAD_MASK;
        p_ctx->prev_right_count = right_count_24 & ACTUATOR_IPC_PAYLOAD_MASK;
        p_ctx->prev_uptime_ms = uptime_ms_24 & ACTUATOR_IPC_PAYLOAD_MASK;
        p_ctx->initialized = TRUE;
        return;
    }

    UW const curr_left = left_count_24 & ACTUATOR_IPC_PAYLOAD_MASK;
    UW const curr_right = right_count_24 & ACTUATOR_IPC_PAYLOAD_MASK;
    UW const curr_time = uptime_ms_24 & ACTUATOR_IPC_PAYLOAD_MASK;

    W const d_left_count = odometry_diff_24(curr_left, p_ctx->prev_left_count);
    W const d_right_count = odometry_diff_24(curr_right, p_ctx->prev_right_count);
    UW const dt_ms = (curr_time - p_ctx->prev_uptime_ms) & ACTUATOR_IPC_PAYLOAD_MASK;

    p_ctx->prev_left_count = curr_left;
    p_ctx->prev_right_count = curr_right;
    p_ctx->prev_uptime_ms = curr_time;
    p_ctx->left_unwrapped += d_left_count;
    p_ctx->right_unwrapped += d_right_count;

    /* 時刻更新がない、または異常に長い更新（1秒超）は積分をスキップ */
    if ((0U == dt_ms) || (dt_ms > 1000U)) {
        return;
    }

    float const dt_s = (float) dt_ms * 0.001f;

    /* 静止判定: RPMが小さく、カウント変化も極小 */
    BOOL const stationary = (abs((INT) left_rpm_x10) <= ODOMETRY_GYRO_STATIONARY_RPM_LIMIT) &&
                            (abs((INT) right_rpm_x10) <= ODOMETRY_GYRO_STATIONARY_RPM_LIMIT) &&
                            (abs((INT) d_left_count) <= 1) && (abs((INT) d_right_count) <= 1);

    float const raw_gyro_z_dps = (float) gyro_z_dps_x10 * 0.1f;
    if (stationary) {
        /* 静止時はジャイロバイアスを指数移動平均で校正 */
        if (!p_ctx->gyro_calibrated) {
            p_ctx->gyro_bias_dps = raw_gyro_z_dps;
            p_ctx->gyro_calibrated = TRUE;
        } else {
            p_ctx->gyro_bias_dps = ((1.0f - ODOMETRY_GYRO_BIAS_ALPHA) * p_ctx->gyro_bias_dps) +
                                   (ODOMETRY_GYRO_BIAS_ALPHA * raw_gyro_z_dps);
        }
    }

    /* 各輪の移動距離[mm] */
    float const d_left_mm = (float) d_left_count * ODOMETRY_MM_PER_COUNT;
    float const d_right_mm = (float) d_right_count * ODOMETRY_MM_PER_COUNT;
    float const d_center_mm = 0.5f * (d_left_mm + d_right_mm);

    /* 旋回角増分計算（車輪差動） */
    float const d_theta_wheel = (d_right_mm - d_left_mm) / ODOMETRY_TRACK_WIDTH_MM;

    /* 旋回角増分計算（ジャイロ） */
    float const corrected_gyro_dps = raw_gyro_z_dps - p_ctx->gyro_bias_dps;
    float const gyro_rad_s = DEG_TO_RAD(corrected_gyro_dps);
    float const d_theta_gyro = gyro_rad_s * dt_s;

    /* 相補フィルタによる方位角増分合成 */
    float d_theta;
    if (stationary) {
        d_theta = 0.0f;
    } else {
        d_theta = (ODOMETRY_COMPLEMENTARY_ALPHA * d_theta_gyro) +
                  ((1.0f - ODOMETRY_COMPLEMENTARY_ALPHA) * d_theta_wheel);
    }

    /* Runge-Kutta 2次（中点法）による座標更新 */
    float const mid_theta = p_ctx->theta_rad + (0.5f * d_theta);
    p_ctx->x_mm += d_center_mm * cosf(mid_theta);
    p_ctx->y_mm += d_center_mm * sinf(mid_theta);
    p_ctx->theta_rad += d_theta;

    /* 方位角を [-pi, pi] に正規化 */
    while (p_ctx->theta_rad > M_PI) {
        p_ctx->theta_rad -= 2.0f * M_PI;
    }
    while (p_ctx->theta_rad < -M_PI) {
        p_ctx->theta_rad += 2.0f * M_PI;
    }

    p_ctx->total_distance_mm += fabsf(d_center_mm);
    p_ctx->linear_speed_mm_s = d_center_mm / dt_s;
    p_ctx->angular_speed_rad_s = d_theta / dt_s;
}

/** =================================================================*
 * @brief  オドメトリ結果取得（純関数）
 * @param[in]  p_ctx  オドメトリコンテキスト
 * @param[out] p_pose 取得先ポーズ構造体
 * ================================================================= */
EXPORT void odometry_get_pose(const odometry_context_t * p_ctx, odometry_pose_t * p_pose) {
    if ((NULL == p_ctx) || (NULL == p_pose)) {
        return;
    }

    p_pose->x_mm = (W) roundf(p_ctx->x_mm);
    p_pose->y_mm = (W) roundf(p_ctx->y_mm);
    p_pose->theta_mrad = (W) roundf(p_ctx->theta_rad * 1000.0f);
    p_pose->theta_deg_x10 = (H) roundf(RAD_TO_DEG(p_ctx->theta_rad) * 10.0f);
    p_pose->total_distance_mm = (UW) roundf(p_ctx->total_distance_mm);
    p_pose->linear_speed_mm_s = (H) roundf(p_ctx->linear_speed_mm_s);
    p_pose->angular_speed_mrad_s = (H) roundf(p_ctx->angular_speed_rad_s * 1000.0f);
    p_pose->left_encoder_total = p_ctx->left_unwrapped;
    p_pose->right_encoder_total = p_ctx->right_unwrapped;
    p_pose->timestamp_ms = p_ctx->prev_uptime_ms;
    p_pose->valid = p_ctx->initialized;
}

/** =================================================================*
 * @brief  システムオドメトリサービス初期化
 * ================================================================= */
EXPORT void odometry_service_init(void) {
    odometry_init(&s_odometry_ctx);
    odometry_get_pose(&s_odometry_ctx, &s_latest_pose);
    s_service_initialized = TRUE;
}

/** =================================================================*
 * @brief  システムオドメトリサービス更新
 * @param[in] p_status       CPU1状態
 * @param[in] gyro_z_dps_x10 BMI270 Z軸角速度[0.1 dps]
 * ================================================================= */
EXPORT void odometry_service_update(const actuator_status_t * p_status, H gyro_z_dps_x10) {
    if (!s_service_initialized || (NULL == p_status)) {
        return;
    }

    odometry_update(&s_odometry_ctx,
                    p_status->left_encoder_count,
                    p_status->right_encoder_count,
                    p_status->status_uptime_ms,
                    gyro_z_dps_x10,
                    p_status->left_encoder_rpm_x10,
                    p_status->right_encoder_rpm_x10);

    FSP_CRITICAL_SECTION_DEFINE;
    FSP_CRITICAL_SECTION_ENTER;
    odometry_get_pose(&s_odometry_ctx, &s_latest_pose);
    FSP_CRITICAL_SECTION_EXIT;
}

/** =================================================================*
 * @brief  システムオドメトリ最新ポーズ取得（スレッドセーフ）
 * @param[out] p_pose 格納先ポーズ構造体
 * ================================================================= */
EXPORT void odometry_service_get_pose(odometry_pose_t * p_pose) {
    if (NULL == p_pose) {
        return;
    }

    FSP_CRITICAL_SECTION_DEFINE;
    FSP_CRITICAL_SECTION_ENTER;
    *p_pose = s_latest_pose;
    FSP_CRITICAL_SECTION_EXIT;
}
