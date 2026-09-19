/** =================================================================*
 * @file   odometry.h
 * @brief  車輪エンコーダとジャイロによるオドメトリ計算・位置推定
 * ================================================================= */
#ifndef SEROV_CPU0_SERVICE_ODOMETRY_H
#define SEROV_CPU0_SERVICE_ODOMETRY_H

#include <tk/tkernel.h>
#include "../../../common/ipc_message.h"

/* 車体幾何・エンコーダ諸元 */
#define ODOMETRY_WHEEL_DIAMETER_MM          (108.0f)
#define ODOMETRY_WHEEL_CIRCUMFERENCE_MM     (339.2920f)
#define ODOMETRY_WHEEL_ENCODER_COUNTS       (702.0f)
#define ODOMETRY_MM_PER_COUNT               (0.48332194f)          /* 339.292 / 702.0 */
#define ODOMETRY_TRACK_WIDTH_MM             (260.0f)               /* 実効トレッド幅[mm] */

/* ジャイロ自動バイアス校正パラメータ */
#define ODOMETRY_GYRO_STATIONARY_RPM_LIMIT  (20)                   /* 静止判定RPM上限(0.1 RPM単位: 2.0 RPM) */
#define ODOMETRY_GYRO_BIAS_ALPHA            (0.02f)                /* 静止時バイアス平滑化係数 */
#define ODOMETRY_COMPLEMENTARY_ALPHA        (0.95f)                /* 相補フィルタ: ジャイロ重み */

/* 推定位置・姿勢構造体（sensor_snapshot_tとは独立） */
typedef struct st_odometry_pose {
    W    x_mm;                                                     /**< 車体中心推定X座標[mm] */
    W    y_mm;                                                     /**< 車体中心推定Y座標[mm] */
    W    theta_mrad;                                               /**< 推定方位角[-pi, pi][mrad] */
    H    theta_deg_x10;                                            /**< 推定方位角[0.1 deg] */
    UW   total_distance_mm;                                        /**< 累積走行距離[mm] */
    H    linear_speed_mm_s;                                        /**< 並進速度[mm/s] */
    H    angular_speed_mrad_s;                                     /**< 旋回角速度[mrad/s] */
    W    left_encoder_total;                                       /**< 左累積アンラップカウント */
    W    right_encoder_total;                                      /**< 右累積アンラップカウント */
    UW   timestamp_ms;                                             /**< CPU1タイムスタンプ[ms] */
    BOOL valid;                                                    /**< 有効フラグ（初期化・同期済み） */
} odometry_pose_t;

/* オドメトリ内部コンテキスト（純関数更新用） */
typedef struct st_odometry_context {
    float x_mm;
    float y_mm;
    float theta_rad;
    float total_distance_mm;
    float linear_speed_mm_s;
    float angular_speed_rad_s;
    float gyro_bias_dps;

    UW    prev_left_count;
    UW    prev_right_count;
    UW    prev_uptime_ms;
    W     left_unwrapped;
    W     right_unwrapped;

    BOOL  initialized;
    BOOL  gyro_calibrated;
} odometry_context_t;

/* 純関数API */
EXPORT void odometry_init(odometry_context_t * p_ctx);
EXPORT void odometry_update(odometry_context_t * p_ctx,
                            UW left_count_24,
                            UW right_count_24,
                            UW uptime_ms_24,
                            H gyro_z_dps_x10,
                            H left_rpm_x10,
                            H right_rpm_x10);
EXPORT void odometry_get_pose(const odometry_context_t * p_ctx, odometry_pose_t * p_pose);

/* システム共有サービスAPI（スレッドセーフ） */
EXPORT void odometry_service_init(void);
EXPORT void odometry_service_update(const actuator_status_t * p_status, H gyro_z_dps_x10);
EXPORT void odometry_service_get_pose(odometry_pose_t * p_pose);

#endif /* SEROV_CPU0_SERVICE_ODOMETRY_H */
