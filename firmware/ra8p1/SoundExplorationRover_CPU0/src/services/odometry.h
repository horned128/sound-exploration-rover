/** =================================================================*
 * @file   odometry.h
 * @brief  車輪エンコーダとジャイロによるオドメトリ計算・位置推定
 * ================================================================= */
#ifndef SEROV_CPU0_SERVICE_ODOMETRY_H
#define SEROV_CPU0_SERVICE_ODOMETRY_H

#include <tk/tkernel.h>                                     /* μT-Kernel基本型 */
#include "../../../common/ipc_message.h"                    /* CPU間テレメトリ型 */

/* 車体幾何・エンコーダ諸元 */
#define ODOMETRY_WHEEL_DIAMETER_MM         (108.0f)         /**< odometry車輪のdiameter[mm] */
#define ODOMETRY_WHEEL_CIRCUMFERENCE_MM    (339.2920f)      /**< odometry車輪のcircumference[mm] */
#define ODOMETRY_WHEEL_ENCODER_COUNTS      (702.0f)         /**< odometry車輪エンコーダのカウント数 */
#define ODOMETRY_MM_PER_COUNT              (0.48332194f)    /**< 1countの移動量[mm] */
#define ODOMETRY_TRACK_WIDTH_MM            (260.0f)         /**< 左右車輪間の実効トレッド幅[mm] */

/* ジャイロ自動バイアス校正パラメータ */
#define ODOMETRY_GYRO_STATIONARY_RPM_LIMIT (20)             /**< bias更新を許可する停止RPM上限 */
#define ODOMETRY_GYRO_BIAS_ALPHA           (0.02f)          /**< 停止中ジャイロバイアスの平滑化係数 */
#define ODOMETRY_COMPLEMENTARY_ALPHA       (0.95f)          /**< 相補フィルタのジャイロ重み */

/**< 推定位置・姿勢を公開するオドメトリ結果 */
typedef struct st_odometry_pose {
    W    x_mm;                                              /**< 車体中心推定X座標[mm] */
    W    y_mm;                                              /**< 車体中心推定Y座標[mm] */
    W    theta_mrad;                                        /**< 推定方位角[-pi, pi][mrad] */
    H    theta_deg_x10;                                     /**< 推定方位角[0.1 deg] */
    UW   total_distance_mm;                                 /**< 累積走行距離[mm] */
    H    linear_speed_mm_s;                                 /**< 並進速度[mm/s] */
    H    angular_speed_mrad_s;                              /**< 旋回角速度[mrad/s] */
    W    left_encoder_total;                                /**< 左累積アンラップカウント */
    W    right_encoder_total;                               /**< 右累積アンラップカウント */
    UW   timestamp_ms;                                      /**< CPU1タイムスタンプ[ms] */
    BOOL valid;                                             /**< 有効フラグ（初期化・同期済み） */
} odometry_pose_t;

/**< 純関数のオドメトリ更新が保持する内部状態 */
typedef struct st_odometry_context {
    float x_mm;                                             /**< 車体中心推定X座標[mm] */
    float y_mm;                                             /**< 車体中心推定Y座標[mm] */
    float theta_rad;                                        /**< 推定方位角[rad] */
    float total_distance_mm;                                /**< 累積走行距離[mm] */
    float linear_speed_mm_s;                                /**< 並進速度[mm/s] */
    float angular_speed_rad_s;                              /**< 旋回角速度[rad/s] */
    float gyro_bias_dps;                                    /**< ジャイロZ軸バイアス[dps] */

    UW    prev_left_count;                                  /**< 前回左エンコーダカウント */
    UW    prev_right_count;                                 /**< 前回右エンコーダカウント */
    UW    prev_uptime_ms;                                   /**< 前回更新時刻[ms] */
    W     left_unwrapped;                                   /**< 左エンコーダのアンラップ値 */
    W     right_unwrapped;                                  /**< 右エンコーダのアンラップ値 */

    BOOL  initialized;                                      /**< オドメトリ初期化完了状態 */
    BOOL  gyro_calibrated;                                  /**< ジャイロバイアス校正完了状態 */
} odometry_context_t;

/* 純関数API */
EXPORT void odometry_init(odometry_context_t * p_ctx);      /* オドメトリ状態初期化 */
EXPORT void odometry_update(odometry_context_t * p_ctx,
                            UW left_count_24,
                            UW right_count_24,
                            UW uptime_ms_24,
                            H gyro_z_dps_x10,
                            H left_rpm_x10,
                            H right_rpm_x10); /* エンコーダ・ジャイロから姿勢更新 */
EXPORT void odometry_get_pose(const odometry_context_t * p_ctx, odometry_pose_t * p_pose); /* 姿勢取得 */

/* システム共有サービスAPI（スレッドセーフ） */
EXPORT void odometry_service_init(void);                    /* オドメトリサービス初期化 */
EXPORT void odometry_service_update(const actuator_status_t * p_status, H gyro_z_dps_x10); /* 姿勢更新 */
EXPORT void odometry_service_get_pose(odometry_pose_t * p_pose); /* オドメトリサービス姿勢取得 */

#endif /* SEROV_CPU0_SERVICE_ODOMETRY_H */
