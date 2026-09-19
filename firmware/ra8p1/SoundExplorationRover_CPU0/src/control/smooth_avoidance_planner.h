/** =================================================================*
 * @file   smooth_avoidance_planner.h
 * @brief  曲率制限付き滑らか障害物回避プランナAPI（ポテンシャル法純関数）
 * ================================================================= */
#ifndef SEROV_CPU0_SMOOTH_AVOIDANCE_PLANNER_H
#define SEROV_CPU0_SMOOTH_AVOIDANCE_PLANNER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SMOOTH_PLANNER_MAX_STEERING_DEG      (45.0f)   /**< 最大舵角 [deg] */
#define SMOOTH_PLANNER_HARD_STOP_MM          (250.0f)  /**< ハード停止距離 [mm] */
#define SMOOTH_PLANNER_RECOVER_CLEAR_MM      (650.0f)  /**< 走行復帰基準距離 [mm] */
#define SMOOTH_PLANNER_INFLUENCE_DISTANCE_MM (800.0f)  /**< 障害物影響距離 [mm] */
#define SMOOTH_PLANNER_FAR_DISTANCE_MM       (4000.0f) /**< 無効チャネル仮定距離 [mm] */
#define SMOOTH_PLANNER_MIN_SPEED_SCALE       (0.20f)   /**< 走行時最小速度スケール */
#define SMOOTH_PLANNER_MAX_STEER_RATE_DPS    (90.0f)   /**< 最大操舵角変化率 [deg/s] */
#define SMOOTH_PLANNER_MAX_ACCEL_PER_SEC     (1.5f)    /**< 最大速度スケール変化率 [/s] */

/**< ToF 3台の車体中心基準配置オフセット [mm] (PLAN.md 0-D-1) */
#define SMOOTH_PLANNER_TOF_LEFT_X_MM         (94.0f)
#define SMOOTH_PLANNER_TOF_LEFT_Y_MM         (-90.0f)
#define SMOOTH_PLANNER_TOF_CENTER_X_MM       (0.0f)
#define SMOOTH_PLANNER_TOF_CENTER_Y_MM       (0.0f)
#define SMOOTH_PLANNER_TOF_RIGHT_X_MM        (94.0f)
#define SMOOTH_PLANNER_TOF_RIGHT_Y_MM        (90.0f)

typedef struct st_smooth_avoidance_input {
    float tof_distance_mm[3];       /**< 0: Left, 1: Center, 2: Right [mm] */
    bool  tof_valid[3];             /**< 各ToFの有効フラグ */
    float target_heading_deg;       /**< 目標方位角 [-180, +180], 正=右, 負=左 */
    float current_steering_deg;     /**< 直前の操舵角 [-45, +45] */
    float current_speed_scale;      /**< 直前の速度スケール [0.0, 1.0] */
    float dt_sec;                   /**< 周期時間 [s] (通常 0.05f または 0.10f) */
} smooth_avoidance_input_t;

typedef struct st_smooth_avoidance_output {
    float steering_deg;             /**< 決定操舵角 [-45, +45], 正=右, 負=左 */
    float speed_scale;              /**< 決定速度スケール [0.0, 1.0] */
    bool  is_blocked;               /**< ハード停止 (250mm未満または全無効) */
    float left_clearance_mm;        /**< 幾何オフセット考慮後左クリアランス [mm] */
    float center_clearance_mm;      /**< 幾何オフセット考慮後中央クリアランス [mm] */
    float right_clearance_mm;       /**< 幾何オフセット考慮後右クリアランス [mm] */
} smooth_avoidance_output_t;

/** =================================================================*
 * @brief  滑らかな操舵角と速度スケールを算出する純関数
 * @param[in]  p_input  観測・前回状態入力
 * @param[out] p_output 計画された操舵角と速度
 * ================================================================= */
void smooth_avoidance_plan(const smooth_avoidance_input_t * p_input,
                           smooth_avoidance_output_t * p_output);

#ifdef __cplusplus
}
#endif

#endif /* SEROV_CPU0_SMOOTH_AVOIDANCE_PLANNER_H */
