/** =================================================================*
 * @file   smooth_avoidance_planner.h
 * @brief  曲率制限付き滑らか障害物回避プランナAPI（ポテンシャル法純関数）
 * ================================================================= */
#ifndef SEROV_CPU0_SMOOTH_AVOIDANCE_PLANNER_H
#define SEROV_CPU0_SMOOTH_AVOIDANCE_PLANNER_H

#include <tk/tkernel.h>                                     /* μT-Kernel基本型 */

#ifdef __cplusplus
extern "C" {
#endif

#define SMOOTH_PLANNER_MAX_STEERING_DEG    (45.0f)          /**< 最大舵角 [deg] */
#define SMOOTH_PLANNER_HARD_STOP_MM        (250.0f)         /**< ハード停止距離 [mm] */
#define SMOOTH_PLANNER_RECOVER_CLEAR_MM    (650.0f)         /**< 走行復帰基準距離 [mm] */
#define SMOOTH_PLANNER_INFLUENCE_DISTANCE_MM (800.0f)       /**< 障害物影響距離 [mm] */
#define SMOOTH_PLANNER_FAR_DISTANCE_MM     (4000.0f)        /**< 無効チャネル仮定距離 [mm] */
#define SMOOTH_PLANNER_MIN_SPEED_SCALE     (0.20f)          /**< 走行時最小速度スケール */
#define SMOOTH_PLANNER_MAX_STEER_RATE_DPS  (90.0f)          /**< 最大操舵角変化率 [deg/s] */
#define SMOOTH_PLANNER_MAX_ACCEL_PER_SEC   (1.5f)           /**< 最大速度スケール変化率 [/s] */

/* ToF 3台の車体中心基準配置オフセット [mm] (PLAN.md 0-D-1) */
#define SMOOTH_PLANNER_TOF_LEFT_X_MM       (94.0f)          /**< 滑らか回避プランナToF左のX[mm] */
#define SMOOTH_PLANNER_TOF_LEFT_Y_MM       (-90.0f)         /**< 滑らか回避プランナToF左のY[mm] */
#define SMOOTH_PLANNER_TOF_CENTER_X_MM     (0.0f)           /**< 滑らか回避プランナToF中央のX[mm] */
#define SMOOTH_PLANNER_TOF_CENTER_Y_MM     (0.0f)           /**< 滑らか回避プランナToF中央のY[mm] */
#define SMOOTH_PLANNER_TOF_RIGHT_X_MM      (94.0f)          /**< 滑らか回避プランナToF右のX[mm] */
#define SMOOTH_PLANNER_TOF_RIGHT_Y_MM      (90.0f)          /**< 滑らか回避プランナToF右のY[mm] */

/**< 滑らか回避プランナへ渡すセンサーと車体状態 */
typedef struct st_smooth_avoidance_input {
    float tof_distance_mm[3];                               /**< 0: Left, 1: Center, 2: Right [mm] */
    BOOL  tof_valid[3];                                     /**< 各ToFの有効フラグ */
    float target_heading_deg;                               /**< 目標方位角 [-180, +180], 正=右, 負=左 */
    float current_steering_deg;                             /**< 直前の操舵角 [-45, +45] */
    float current_speed_scale;                              /**< 直前の速度スケール [0.0, 1.0] */
    float dt_sec;                                           /**< 周期時間 [s] (通常 0.05f または 0.10f) */
} smooth_avoidance_input_t;

/**< 滑らか回避プランナが算出する操舵・速度指令 */
typedef struct st_smooth_avoidance_output {
    float steering_deg;                                     /**< 決定操舵角 [-45, +45], 正=右, 負=左 */
    float speed_scale;                                      /**< 決定速度スケール [0.0, 1.0] */
    BOOL  is_blocked;                                       /**< ハード停止 (250mm未満または全無効) */
    float left_clearance_mm;                                /**< 左クリアランス[mm] */
    float center_clearance_mm;                              /**< 中央クリアランス[mm] */
    float right_clearance_mm;                               /**< 右クリアランス[mm] */
} smooth_avoidance_output_t;

EXPORT void smooth_avoidance_plan(const smooth_avoidance_input_t * p_input,
                                  smooth_avoidance_output_t * p_output); /* 回避計画算出 */

#ifdef __cplusplus
}
#endif

#endif /* SEROV_CPU0_SMOOTH_AVOIDANCE_PLANNER_H */
