/** =================================================================*
 * @file   obstacle_avoidance_controller.h
 * @brief  ToF・IMU用ルールベース走行判断API
 * ================================================================= */
#ifndef SEROV_CPU0_OBSTACLE_AVOIDANCE_CONTROLLER_H
#define SEROV_CPU0_OBSTACLE_AVOIDANCE_CONTROLLER_H

#include "sound_follow_controller.h"                        /* CPU0思考状態型 */
#include "services/sensor_hub.h"                            /* センサースナップショット型 */
#include <tk/tkernel.h>                                     /* μT-Kernel型 */

/**< ToF・IMU障害物回避で適用するルール */
typedef enum e_sensor_rule {
    CPU0_SENSOR_RULE_SAFE_STOP = 0U,                        /**< センサー未準備時の安全停止 */
    CPU0_SENSOR_RULE_FORWARD,                               /**< 前進 */
    CPU0_SENSOR_RULE_CAUTION_FORWARD,                       /**< 注意前進 */
    CPU0_SENSOR_RULE_TURN_LEFT,                             /**< 左旋回 */
    CPU0_SENSOR_RULE_TURN_RIGHT,                            /**< 右旋回 */
    CPU0_SENSOR_RULE_BLOCKED_STOP,                          /**< 障害物による停止 */
    CPU0_SENSOR_RULE_IMU_STOP,                              /**< IMU異常による停止 */
    CPU0_SENSOR_RULE_PIVOT_LEFT,                            /**< 左ピボット */
    CPU0_SENSOR_RULE_PIVOT_RIGHT,                           /**< 右ピボット */
    CPU0_SENSOR_RULE_BACKUP,                                /**< 正面近接からの短距離後退 */
} obstacle_avoidance_rule_t;

/**< ルールベース障害物回避の状態とアクチュエータ指令 */
typedef struct st_obstacle_avoidance_output {
    sound_follow_state_t state;                             /**< 走行状態 */
    obstacle_avoidance_rule_t rule;                         /**< 適用した障害物回避ルール */
    H steering_deg;                                         /**< 操舵角指令[deg] */
    BOOL is_spin_turn;                                      /**< X字操舵の最小並進旋回 */
    H left_rpm;                                             /**< 左車輪指令RPM */
    H right_rpm;                                            /**< 右車輪指令RPM */
    BOOL actuator_enable;                                   /**< アクチュエータ出力許可 */
    BOOL emergency_stop;                                    /**< 非常停止指令 */
    BOOL avoidance_in_progress;                             /**< 回避側を保持して通過中 */
    BOOL avoidance_completed;                               /**< 安全なクリアランスを確認して回避完了 */
} obstacle_avoidance_output_t;

EXPORT void obstacle_avoidance_controller_init(void);       /* ルール判断初期化 */
EXPORT void obstacle_avoidance_encoder_feedback_set(BOOL valid, UW status_sequence,
    H left_rpm_x10, H right_rpm_x10); /* CPU1実測速度を渡す */
EXPORT BOOL obstacle_avoidance_spin_space_available(const sensor_snapshot_t * p_snapshot); /* 音源旋回の前方クリアランス */
EXPORT B obstacle_avoidance_rear_seam_turn_preference(const sensor_snapshot_t * p_snapshot); /* 真後ろ音源の安全な旋回側 */
EXPORT void obstacle_avoidance_controller_step(const sensor_snapshot_t * p_snapshot,
    BOOL fault_active, UW now_ms, H target_steering_deg,
    H linear_speed_mm_s, obstacle_avoidance_output_t * p_output); /* 回避指令算出 */

#endif /* SEROV_CPU0_OBSTACLE_AVOIDANCE_CONTROLLER_H */
