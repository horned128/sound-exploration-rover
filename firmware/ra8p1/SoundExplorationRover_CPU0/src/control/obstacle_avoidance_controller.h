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
    CPU0_SENSOR_RULE_BACKUP,                                /**< 後退 */
} obstacle_avoidance_rule_t;

/**< ルールベース障害物回避の状態とアクチュエータ指令 */
typedef struct st_obstacle_avoidance_output {
    sound_follow_state_t state;                             /**< 走行状態 */
    obstacle_avoidance_rule_t rule;                         /**< 適用した障害物回避ルール */
    H steering_deg;                                         /**< 操舵角指令[deg] */
    H left_rpm;                                             /**< 左車輪指令RPM */
    H right_rpm;                                            /**< 右車輪指令RPM */
    BOOL actuator_enable;                                   /**< アクチュエータ出力許可 */
    BOOL emergency_stop;                                    /**< 非常停止指令 */
} obstacle_avoidance_output_t;

EXPORT void obstacle_avoidance_controller_init(void);       /* ルール判断初期化 */
EXPORT void obstacle_avoidance_controller_step(const sensor_snapshot_t * p_snapshot,
                                                BOOL fault_active, UW now_ms, H target_steering_deg,
                                                obstacle_avoidance_output_t * p_output); /* 回避指令算出 */

#endif /* SEROV_CPU0_OBSTACLE_AVOIDANCE_CONTROLLER_H */
