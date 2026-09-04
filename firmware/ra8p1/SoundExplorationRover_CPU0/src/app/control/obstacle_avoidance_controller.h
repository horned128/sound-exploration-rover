/** =================================================================*
 * @file   obstacle_avoidance_controller.h
 * @brief  ToF・IMU用ルールベース走行判断API
 * ================================================================= */
#ifndef SEROV_CPU0_OBSTACLE_AVOIDANCE_CONTROLLER_H
#define SEROV_CPU0_OBSTACLE_AVOIDANCE_CONTROLLER_H

#include "sound_follow_controller.h"                       /* CPU0思考状態型 */
#include "../sensors/sensor_hub.h"                          /* センサースナップショット型 */
#include <tk/tkernel.h>                                     /* μT-Kernel型 */

typedef enum e_cpu0_sensor_rule {
    CPU0_SENSOR_RULE_SAFE_STOP = 0U,
    CPU0_SENSOR_RULE_FORWARD,
    CPU0_SENSOR_RULE_CAUTION_FORWARD,
    CPU0_SENSOR_RULE_TURN_LEFT,
    CPU0_SENSOR_RULE_TURN_RIGHT,
    CPU0_SENSOR_RULE_BLOCKED_STOP,
    CPU0_SENSOR_RULE_IMU_STOP,
} cpu0_sensor_rule_t;

typedef struct st_obstacle_avoidance_output {
    cpu0_think_state_t state;
    cpu0_sensor_rule_t rule;
    H steering_deg;
    H left_rpm;
    H right_rpm;
    BOOL actuator_enable;
    BOOL emergency_stop;
} obstacle_avoidance_output_t;

EXPORT void obstacle_avoidance_controller_init(void);       /* ルール判断初期化 */
EXPORT void obstacle_avoidance_controller_step(const cpu0_sensor_snapshot_t * p_snapshot,
                                                BOOL fault_active, obstacle_avoidance_output_t * p_output);

#endif /* SEROV_CPU0_OBSTACLE_AVOIDANCE_CONTROLLER_H */
