/** =================================================================*
 * @file   obstacle_avoidance_controller.c
 * @brief  ToF・IMU用ルールベース走行判断実装
 * ================================================================= */
#include "obstacle_avoidance_controller.h"                 /* 障害物回避判断API */
#include "sound_follow_controller.h"                       /* CPU0思考状態型 */
#include "config/control_config.h"                         /* 距離、IMU、速度設定 */
#include "config/sensor_config.h"                          /* センサー更新期限 */

LOCAL W obstacle_avoidance_abs_i16(H value);               /* Hの安全な絶対値 */
LOCAL BOOL obstacle_avoidance_snapshot_usable(const sensor_snapshot_t * p_snapshot); /* 利用可能性判定 */
LOCAL BOOL obstacle_avoidance_imu_safe(const sensor_snapshot_t * p_snapshot); /* IMU安全判定 */
LOCAL void obstacle_avoidance_output_stop(obstacle_avoidance_rule_t rule, sound_follow_state_t state,
                                          obstacle_avoidance_output_t * p_output); /* 停止指令生成 */
LOCAL void obstacle_avoidance_output_turn(obstacle_avoidance_rule_t rule, H steering_deg, H outer_rpm, H inner_rpm,
                                          obstacle_avoidance_output_t * p_output); /* 緩い旋回指令生成 */

/** =================================================================*
 * @brief  Hの安全な絶対値をWで取得
 * @param[in] value 入力値
 * @return 絶対値
 * ================================================================= */
LOCAL W obstacle_avoidance_abs_i16(H value) {
    return (value < 0) ? -(W) value : (W) value;
}

/** =================================================================*
 * @brief  全センサーの値が走行判断に利用可能か判定
 * @param[in] p_snapshot 最新センサースナップショット
 * @return 全ToF、IMUが有効かつ更新期限内ならtrue
 * ================================================================= */
LOCAL BOOL obstacle_avoidance_snapshot_usable(const sensor_snapshot_t * p_snapshot) {
    return (NULL != p_snapshot) && p_snapshot->initialized && (CPU0_SENSOR_VALID_ALL == p_snapshot->valid_flags) &&
           (p_snapshot->age_ms <= CPU0_SENSOR_STALE_TIMEOUT_MS);
}

/** =================================================================*
 * @brief  IMU値が安全範囲内か判定
 * @details BMI270のZ軸を車体上向きとして搭載し、X/Yの重力成分で大きな傾きを検知する。
 * @param[in] p_snapshot 最新センサースナップショット
 * @return 傾き、衝撃、角速度が安全範囲ならtrue
 * ================================================================= */
LOCAL BOOL obstacle_avoidance_imu_safe(const sensor_snapshot_t * p_snapshot) {
    W acceleration_l1_mg = 0;
    for (UW axis = 0U; axis < 3U; axis++) {
        acceleration_l1_mg += obstacle_avoidance_abs_i16(p_snapshot->accel_mg[axis]);
        if (obstacle_avoidance_abs_i16(p_snapshot->gyro_dps_x10[axis]) > CPU0_SENSOR_IMU_MAX_GYRO_DPS_X10) {
            return FALSE;
        }
    }

    if ((obstacle_avoidance_abs_i16(p_snapshot->accel_mg[0]) > CPU0_SENSOR_IMU_MAX_TILT_MG) ||
        (obstacle_avoidance_abs_i16(p_snapshot->accel_mg[1]) > CPU0_SENSOR_IMU_MAX_TILT_MG) ||
        (acceleration_l1_mg > CPU0_SENSOR_IMU_MAX_SHOCK_L1_MG)) {
        return FALSE;
    }

    return TRUE;
}

/** =================================================================*
 * @brief  停止指令を生成
 * @param[in] rule 停止理由
 * @param[in] state CPU0思考状態
 * @param[out] p_output 走行指令
 * ================================================================= */
LOCAL void obstacle_avoidance_output_stop(obstacle_avoidance_rule_t rule, sound_follow_state_t state,
                                          obstacle_avoidance_output_t * p_output) {
    *p_output = (obstacle_avoidance_output_t){
        .state = state,
        .rule = rule,
        .steering_deg = 0,
        .left_rpm = 0,
        .right_rpm = 0,
        .actuator_enable = FALSE,
        .emergency_stop = FALSE,
    };
}

/** =================================================================*
 * @brief  緩い旋回を伴う前進指令を生成
 * @param[in] rule 選択ルール
 * @param[in] steering_deg 右正の車体操舵角
 * @param[in] outer_rpm 外輪RPM
 * @param[in] inner_rpm 内輪RPM
 * @param[out] p_output 走行指令
 * ================================================================= */
LOCAL void obstacle_avoidance_output_turn(obstacle_avoidance_rule_t rule, H steering_deg, H outer_rpm, H inner_rpm,
                                          obstacle_avoidance_output_t * p_output) {
    *p_output = (obstacle_avoidance_output_t){
        .state = (steering_deg < 0) ? CPU0_THINK_STATE_SENSOR_TURN_LEFT : CPU0_THINK_STATE_SENSOR_TURN_RIGHT,
        .rule = rule,
        .steering_deg = steering_deg,
        .left_rpm = (steering_deg < 0) ? inner_rpm : outer_rpm,
        .right_rpm = (steering_deg < 0) ? outer_rpm : inner_rpm,
        .actuator_enable = TRUE,
        .emergency_stop = FALSE,
    };
}

/** =================================================================*
 * @brief  ルール判断状態を初期化
 * ================================================================= */
EXPORT void obstacle_avoidance_controller_init(void) {
}

/** =================================================================*
 * @brief  ToF 3台とBMI270から走行指令を決定
 * @details 後方距離は未計測のため、自律後退は行わず、全方向近接時は安全に停止する。
 * @param[in] p_snapshot 最新センサースナップショット
 * @param[in] fault_active CPU0の非回復可能fault有無
 * @param[out] p_output 4輪操舵・左右モーターへ渡す指令
 * ================================================================= */
EXPORT void obstacle_avoidance_controller_step(const sensor_snapshot_t * p_snapshot, BOOL fault_active,
                                                obstacle_avoidance_output_t * p_output) {
    if (NULL == p_output) {
        return;
    }
    if (fault_active || !obstacle_avoidance_snapshot_usable(p_snapshot)) {
        obstacle_avoidance_output_stop(CPU0_SENSOR_RULE_SAFE_STOP, CPU0_THINK_STATE_SENSOR_SAFE_STOP, p_output);
        return;
    }
    if (!obstacle_avoidance_imu_safe(p_snapshot)) {
        obstacle_avoidance_output_stop(CPU0_SENSOR_RULE_IMU_STOP, CPU0_THINK_STATE_SENSOR_IMU_STOP, p_output);
        return;
    }

    UH const left_mm = p_snapshot->tof_distance_mm[CPU0_TOF_LEFT];
    UH const center_mm = p_snapshot->tof_distance_mm[CPU0_TOF_CENTER];
    UH const right_mm = p_snapshot->tof_distance_mm[CPU0_TOF_RIGHT];

    if ((left_mm <= CPU0_SENSOR_BLOCKED_DISTANCE_MM) && (center_mm <= CPU0_SENSOR_BLOCKED_DISTANCE_MM) &&
        (right_mm <= CPU0_SENSOR_BLOCKED_DISTANCE_MM)) {
        obstacle_avoidance_output_stop(CPU0_SENSOR_RULE_BLOCKED_STOP, CPU0_THINK_STATE_SENSOR_BLOCKED_STOP, p_output);
    } else if (center_mm <= CPU0_SENSOR_HARD_STOP_DISTANCE_MM) {
        obstacle_avoidance_output_stop(CPU0_SENSOR_RULE_BLOCKED_STOP, CPU0_THINK_STATE_SENSOR_BLOCKED_STOP, p_output);
    } else if ((center_mm <= CPU0_SENSOR_CAUTION_DISTANCE_MM) || (left_mm <= CPU0_SENSOR_SIDE_DISTANCE_MM) ||
               (right_mm <= CPU0_SENSOR_SIDE_DISTANCE_MM)) {
        if (left_mm >= right_mm) {
            obstacle_avoidance_output_turn(CPU0_SENSOR_RULE_TURN_LEFT, -CPU0_SENSOR_TURN_STEERING_DEG,
                                           CPU0_SENSOR_TURN_OUTER_RPM, CPU0_SENSOR_TURN_INNER_RPM, p_output);
        } else {
            obstacle_avoidance_output_turn(CPU0_SENSOR_RULE_TURN_RIGHT, CPU0_SENSOR_TURN_STEERING_DEG,
                                           CPU0_SENSOR_TURN_OUTER_RPM, CPU0_SENSOR_TURN_INNER_RPM, p_output);
        }
    } else if (center_mm <= CPU0_SENSOR_CLEAR_DISTANCE_MM) {
        *p_output = (obstacle_avoidance_output_t){
            .state = CPU0_THINK_STATE_SENSOR_CAUTION_FORWARD,
            .rule = CPU0_SENSOR_RULE_CAUTION_FORWARD,
            .steering_deg = 0,
            .left_rpm = CPU0_SENSOR_CAUTION_RPM,
            .right_rpm = CPU0_SENSOR_CAUTION_RPM,
            .actuator_enable = TRUE,
            .emergency_stop = FALSE,
        };
    } else {
        *p_output = (obstacle_avoidance_output_t){
            .state = CPU0_THINK_STATE_SENSOR_FORWARD,
            .rule = CPU0_SENSOR_RULE_FORWARD,
            .steering_deg = 0,
            .left_rpm = CPU0_SENSOR_FORWARD_RPM,
            .right_rpm = CPU0_SENSOR_FORWARD_RPM,
            .actuator_enable = TRUE,
            .emergency_stop = FALSE,
        };
    }
}
