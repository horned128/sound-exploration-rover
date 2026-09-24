/** =================================================================*
 * @file   obstacle_avoidance_controller.c
 * @brief  ToF・IMUによる方向保持付き障害物回避と有限回の最小並進旋回
 * ================================================================= */
#include "obstacle_avoidance_controller.h"                  /* 障害物回避判断API */
#include "config/control_config.h"                          /* 距離、IMU、速度設定 */
#include "config/sensor_config.h"                           /* センサー更新期限 */

/**< 障害物脱出シーケンスの段階 */
typedef enum e_avoidance_escape_phase {
    AVOIDANCE_ESCAPE_NONE = 0,                              /**< 通常走行中 */
    AVOIDANCE_ESCAPE_STEER,                                 /**< 操舵脱出中 */
    AVOIDANCE_ESCAPE_PIVOT,                                 /**< ピボット脱出中 */
    AVOIDANCE_ESCAPE_BACKUP,                                /**< 正面近接からの短距離後退中 */
    AVOIDANCE_ESCAPE_BLOCKED,                               /**< 脱出不能で停止 */
} avoidance_escape_phase_t;

/**< 障害物回避の脱出段階・方向・進行監視状態 */
typedef struct st_avoidance_controller {
    avoidance_escape_phase_t phase;                         /**< 現在の脱出段階 */
    B direction;                                            /**< 回避方向（左負、右正） */
    UW phase_ms;                                            /**< 段階開始からの経過時間[ms] */
    UW session_ms;                                          /**< 回避開始からの経過時間[ms] */
    UW progress_ms;                                         /**< 進行観測からの経過時間[ms] */
    W yaw_mdeg;                                             /**< 現在の相対ヨー角[mdeg] */
    W progress_yaw_mdeg;                                    /**< 進行判定基準のヨー角[mdeg] */
    UB frontal_collision_count;                             /**< 正面近接の連続確認回数 */
    UB recovery_attempts;                                   /**< 後退・再旋回の試行回数 */
    UW wheel_stall_ms;                                      /**< 実測速度異常の継続時間[ms] */
    UW feedback_sequence;                                   /**< 最後に観測したCPU1状態sequence */
    H feedback_left_rpm_x10;                                /**< CPU1左輪実測[0.1RPM] */
    H feedback_right_rpm_x10;                               /**< CPU1右輪実測[0.1RPM] */
    BOOL feedback_valid;                                    /**< 実測状態を取得済み */
    BOOL feedback_new;                                      /**< この思考周期に新しい実測を受信 */
    UW last_ms;                                             /**< 前回制御時刻[ms] */
    UW last_sample_ms;                                      /**< 前回センサー観測時刻[ms] */
    UW last_update_count;                                   /**< 前回センサー更新回数 */
    BOOL clock_valid;                                       /**< 制御時刻の有効状態 */
    BOOL sample_valid;                                      /**< 最新センサー観測の有効状態 */
    BOOL session_active;                                    /**< 音源追従中に始めた回避シーケンス */
    UW stuck_elapsed_ms;                                    /**< スタック状態の継続時間[ms] */
    H last_cmd_left_rpm;                                    /**< 直前ステップの左車輪指令RPM */
    H last_cmd_right_rpm;                                   /**< 直前ステップの右車輪指令RPM */
} avoidance_controller_t;

LOCAL avoidance_controller_t avoidance;                     /**< 回避方向、回頭量、脱出履歴 */

/** =================================================================*
 * @brief  Hの安全な絶対値
 * @param[in] value 符号付き値
 * @return Wで表した絶対値
 * ================================================================= */
LOCAL W obstacle_avoidance_abs_i16(H value) {
    return (value < 0) ? -(W) value : (W) value;
}

/** =================================================================*
 * @brief  走行判断用スナップショットの利用可能性判定
 * @param[in] p_snapshot 最新センサー値
 * @return 全センサーが有効かつ期限内ならTRUE
 * ================================================================= */
LOCAL BOOL obstacle_avoidance_snapshot_usable(const sensor_snapshot_t * p_snapshot) {
    return (NULL != p_snapshot) && p_snapshot->initialized && (CPU0_SENSOR_VALID_ALL == p_snapshot->valid_flags) &&
           (p_snapshot->age_ms <= CPU0_SENSOR_STALE_TIMEOUT_MS);
}

/** =================================================================*
 * @brief  傾斜・衝撃・過大角速度判定
 * @param[in] p_snapshot 最新センサー値
 * @return 安全範囲内ならTRUE
 * ================================================================= */
LOCAL BOOL obstacle_avoidance_imu_safe(const sensor_snapshot_t * p_snapshot) {
    W acceleration_l1_mg = 0;
    for (UW axis = 0U; axis < 3U; axis++) {
        acceleration_l1_mg += obstacle_avoidance_abs_i16(p_snapshot->accel_mg[axis]);
        if (obstacle_avoidance_abs_i16(p_snapshot->gyro_dps_x10[axis]) > CPU0_SENSOR_IMU_MAX_GYRO_DPS_X10) {
            return FALSE;
        }
    }
    return (obstacle_avoidance_abs_i16(p_snapshot->accel_mg[0]) <= CPU0_SENSOR_IMU_MAX_TILT_MG) &&
           (obstacle_avoidance_abs_i16(p_snapshot->accel_mg[1]) <= CPU0_SENSOR_IMU_MAX_TILT_MG) &&
           (acceleration_l1_mg <= CPU0_SENSOR_IMU_MAX_SHOCK_L1_MG);
}

/** =================================================================*
 * @brief  音源方向への旋回可否判定
 * @details 3眼ToFは後方を見られない。前方と斜め方向の余裕のみ判定し、
 *          閾値未満では従来の後退・ピボット脱出へ委ねる。
 * @param[in] p_snapshot 最新センサー値
 * @return 全ToFとIMUが旋回可能範囲ならTRUE
 * ================================================================= */
EXPORT BOOL obstacle_avoidance_spin_space_available(const sensor_snapshot_t * p_snapshot) {
    if (!obstacle_avoidance_snapshot_usable(p_snapshot) || !obstacle_avoidance_imu_safe(p_snapshot)) {
        return FALSE;
    }
    /* 実演用閾値が下げられていても、前輪接触・旋回掃引を避ける
     * 脱出側面距離より近ければその場旋回を許可しない。 */
    UH const clearance_mm = (CPU0_SOUND_SPIN_CLEARANCE_MM > CPU0_SENSOR_ESCAPE_SIDE_MM) ?
        CPU0_SOUND_SPIN_CLEARANCE_MM : CPU0_SENSOR_ESCAPE_SIDE_MM;
    return (p_snapshot->tof_distance_mm[CPU0_TOF_LEFT] >= clearance_mm) &&
           (p_snapshot->tof_distance_mm[CPU0_TOF_CENTER] >= clearance_mm) &&
           (p_snapshot->tof_distance_mm[CPU0_TOF_RIGHT] >= clearance_mm);
}

/** =================================================================*
 * @brief  真後ろ音源の旋回方向選択
 * @param[in] p_snapshot 最新センサー値
 * @return 右+1、左-1、差が小さいか無効なら0
 * ================================================================= */
EXPORT B obstacle_avoidance_rear_seam_turn_preference(const sensor_snapshot_t * p_snapshot) {
    if (!obstacle_avoidance_snapshot_usable(p_snapshot)) {
        return 0;
    }
    UH const left_mm = p_snapshot->tof_distance_mm[CPU0_TOF_LEFT];
    UH const right_mm = p_snapshot->tof_distance_mm[CPU0_TOF_RIGHT];
    UH const clearance_mm = (CPU0_SOUND_SPIN_CLEARANCE_MM > CPU0_SENSOR_ESCAPE_SIDE_MM) ?
        CPU0_SOUND_SPIN_CLEARANCE_MM : CPU0_SENSOR_ESCAPE_SIDE_MM;
    if ((left_mm < clearance_mm) && (right_mm >= clearance_mm)) {
        return 1;
    }
    if ((right_mm < clearance_mm) && (left_mm >= clearance_mm)) {
        return -1;
    }
    W const difference_mm = (W) right_mm - (W) left_mm;
    if (difference_mm >= (W) CPU0_SENSOR_ESCAPE_DIRECTION_SIDE_DELTA_MM) {
        return 1;
    }
    if (difference_mm <= -(W) CPU0_SENSOR_ESCAPE_DIRECTION_SIDE_DELTA_MM) {
        return -1;
    }
    return 0;
}

/** =================================================================*
 * @brief  駆動無効の停止指令生成
 * @param[in] rule 停止理由
 * @param[in] state 思考状態
 * @param[out] p_output 停止指令
 * ================================================================= */
LOCAL void obstacle_avoidance_output_stop(obstacle_avoidance_rule_t rule, sound_follow_state_t state,
                                         obstacle_avoidance_output_t * p_output) {
    *p_output = (obstacle_avoidance_output_t){.state = state, .rule = rule};
}

/** =================================================================*
 * @brief  X字操舵での最小並進旋回または舵角整定指令生成
 * @details 6輪の中央輪固定と片側共通RPMにより無滑りのzero-radius旋回ではない。
 *          既存のPIVOTテレメトリ名は維持し、低速・時間制限付きで使用する。
 * @param[in] moving 整定を終えて旋回を開始してよければTRUE
 * @param[out] p_output 旋回指令
 * ================================================================= */
LOCAL void obstacle_avoidance_output_pivot(BOOL moving, obstacle_avoidance_output_t * p_output) {
    BOOL const left = avoidance.direction < 0;
    *p_output = (obstacle_avoidance_output_t){
        .state = left ? CPU0_THINK_STATE_SENSOR_PIVOT_LEFT : CPU0_THINK_STATE_SENSOR_PIVOT_RIGHT,
        .rule = left ? CPU0_SENSOR_RULE_PIVOT_LEFT : CPU0_SENSOR_RULE_PIVOT_RIGHT,
        .steering_deg = 0,
        .is_spin_turn = TRUE,
        .left_rpm = moving ? (left ? -CPU0_SENSOR_ESCAPE_TURN_RPM : CPU0_SENSOR_ESCAPE_TURN_RPM) : 0,
        .right_rpm = moving ? (left ? CPU0_SENSOR_ESCAPE_TURN_RPM : -CPU0_SENSOR_ESCAPE_TURN_RPM) : 0,
        .actuator_enable = TRUE,
        .avoidance_in_progress = TRUE,
    };
}

/** =================================================================*
 * @brief  正面近接から距離を取るための短距離後退指令生成
 * @param[out] p_output 後退指令
 * ================================================================= */
LOCAL void obstacle_avoidance_output_backup(obstacle_avoidance_output_t * p_output) {
    *p_output = (obstacle_avoidance_output_t){
        .state = CPU0_THINK_STATE_SENSOR_BACKUP,
        .rule = CPU0_SENSOR_RULE_BACKUP,
        .steering_deg = 0,
        .left_rpm = -CPU0_SENSOR_BACKUP_RPM,
        .right_rpm = -CPU0_SENSOR_BACKUP_RPM,
        .actuator_enable = TRUE,
        .avoidance_in_progress = TRUE,
    };
}

/** =================================================================*
 * @brief  後退・再旋回の試行上限に達した場合の安全停止
 * @param[out] p_output 停止指令
 * ================================================================= */
LOCAL void obstacle_avoidance_block(obstacle_avoidance_output_t * p_output) {
    avoidance.phase = AVOIDANCE_ESCAPE_BLOCKED;
    obstacle_avoidance_output_stop(CPU0_SENSOR_RULE_BLOCKED_STOP, CPU0_THINK_STATE_SENSOR_BLOCKED_STOP, p_output);
}

/** =================================================================*
 * @brief  近傍の空きと停止中に確定した音源方位から脱出方向を選ぶ
 * @details 音源側に脱出可能な横クリアランスがあれば、多少反対側が広くても
 *          音源側を優先する。十分な空間があるのに反対側の壁沿いへ逃げると、
 *          再聴取後に大きな向き直しが必要になり袋小路へ入りやすいためである。
 *          音源側が狭い場合だけ空き側を選び、安全性を優先する。
 * @param[in] left_mm 左ToF距離[mm]
 * @param[in] right_mm 右ToF距離[mm]
 * @param[in] target_steering_deg 静止中に確定した音源方位（右正）[deg]
 * @param[in] fallback_direction いずれも決め手がない場合の継続方向
 * @return 脱出方向（左負、右正）
 * ================================================================= */
LOCAL B obstacle_avoidance_choose_escape_direction(UH left_mm, UH right_mm, H target_steering_deg,
                                                    B fallback_direction) {
    if (obstacle_avoidance_abs_i16(target_steering_deg) >= CPU0_SENSOR_ESCAPE_TARGET_DEADBAND_DEG) {
        BOOL const target_on_left = target_steering_deg < 0;
        UH const target_side_mm = target_on_left ? left_mm : right_mm;
        if (target_side_mm >= CPU0_SENSOR_ESCAPE_SIDE_MM) {
            return target_on_left ? -1 : 1;
        }
    }
    W const side_difference_mm = (left_mm >= right_mm) ?
        (W) left_mm - (W) right_mm : (W) right_mm - (W) left_mm;
    if (side_difference_mm >= (W) CPU0_SENSOR_ESCAPE_DIRECTION_SIDE_DELTA_MM) {
        return (left_mm >= right_mm) ? -1 : 1;
    }
    if (0 != fallback_direction) {
        return fallback_direction;
    }
    return (left_mm >= right_mm) ? -1 : 1;
}

/** =================================================================*
 * @brief  最小並進旋回開始
 * @param[in] left_mm 左ToF距離[mm]
 * @param[in] right_mm 右ToF距離[mm]
 * @param[in] target_steering_deg 静止中に確定した音源方位（右正）[deg]
 * @param[out] p_output 操舵整定または上限停止指令
 * ================================================================= */
LOCAL void obstacle_avoidance_start_turn(UH left_mm, UH right_mm, H target_steering_deg,
                                         obstacle_avoidance_output_t * p_output) {
    avoidance.direction = obstacle_avoidance_choose_escape_direction(left_mm, right_mm, target_steering_deg,
                                                                      avoidance.direction);
    avoidance.yaw_mdeg = 0;
    avoidance.phase = AVOIDANCE_ESCAPE_STEER;
    avoidance.session_active = TRUE;
    avoidance.phase_ms = 0U;
    avoidance.progress_ms = 0U;
    avoidance.progress_yaw_mdeg = avoidance.yaw_mdeg;
    obstacle_avoidance_output_pivot(FALSE, p_output);
}

/** =================================================================*
 * @brief  衝突候補または回頭不成立から後退復帰を開始
 * @details 後退後の旋回方向は新しいToFと音源方位で選び直す。試行回数を
 *          制限し、壁際での無限ピボットを安全停止へ収束させる。
 * @param[out] p_output 後退または安全停止指令
 * ================================================================= */
LOCAL void obstacle_avoidance_start_backup(obstacle_avoidance_output_t * p_output) {
    if (avoidance.recovery_attempts >= CPU0_SENSOR_MAX_RECOVERY_ATTEMPTS) {
        obstacle_avoidance_block(p_output);
        return;
    }
    avoidance.recovery_attempts++;
    avoidance.frontal_collision_count = 0U;
    avoidance.wheel_stall_ms = 0U;
    avoidance.stuck_elapsed_ms = 0U;
    avoidance.phase = AVOIDANCE_ESCAPE_BACKUP;
    avoidance.phase_ms = 0U;
    avoidance.progress_ms = 0U;
    avoidance.progress_yaw_mdeg = avoidance.yaw_mdeg;
    avoidance.session_active = TRUE;
    obstacle_avoidance_output_backup(p_output);
}

/** =================================================================*
 * @brief  回避状態初期化
 * ================================================================= */
EXPORT void obstacle_avoidance_controller_init(void) {
    avoidance = (avoidance_controller_t){0};
}

/** =================================================================*
 * @brief  CPU1実測速度の設定
 * @details 同じstatus_sequenceを繰り返し受けたときは古い速度を再加算しない。
 * @param[in] valid 実測値の有効状態
 * @param[in] status_sequence CPU1状態更新番号
 * @param[in] left_rpm_x10 左輪実測速度[0.1RPM]
 * @param[in] right_rpm_x10 右輪実測速度[0.1RPM]
 * ================================================================= */
EXPORT void obstacle_avoidance_encoder_feedback_set(BOOL valid, UW status_sequence,
    H left_rpm_x10, H right_rpm_x10) {
    avoidance.feedback_new = valid &&
        (!avoidance.feedback_valid || (avoidance.feedback_sequence != status_sequence));
    avoidance.feedback_valid = valid;
    if (avoidance.feedback_new) {
        avoidance.feedback_sequence = status_sequence;
        avoidance.feedback_left_rpm_x10 = left_rpm_x10;
        avoidance.feedback_right_rpm_x10 = right_rpm_x10;
    }
    if (!valid) {
        avoidance.wheel_stall_ms = 0U;
    }
}

/** =================================================================*
 * @brief  ToFとIMUによる障害物回避指令の決定
 * @details now_msは単調増加時計の下位32 bit。繰り返し観測を回頭量へ二重加算しない。
 * @param[in] p_snapshot 最新センサー値。NULLは安全停止
 * @param[in] fault_active 異常・走行禁止状態
 * @param[in] now_ms 呼出側の実時刻[ms]
 * @param[in] target_steering_deg 音源方向への引力として用いる目標操舵角[°]
 * @param[in] linear_speed_mm_s エンコーダ実測並進速度[mm/s]
 * @param[out] p_output 走行指令
 * ================================================================= */
EXPORT void obstacle_avoidance_controller_step(const sensor_snapshot_t * p_snapshot, BOOL fault_active,
    UW now_ms, H target_steering_deg, H linear_speed_mm_s,
    obstacle_avoidance_output_t * p_output) {
    if (NULL == p_output) {
        return;
    }
    UW const elapsed_ms = avoidance.clock_valid ? now_ms - avoidance.last_ms : 0U;
    if (fault_active || !obstacle_avoidance_snapshot_usable(p_snapshot) ||
        (elapsed_ms > CPU0_SENSOR_MAX_STEP_MS)) {
        obstacle_avoidance_controller_init();
        obstacle_avoidance_output_stop(CPU0_SENSOR_RULE_SAFE_STOP, CPU0_THINK_STATE_SENSOR_SAFE_STOP, p_output);
        return;
    }
    if (!obstacle_avoidance_imu_safe(p_snapshot)) {
        obstacle_avoidance_controller_init();
        obstacle_avoidance_output_stop(CPU0_SENSOR_RULE_IMU_STOP, CPU0_THINK_STATE_SENSOR_IMU_STOP, p_output);
        return;
    }
    avoidance.last_ms = now_ms;
    avoidance.clock_valid = TRUE;

    /* gyro[0.1 dps] × ms / 10 = mdeg。逆向き回頭は差し引き、往復振動を成功と数えない。 */
    BOOL const new_sample = !avoidance.sample_valid || (avoidance.last_update_count != p_snapshot->update_count);
    if (new_sample) {
        UW const sample_ms = now_ms - p_snapshot->age_ms;
        UW const sample_elapsed_ms = sample_ms - avoidance.last_sample_ms;
        if (avoidance.sample_valid && (0 != avoidance.direction) &&
            ((AVOIDANCE_ESCAPE_PIVOT == avoidance.phase) || (AVOIDANCE_ESCAPE_NONE == avoidance.phase)) &&
            (sample_elapsed_ms <= CPU0_SENSOR_MAX_STEP_MS)) {
            H const gyro = p_snapshot->gyro_dps_x10[CPU0_SENSOR_YAW_AXIS];
            if (obstacle_avoidance_abs_i16(gyro) >= CPU0_SENSOR_YAW_DEADBAND_DPS_X10) {
                avoidance.yaw_mdeg += (W) gyro * CPU0_SENSOR_YAW_RIGHT_SIGN * avoidance.direction *
                                      (W) sample_elapsed_ms / 10;
                if (avoidance.yaw_mdeg > 360000) {
                    avoidance.yaw_mdeg = 360000;
                } else if (avoidance.yaw_mdeg < -360000) {
                    avoidance.yaw_mdeg = -360000;
                }
            }
        }
        avoidance.last_sample_ms = sample_ms;
        avoidance.last_update_count = p_snapshot->update_count;
        avoidance.sample_valid = TRUE;
    }

    UH const left_mm = p_snapshot->tof_distance_mm[CPU0_TOF_LEFT];
    UH const center_mm = p_snapshot->tof_distance_mm[CPU0_TOF_CENTER];
    UH const right_mm = p_snapshot->tof_distance_mm[CPU0_TOF_RIGHT];
    BOOL const frontal_collision = center_mm <= CPU0_SENSOR_BACKUP_TRIGGER_DISTANCE_MM;
    BOOL avoidance_completed = FALSE;

    BOOL const forward_motion_commanded = (avoidance.last_cmd_left_rpm >= CPU0_SENSOR_STUCK_MIN_CMD_RPM) &&
                                          (avoidance.last_cmd_right_rpm >= CPU0_SENSOR_STUCK_MIN_CMD_RPM);

    /* 壁面へ正対したときは、正面300mm以内を新しい観測で連続確認して後退する。
     * 確認後は停止固定ではなく短距離後退へ移る。後退中の前方ToFは同じ壁を
     * 見続けるため、次の復帰試行へ持ち越さない。 */
    if (new_sample) {
        if (AVOIDANCE_ESCAPE_BACKUP == avoidance.phase) {
            avoidance.frontal_collision_count = 0U;
        } else if (frontal_collision) {
            if (avoidance.frontal_collision_count < CPU0_SENSOR_COLLISION_CONFIRM_COUNT) {
                avoidance.frontal_collision_count++;
            }
        } else {
            avoidance.frontal_collision_count = 0U;
        }
    }
    if ((AVOIDANCE_ESCAPE_BACKUP != avoidance.phase) &&
        (avoidance.frontal_collision_count >= CPU0_SENSOR_COLLISION_CONFIRM_COUNT)) {
        obstacle_avoidance_start_backup(p_output);
        avoidance.last_cmd_left_rpm = p_output->left_rpm;
        avoidance.last_cmd_right_rpm = p_output->right_rpm;
        return;
    }

    /* 前進指令中、並進速度が±CPU0_SENSOR_STUCK_STALL_SPEED_MM_S以内、または
     * 前後軸加速度の絶対値がCPU0_SENSOR_STUCK_TILT_THRESH_MG以上の状態が
     * CPU0_SENSOR_STUCK_CONFIRM_MS続けば、ToFだけでは検知できないスタックとして後退する。
     * 片輪停止や左右速度不均衡は、CPU1状態更新ごとに別の確認時間で判定する。 */
    if ((AVOIDANCE_ESCAPE_NONE == avoidance.phase) && forward_motion_commanded && avoidance.feedback_new) {
        BOOL const left_stalled = (obstacle_avoidance_abs_i16(avoidance.feedback_left_rpm_x10) <=
                                   CPU0_SENSOR_WHEEL_STALL_RPM_X10) &&
                                  (obstacle_avoidance_abs_i16(avoidance.feedback_right_rpm_x10) >=
                                   CPU0_SENSOR_WHEEL_MOVING_RPM_X10);
        BOOL const right_stalled = (obstacle_avoidance_abs_i16(avoidance.feedback_right_rpm_x10) <=
                                    CPU0_SENSOR_WHEEL_STALL_RPM_X10) &&
                                   (obstacle_avoidance_abs_i16(avoidance.feedback_left_rpm_x10) >=
                                    CPU0_SENSOR_WHEEL_MOVING_RPM_X10);
        W const left_rpm_x10 = obstacle_avoidance_abs_i16(avoidance.feedback_left_rpm_x10);
        W const right_rpm_x10 = obstacle_avoidance_abs_i16(avoidance.feedback_right_rpm_x10);
        BOOL const near_side = (left_mm <= CPU0_SENSOR_SIDE_CONTACT_DISTANCE_MM) ||
                               (right_mm <= CPU0_SENSOR_SIDE_CONTACT_DISTANCE_MM);
        /* 意図的な左右RPM差を除外し、指令に対する実測比が半分未満の側だけを異常とする。 */
        BOOL const side_imbalance = near_side &&
            (((left_rpm_x10 >= CPU0_SENSOR_WHEEL_MOVING_RPM_X10) &&
              (right_rpm_x10 * 2 * avoidance.last_cmd_left_rpm <
               left_rpm_x10 * avoidance.last_cmd_right_rpm)) ||
             ((right_rpm_x10 >= CPU0_SENSOR_WHEEL_MOVING_RPM_X10) &&
              (left_rpm_x10 * 2 * avoidance.last_cmd_right_rpm <
               right_rpm_x10 * avoidance.last_cmd_left_rpm)));
        avoidance.wheel_stall_ms = (left_stalled || right_stalled || side_imbalance) ?
            avoidance.wheel_stall_ms + elapsed_ms : 0U;
        UW const confirm_ms = side_imbalance ? CPU0_SENSOR_WHEEL_SIDE_STALL_CONFIRM_MS :
                                               CPU0_SENSOR_WHEEL_STALL_CONFIRM_MS;
        if (avoidance.wheel_stall_ms >= confirm_ms) {
            obstacle_avoidance_start_backup(p_output);
            avoidance.last_cmd_left_rpm = p_output->left_rpm;
            avoidance.last_cmd_right_rpm = p_output->right_rpm;
            return;
        }
    } else if (!forward_motion_commanded || (AVOIDANCE_ESCAPE_NONE != avoidance.phase)) {
        avoidance.wheel_stall_ms = 0U;
    }
    if ((AVOIDANCE_ESCAPE_NONE == avoidance.phase) && forward_motion_commanded) {
        BOOL const stall_detected = (linear_speed_mm_s <= CPU0_SENSOR_STUCK_STALL_SPEED_MM_S) &&
                                    (linear_speed_mm_s >= -CPU0_SENSOR_STUCK_STALL_SPEED_MM_S);
        BOOL const tilt_detected = (obstacle_avoidance_abs_i16(
            p_snapshot->accel_mg[CPU0_SENSOR_FORWARD_ACCEL_AXIS]) >= CPU0_SENSOR_STUCK_TILT_THRESH_MG);

        if (stall_detected || tilt_detected) {
            avoidance.stuck_elapsed_ms += elapsed_ms;
        } else {
            avoidance.stuck_elapsed_ms = 0U;
        }

        if (avoidance.stuck_elapsed_ms >= CPU0_SENSOR_STUCK_CONFIRM_MS) {
            avoidance.stuck_elapsed_ms = 0U;
            obstacle_avoidance_start_backup(p_output);
            avoidance.last_cmd_left_rpm = p_output->left_rpm;
            avoidance.last_cmd_right_rpm = p_output->right_rpm;
            return;
        }
    } else {
        avoidance.stuck_elapsed_ms = 0U;
    }

    if (AVOIDANCE_ESCAPE_BLOCKED == avoidance.phase) {
        /* 正面衝突候補による停止は、距離クリアランスで自動再開しない。 */
        obstacle_avoidance_output_stop(CPU0_SENSOR_RULE_BLOCKED_STOP,
                                       CPU0_THINK_STATE_SENSOR_BLOCKED_STOP, p_output);
        return;
    }

    /* 片側に十分な空間(>= 800mm)が開いている場合はポール等の単一障害物であるため、
     * 500mmで停止せず前進操舵すり抜けを優先する。
     * 壁・袋小路(両側 < 800mm)または至近距離(<= 300mm)では通常通り脱出回頭を開始する。 */
    BOOL const wide_open_side = (left_mm >= 800U) || (right_mm >= 800U);
    BOOL const escape_trigger = (!wide_open_side && (center_mm <= CPU0_SENSOR_PIVOT_DISTANCE_MM)) ||
                                (center_mm <= 300U) ||
                               ((left_mm <= CPU0_SENSOR_ESCAPE_SIDE_MM) &&
                                (right_mm <= CPU0_SENSOR_ESCAPE_SIDE_MM) &&
                                (center_mm <= CPU0_SENSOR_ESCAPE_FRONT_MM));
    if (AVOIDANCE_ESCAPE_NONE != avoidance.phase) {
        avoidance.phase_ms += elapsed_ms;
    }
    if ((0 != avoidance.direction) || (AVOIDANCE_ESCAPE_NONE != avoidance.phase)) {
        avoidance.session_ms += elapsed_ms;
    }
    if (AVOIDANCE_ESCAPE_BACKUP == avoidance.phase) {
        if (avoidance.phase_ms >= CPU0_SENSOR_BACKUP_MS) {
            obstacle_avoidance_start_turn(left_mm, right_mm, target_steering_deg, p_output);
        } else {
            obstacle_avoidance_output_backup(p_output);
        }
        return;
    }
    if (AVOIDANCE_ESCAPE_STEER == avoidance.phase) {
        if (avoidance.phase_ms >= CPU0_SENSOR_SETTLE_MS) {
            avoidance.phase = AVOIDANCE_ESCAPE_PIVOT;
            avoidance.phase_ms = 0U;
            avoidance.progress_ms = 0U;
            avoidance.progress_yaw_mdeg = avoidance.yaw_mdeg;
            obstacle_avoidance_output_pivot(TRUE, p_output);
        } else {
            obstacle_avoidance_output_pivot(FALSE, p_output);
        }
        return;
    }
    if (AVOIDANCE_ESCAPE_PIVOT == avoidance.phase) {
        avoidance.progress_ms += elapsed_ms;
        BOOL const turned = (avoidance.yaw_mdeg >= CPU0_SENSOR_PIVOT_MIN_YAW_MDEG) &&
                            (avoidance.phase_ms >= CPU0_SENSOR_PIVOT_MIN_MS);
        if (turned) {
            /* 回頭量だけでピボットを完了し、前方・側面のクリアランス待ちはしない。 */
            avoidance.phase = AVOIDANCE_ESCAPE_NONE;
            avoidance.phase_ms = 0U;
            avoidance.progress_ms = 0U;
            avoidance.progress_yaw_mdeg = avoidance.yaw_mdeg;
        } else {
            BOOL const no_progress = (avoidance.progress_ms >= CPU0_SENSOR_PIVOT_PROGRESS_MS) &&
                (avoidance.yaw_mdeg - avoidance.progress_yaw_mdeg < CPU0_SENSOR_PIVOT_PROGRESS_MDEG);
            if (no_progress || (avoidance.phase_ms >= CPU0_SENSOR_PIVOT_MAX_MS)) {
                obstacle_avoidance_start_backup(p_output);
                return;
            }
            if (avoidance.progress_ms >= CPU0_SENSOR_PIVOT_PROGRESS_MS) {
                avoidance.progress_ms = 0U;
                avoidance.progress_yaw_mdeg = avoidance.yaw_mdeg;
            }
            obstacle_avoidance_output_pivot(TRUE, p_output);
            return;
        }
    }

    /* 前方に障害物がある場合、回避方向をラッチしてすり抜け中の自爆逆ステアを防止。
     * 左右どちらかが十分広い場合は800mmから先行して回避側を確定するため、
     * 直進のまま注意距離へ突入しない。左右ToFが外向き（10〜15度）のため、
     * 正面の単一障害物（ポール等）では左右両側が700mm以上開く。この場合も先行回避する。 */
    BOOL const both_sides_open = (left_mm >= CPU0_SENSOR_CAUTION_DISTANCE_MM) &&
                                 (right_mm >= CPU0_SENSOR_CAUTION_DISTANCE_MM);
    W const side_difference_mm = (left_mm >= right_mm) ?
        (W) left_mm - (W) right_mm : (W) right_mm - (W) left_mm;
    BOOL const early_avoid = (center_mm < CPU0_SENSOR_EARLY_AVOID_DISTANCE_MM) &&
                             ((side_difference_mm >= (W) CPU0_SENSOR_EARLY_AVOID_SIDE_DELTA_MM) ||
                              both_sides_open);
    BOOL const center_obstacle = (center_mm < CPU0_SENSOR_CAUTION_DISTANCE_MM) || early_avoid;
    if ((0 == avoidance.direction) && (center_obstacle || escape_trigger)) {
        avoidance.direction = obstacle_avoidance_choose_escape_direction(left_mm, right_mm,
                                                                          target_steering_deg, 0);
        avoidance.session_active = TRUE;
        avoidance.phase_ms = 0U;
        avoidance.session_ms = 0U;
        avoidance.progress_ms = 0U;
        avoidance.progress_yaw_mdeg = avoidance.yaw_mdeg;
    }

    /* 正面障害物を十分に回避したか確認:
     * - 正面ToFが十分に開いている
     * - 回避セッションで必要な回頭（35度以上）を完了している
     * - 障害物側のToFが十分に開いている（側面に障害物が残っていない）
     */
    BOOL const turned_enough = (avoidance.yaw_mdeg >= CPU0_SENSOR_COMMIT_YAW_MDEG);
    /* 左回避中(direction < 0)は右側に障害物、右回避中(direction > 0)は左側に障害物がある */
    BOOL const obstacle_side_clear = (avoidance.direction < 0) ?
        (right_mm >= CPU0_SENSOR_AVOID_CLEAR_SIDE_MM) :
        (left_mm >= CPU0_SENSOR_AVOID_CLEAR_SIDE_MM);
    BOOL const clearance_safe = turned_enough && obstacle_side_clear &&
                                (center_mm >= CPU0_SENSOR_AVOID_CLEAR_DISTANCE_MM) &&
                                (left_mm >= CPU0_SENSOR_AVOID_CLEAR_SIDE_MM) &&
                                (right_mm >= CPU0_SENSOR_AVOID_CLEAR_SIDE_MM);
    BOOL const bounded_relisten = avoidance.session_active &&
                                  (avoidance.session_ms >= CPU0_SENSOR_AVOID_RELISTEN_MS);
    if ((0 != avoidance.direction) &&
        (clearance_safe || bounded_relisten)) {
        if (avoidance.session_active) {
            avoidance_completed = TRUE;
        }
        avoidance.direction = 0;
        avoidance.yaw_mdeg = 0;
        avoidance.session_active = FALSE;
        avoidance.session_ms = 0U;
        avoidance.recovery_attempts = 0U;
    }
    if (escape_trigger) {
        obstacle_avoidance_start_turn(left_mm, right_mm, target_steering_deg, p_output);
        return;
    }
    if ((0 != avoidance.direction) && (avoidance.yaw_mdeg < CPU0_SENSOR_COMMIT_YAW_MDEG)) {
        avoidance.progress_ms += elapsed_ms;
        BOOL const no_progress = (avoidance.progress_ms >= CPU0_SENSOR_PIVOT_PROGRESS_MS) &&
            (avoidance.yaw_mdeg - avoidance.progress_yaw_mdeg < CPU0_SENSOR_PIVOT_PROGRESS_MDEG);
        /* 前進スタック脱出: 前方に障害物が近接し進行が止まっている場合のみピボットへ移行 */
        if (no_progress && (center_mm <= CPU0_SENSOR_PIVOT_DISTANCE_MM)) {
            obstacle_avoidance_start_turn(left_mm, right_mm, target_steering_deg, p_output);
            return;
        }
        if (avoidance.progress_ms >= CPU0_SENSOR_PIVOT_PROGRESS_MS) {
            avoidance.progress_ms = 0U;
            avoidance.progress_yaw_mdeg = avoidance.yaw_mdeg;
        }
    }

    /* 通常走行では距離に応じて減速・操舵する。 */
    /* 左右ToFの反発力（0〜1000スケール固定小数点） */
    W repulse_left = 0;
    if (left_mm < CPU0_SENSOR_CAUTION_DISTANCE_MM) {
        W const dist = (W) (left_mm < 250U ? 250U : left_mm);
        repulse_left = (((W) CPU0_SENSOR_CAUTION_DISTANCE_MM - dist) * 1000) /
                       ((W) CPU0_SENSOR_CAUTION_DISTANCE_MM - 250);
    }

    W repulse_right = 0;
    if (right_mm < CPU0_SENSOR_CAUTION_DISTANCE_MM) {
        W const dist = (W) (right_mm < 250U ? 250U : right_mm);
        repulse_right = (((W) CPU0_SENSOR_CAUTION_DISTANCE_MM - dist) * 1000) /
                        ((W) CPU0_SENSOR_CAUTION_DISTANCE_MM - 250);
    }

    /* 正面ToFの反発力（0〜1000スケール） */
    W repulse_center = 0;
    if (center_mm < CPU0_SENSOR_CAUTION_DISTANCE_MM) {
        W const dist = (W) (center_mm < 250U ? 250U : center_mm);
        repulse_center = (((W) CPU0_SENSOR_CAUTION_DISTANCE_MM - dist) * 1000) /
                         ((W) CPU0_SENSOR_CAUTION_DISTANCE_MM - 250);
    }

    /* 合成操舵力: 左が近ければ右(+), 右が近ければ左(-), 正面が近ければより広い側へ */
    W steer_force = repulse_left - repulse_right;

    /* S3: target_steering_deg（音源方向）への引力を加算。
     * ただし障害物近接時は、障害物側へ引き戻す逆方向の引力をカットして確実なクリアランスを確保する。 */
    W const attractive_force = ((W) target_steering_deg * 1000) /
                               (W) CPU0_SENSOR_STEERING_MAX_DEG;
    W effective_attractive = attractive_force;
    if ((repulse_left > 200) && (effective_attractive < 0)) {
        effective_attractive = 0;
    } else if ((repulse_right > 200) && (effective_attractive > 0)) {
        effective_attractive = 0;
    }
    steer_force += effective_attractive;

    if (center_mm < CPU0_SENSOR_CAUTION_DISTANCE_MM) {
        W const side_bias = (0 != avoidance.direction) ? avoidance.direction :
                            (target_steering_deg >= 0 ? 1 : -1);
        steer_force += (side_bias * repulse_center * 8) / 10;
    }

    /* 壁が近いうちは、斜めToFの交差だけで目標方向を反転させない。 */
    if ((0 != avoidance.direction) && (steer_force * avoidance.direction < 0)) {
        steer_force = 0;
    }

    /* 目標操舵角（-45度〜+45度）へスケーリング */
    W target_steer_w = (steer_force * (W) CPU0_SENSOR_STEERING_MAX_DEG) / 1000;
    if (target_steer_w > (W) CPU0_SENSOR_STEERING_MAX_DEG) {
        target_steer_w = (W) CPU0_SENSOR_STEERING_MAX_DEG;
    } else if (target_steer_w < -(W) CPU0_SENSOR_STEERING_MAX_DEG) {
        target_steer_w = -(W) CPU0_SENSOR_STEERING_MAX_DEG;
    }
    /* 前向き3眼では壁と平行に近づくほど測距が伸びるため、側面を
     * 見失っても音源側への逆ステアには戻さない。回避開始時の強い操舵は必要な
     * 回頭量または保持時間まで続け、前方がCPU0_SENSOR_PIVOT_DISTANCE_MM未満なら維持する。
     * それ以外は小さい同方向操舵へ緩める。 */
    W const directional_min_steering =
        ((avoidance.yaw_mdeg < CPU0_SENSOR_COMMIT_YAW_MDEG) &&
         (avoidance.session_ms < CPU0_SENSOR_COMMIT_MAX_MS)) ||
        (center_mm < CPU0_SENSOR_PIVOT_DISTANCE_MM) ?
            CPU0_SENSOR_COMMIT_STEERING_DEG : CPU0_SENSOR_COMMIT_HOLD_STEERING_DEG;
    if ((0 != avoidance.direction) &&
        (target_steer_w * avoidance.direction < directional_min_steering)) {
        target_steer_w = avoidance.direction * directional_min_steering;
    }
    H const steering_deg = (H) target_steer_w;

    /* 最も近い障害物に応じた減速 */
    UH min_dist = left_mm;
    if (center_mm < min_dist) { min_dist = center_mm; }
    if (right_mm < min_dist) { min_dist = right_mm; }

    H base_rpm;
    if (min_dist >= 850U) {
        base_rpm = CPU0_SENSOR_FORWARD_RPM;
    } else if (min_dist >= 500U) {
        /* 500〜850mm: 100〜120 RPMへ滑らかに減速 */
        W const range = (W) CPU0_SENSOR_FORWARD_RPM - CPU0_SENSOR_CAUTION_RPM;
        base_rpm = (H) (CPU0_SENSOR_CAUTION_RPM + ((W) (min_dist - 500U) * range) / (850 - 500));
    } else {
        /* 380〜500 mm: 85〜100 RPM。近接時も始動トルクの下限を確保する。 */
        W const dist = (W) (min_dist < CPU0_SENSOR_ESCAPE_SIDE_MM ? CPU0_SENSOR_ESCAPE_SIDE_MM : min_dist);
        W const range = (W) CPU0_SENSOR_CAUTION_RPM - CPU0_SENSOR_MIN_FORWARD_RPM;
        base_rpm = (H) (CPU0_SENSOR_MIN_FORWARD_RPM +
                        ((dist - (W) CPU0_SENSOR_ESCAPE_SIDE_MM) * range) / (500 - (W) CPU0_SENSOR_ESCAPE_SIDE_MM));
    }

    /* 5. 旋回時の差動配分（外輪推進力ブースト ＋ 内輪下限ガードでトルク抜け防止） */
    W const steer_mag = obstacle_avoidance_abs_i16(steering_deg);
    /* 内輪にも始動可能な出力を残す。 */
    W const inner_slowdown = (steer_mag * 25) / CPU0_SENSOR_STEERING_MAX_DEG;
    H inner_rpm = (H) (((W) base_rpm * (100 - inner_slowdown)) / 100);
    if (inner_rpm < CPU0_SENSOR_MIN_FORWARD_RPM) {
        inner_rpm = (H) CPU0_SENSOR_MIN_FORWARD_RPM;
    }

    /* 外輪は舵角に応じて最大10%増速する。 */
    W const outer_boost = (steer_mag * 10) / CPU0_SENSOR_STEERING_MAX_DEG;
    H outer_rpm = (H) (((W) base_rpm * (100 + outer_boost)) / 100);
    if (outer_rpm > 130) {
        outer_rpm = 130;
    }

    H left_rpm;
    H right_rpm;
    sound_follow_state_t state;
    obstacle_avoidance_rule_t rule;

    if (steering_deg < 0) {
        /* 左旋回 */
        left_rpm = inner_rpm;
        right_rpm = outer_rpm;
        state = CPU0_THINK_STATE_SENSOR_TURN_LEFT;
        rule = CPU0_SENSOR_RULE_TURN_LEFT;
    } else if (steering_deg > 0) {
        /* 右旋回 */
        left_rpm = outer_rpm;
        right_rpm = inner_rpm;
        state = CPU0_THINK_STATE_SENSOR_TURN_RIGHT;
        rule = CPU0_SENSOR_RULE_TURN_RIGHT;
    } else {
        /* 直進 */
        left_rpm = base_rpm;
        right_rpm = base_rpm;
        if (base_rpm < CPU0_SENSOR_FORWARD_RPM) {
            state = CPU0_THINK_STATE_SENSOR_CAUTION_FORWARD;
            rule = CPU0_SENSOR_RULE_CAUTION_FORWARD;
        } else {
            state = CPU0_THINK_STATE_SENSOR_FORWARD;
            rule = CPU0_SENSOR_RULE_FORWARD;
        }
    }

    *p_output = (obstacle_avoidance_output_t){
        .state = state,
        .rule = rule,
        .steering_deg = steering_deg,
        .left_rpm = left_rpm,
        .right_rpm = right_rpm,
        .actuator_enable = TRUE,
        .emergency_stop = FALSE,
        .avoidance_in_progress = (AVOIDANCE_ESCAPE_NONE != avoidance.phase) ||
                                 (0 != avoidance.direction),
        .avoidance_completed = avoidance_completed,
    };
    avoidance.last_cmd_left_rpm = left_rpm;
    avoidance.last_cmd_right_rpm = right_rpm;
}
