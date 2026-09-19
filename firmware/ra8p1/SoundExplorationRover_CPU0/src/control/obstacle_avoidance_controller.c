/** =================================================================*
 * @file   obstacle_avoidance_controller.c
 * @brief  ToF・IMUによる方向保持付き障害物回避と有限回の切り返し
 * ================================================================= */
#include "obstacle_avoidance_controller.h"                 /* 障害物回避判断API */
#include "config/control_config.h"                         /* 距離、IMU、速度設定 */
#include "config/sensor_config.h"                          /* センサー更新期限 */

typedef enum e_avoidance_escape_phase {
    AVOIDANCE_ESCAPE_NONE = 0,
    AVOIDANCE_ESCAPE_BACKUP,
    AVOIDANCE_ESCAPE_STEER,
    AVOIDANCE_ESCAPE_PIVOT,
    AVOIDANCE_ESCAPE_BLOCKED,
} avoidance_escape_phase_t;

typedef struct st_avoidance_controller {
    avoidance_escape_phase_t phase;
    B direction;
    UB attempts;
    UH backup_start_center_mm;
    UW phase_ms;
    UW clear_ms;
    UW progress_ms;
    W yaw_mdeg;
    W progress_yaw_mdeg;
    UW last_ms;
    UW last_sample_ms;
    UW last_update_count;
    BOOL clock_valid;
    BOOL sample_valid;
} avoidance_controller_t;

LOCAL avoidance_controller_t avoidance;                    /**< 回避方向、回頭量、脱出履歴 */

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
 * @brief  後退または舵角整定中の指令生成
 * @param[in] moving 後退を開始してよければTRUE
 * @param[out] p_output 直進後退指令
 * ================================================================= */
LOCAL void obstacle_avoidance_output_backup(BOOL moving, obstacle_avoidance_output_t * p_output) {
    *p_output = (obstacle_avoidance_output_t){
        .state = CPU0_THINK_STATE_SENSOR_BACKUP,
        .rule = CPU0_SENSOR_RULE_BACKUP,
        .left_rpm = moving ? CPU0_SENSOR_BACKUP_RPM : 0,
        .right_rpm = moving ? CPU0_SENSOR_BACKUP_RPM : 0,
        /* 停車中もサーボを0度へ整定させる。 */
        .actuator_enable = TRUE,
    };
}

/** =================================================================*
 * @brief  最大舵角での前進旋回または舵角整定指令生成
 * @details 左右とも正転する円弧走行。PIVOTという既存のテレメトリ名を維持する。
 * @param[in] moving 整定を終えて旋回を開始してよければTRUE
 * @param[out] p_output 旋回指令
 * ================================================================= */
LOCAL void obstacle_avoidance_output_pivot(BOOL moving, obstacle_avoidance_output_t * p_output) {
    BOOL const left = avoidance.direction < 0;
    *p_output = (obstacle_avoidance_output_t){
        .state = left ? CPU0_THINK_STATE_SENSOR_PIVOT_LEFT : CPU0_THINK_STATE_SENSOR_PIVOT_RIGHT,
        .rule = left ? CPU0_SENSOR_RULE_PIVOT_LEFT : CPU0_SENSOR_RULE_PIVOT_RIGHT,
        .steering_deg = left ? -CPU0_SENSOR_STEERING_MAX_DEG : CPU0_SENSOR_STEERING_MAX_DEG,
        .left_rpm = moving ? (left ? CPU0_SENSOR_PIVOT_INNER_RPM : CPU0_SENSOR_PIVOT_OUTER_RPM) : 0,
        .right_rpm = moving ? (left ? CPU0_SENSOR_PIVOT_OUTER_RPM : CPU0_SENSOR_PIVOT_INNER_RPM) : 0,
        .actuator_enable = TRUE,
    };
}

/** =================================================================*
 * @brief  再試行上限による停止
 * @param[out] p_output 停止指令
 * ================================================================= */
LOCAL void obstacle_avoidance_block(obstacle_avoidance_output_t * p_output) {
    avoidance.phase = AVOIDANCE_ESCAPE_BLOCKED;
    avoidance.clear_ms = 0U;
    obstacle_avoidance_output_stop(CPU0_SENSOR_RULE_BLOCKED_STOP, CPU0_THINK_STATE_SENSOR_BLOCKED_STOP, p_output);
}

/** =================================================================*
 * @brief  切り返し開始
 * @param[in] center_mm 後退前の正面距離[mm]
 * @param[in] change_direction 回頭できなかった方向を変更するならTRUE
 * @param[out] p_output 後退準備または上限停止指令
 * ================================================================= */
LOCAL void obstacle_avoidance_start_backup(UH center_mm, BOOL change_direction,
                                          obstacle_avoidance_output_t * p_output) {
    if (avoidance.attempts >= CPU0_SENSOR_ESCAPE_MAX_ATTEMPTS) {
        obstacle_avoidance_block(p_output);
        return;
    }
    if (change_direction) {
        avoidance.direction = (B) -avoidance.direction;
        avoidance.yaw_mdeg = 0;
    }
    avoidance.attempts++;
    avoidance.phase = AVOIDANCE_ESCAPE_BACKUP;
    avoidance.phase_ms = 0U;
    avoidance.clear_ms = 0U;
    avoidance.backup_start_center_mm = center_mm;
    obstacle_avoidance_output_backup(FALSE, p_output);
}

/** =================================================================*
 * @brief  回避状態初期化
 * ================================================================= */
EXPORT void obstacle_avoidance_controller_init(void) {
    avoidance = (avoidance_controller_t){0};
}

/** =================================================================*
 * @brief  ToFとIMUによる障害物回避指令の決定
 * @details now_msは単調増加時計の下位32 bit。繰り返し観測を回頭量へ二重加算しない。
 * @param[in] p_snapshot 最新センサー値。NULLは安全停止
 * @param[in] fault_active 異常・走行禁止状態
 * @param[in] now_ms 呼出側の実時刻[ms]
 * @param[in] target_steering_deg 音源方向への引力として用いる目標操舵角[°]
 * @param[out] p_output 走行指令
 * ================================================================= */
EXPORT void obstacle_avoidance_controller_step(const sensor_snapshot_t * p_snapshot, BOOL fault_active,
                                               UW now_ms, H target_steering_deg,
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
    UH const side_min_mm = (left_mm < right_mm) ? left_mm : right_mm;
    BOOL const corridor_clear = (center_mm >= CPU0_SENSOR_CLEAR_DISTANCE_MM) &&
                               (side_min_mm >= CPU0_SENSOR_SIDE_DISTANCE_MM);

    /* いずれかの光軸で極近接を検知したら、旋回の継続も許可しない。 */
    if ((center_mm <= CPU0_SENSOR_CRITICAL_STOP_DISTANCE_MM) ||
        (side_min_mm <= CPU0_SENSOR_CRITICAL_STOP_DISTANCE_MM)) {
        obstacle_avoidance_block(p_output);
        return;
    }

    if (AVOIDANCE_ESCAPE_BLOCKED == avoidance.phase) {
        avoidance.clear_ms = (corridor_clear && new_sample) ? avoidance.clear_ms + elapsed_ms : 0U;
        if (avoidance.clear_ms < CPU0_SENSOR_REARM_CLEAR_MS) {
            obstacle_avoidance_output_stop(CPU0_SENSOR_RULE_BLOCKED_STOP, CPU0_THINK_STATE_SENSOR_BLOCKED_STOP, p_output);
            return;
        }
        obstacle_avoidance_controller_init();
    }

    BOOL const escape_trigger = (center_mm <= CPU0_SENSOR_PIVOT_DISTANCE_MM) ||
                               ((left_mm <= CPU0_SENSOR_ESCAPE_SIDE_MM) &&
                                (right_mm <= CPU0_SENSOR_ESCAPE_SIDE_MM) &&
                                (center_mm <= CPU0_SENSOR_ESCAPE_FRONT_MM));
    if (AVOIDANCE_ESCAPE_NONE != avoidance.phase) {
        avoidance.phase_ms += elapsed_ms;
    }
    if (AVOIDANCE_ESCAPE_BACKUP == avoidance.phase) {
        BOOL const clearance = (center_mm >= CPU0_SENSOR_BACKUP_CLEAR_MM) &&
                               (side_min_mm >= CPU0_SENSOR_SIDE_DISTANCE_MM) &&
                               ((W) center_mm - avoidance.backup_start_center_mm >= (W) CPU0_SENSOR_BACKUP_GAIN_MM);
        if ((avoidance.phase_ms >= CPU0_SENSOR_SETTLE_MS + CPU0_SENSOR_BACKUP_MIN_MS) && clearance) {
            avoidance.phase = AVOIDANCE_ESCAPE_STEER;
            avoidance.phase_ms = 0U;
            obstacle_avoidance_output_pivot(FALSE, p_output);
        } else if (avoidance.phase_ms >= CPU0_SENSOR_SETTLE_MS + CPU0_SENSOR_BACKUP_MAX_MS) {
            /* 後方センサーがないため、距離を稼げないまま後退を延長しない。 */
            obstacle_avoidance_block(p_output);
        } else {
            obstacle_avoidance_output_backup(avoidance.phase_ms >= CPU0_SENSOR_SETTLE_MS, p_output);
        }
        return;
    }
    if (AVOIDANCE_ESCAPE_STEER == avoidance.phase) {
        if (escape_trigger || (side_min_mm <= CPU0_SENSOR_ESCAPE_SIDE_MM)) {
            obstacle_avoidance_start_backup(center_mm, FALSE, p_output);
        } else if (avoidance.phase_ms >= CPU0_SENSOR_SETTLE_MS) {
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
        UH const exit_side_mm = (avoidance.direction < 0) ? left_mm : right_mm;
        BOOL const turned = (avoidance.yaw_mdeg >= CPU0_SENSOR_PIVOT_MIN_YAW_MDEG) &&
                            (avoidance.phase_ms >= CPU0_SENSOR_PIVOT_MIN_MS);
        BOOL const exit_clear = turned && (center_mm >= CPU0_SENSOR_PIVOT_CLEAR_DISTANCE_MM) &&
                                (exit_side_mm >= CPU0_SENSOR_SIDE_DISTANCE_MM) &&
                                (side_min_mm >= CPU0_SENSOR_ESCAPE_SIDE_MM);
        avoidance.clear_ms = (exit_clear && new_sample) ? avoidance.clear_ms + elapsed_ms : 0U;
        if (escape_trigger || (side_min_mm <= CPU0_SENSOR_ESCAPE_SIDE_MM)) {
            /* 再接近時も回頭の向きと累積角を保ち、切り返しで少しずつ壁から向きを変える。 */
            obstacle_avoidance_start_backup(center_mm, FALSE, p_output);
            return;
        }
        if (avoidance.clear_ms >= CPU0_SENSOR_CLEAR_HOLD_MS) {
            avoidance.phase = AVOIDANCE_ESCAPE_NONE;
            avoidance.phase_ms = 0U;
            avoidance.clear_ms = 0U;
            avoidance.progress_ms = 0U;
            avoidance.progress_yaw_mdeg = avoidance.yaw_mdeg;
        } else {
            BOOL const no_progress = (avoidance.progress_ms >= CPU0_SENSOR_PIVOT_PROGRESS_MS) &&
                (avoidance.yaw_mdeg - avoidance.progress_yaw_mdeg < CPU0_SENSOR_PIVOT_PROGRESS_MDEG);
            if (no_progress || (avoidance.phase_ms >= CPU0_SENSOR_PIVOT_MAX_MS)) {
                obstacle_avoidance_start_backup(center_mm, TRUE, p_output);
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

    /* 開けた進路を一定時間走るまでは成功履歴を消さず、短周期の再脱出にも上限を適用する。 */
    BOOL const heading_clear = (0 == avoidance.direction) || (avoidance.yaw_mdeg >= CPU0_SENSOR_COMMIT_YAW_MDEG);
    avoidance.clear_ms = (corridor_clear && heading_clear && new_sample) ? avoidance.clear_ms + elapsed_ms : 0U;
    if (avoidance.clear_ms >= CPU0_SENSOR_REARM_CLEAR_MS) {
        avoidance.direction = 0;
        avoidance.attempts = 0U;
        avoidance.yaw_mdeg = 0;
        avoidance.clear_ms = CPU0_SENSOR_REARM_CLEAR_MS;
    }
    if ((0 == avoidance.direction) && (center_mm < CPU0_SENSOR_PIVOT_CLEAR_DISTANCE_MM || escape_trigger)) {
        /* 初回だけ遠い側を選び、以後の微小差で選び直さない。 */
        avoidance.direction = (left_mm >= right_mm) ? -1 : 1;
        avoidance.phase_ms = 0U;
        avoidance.progress_ms = 0U;
        avoidance.progress_yaw_mdeg = avoidance.yaw_mdeg;
    }
    if (escape_trigger) {
        obstacle_avoidance_start_backup(center_mm, FALSE, p_output);
        return;
    }
    if ((0 != avoidance.direction) && (avoidance.yaw_mdeg < CPU0_SENSOR_COMMIT_YAW_MDEG)) {
        avoidance.phase_ms += elapsed_ms;
        avoidance.progress_ms += elapsed_ms;
        BOOL const no_progress = (avoidance.progress_ms >= CPU0_SENSOR_PIVOT_PROGRESS_MS) &&
            (avoidance.yaw_mdeg - avoidance.progress_yaw_mdeg < CPU0_SENSOR_PIVOT_PROGRESS_MDEG);
        if (no_progress || (avoidance.phase_ms >= CPU0_SENSOR_COMMIT_MAX_MS)) {
            obstacle_avoidance_start_backup(center_mm, TRUE, p_output);
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
    if (left_mm < 850U) {
        W const dist = (W) (left_mm < 250U ? 250U : left_mm);
        repulse_left = ((850 - dist) * 1000) / (850 - 250);
    }

    W repulse_right = 0;
    if (right_mm < 850U) {
        W const dist = (W) (right_mm < 250U ? 250U : right_mm);
        repulse_right = ((850 - dist) * 1000) / (850 - 250);
    }

    /* 正面ToFの反発力（0〜1000スケール） */
    W repulse_center = 0;
    if (center_mm < 950U) {
        W const dist = (W) (center_mm < 250U ? 250U : center_mm);
        repulse_center = ((950 - dist) * 1000) / (950 - 250);
    }

    /* 合成操舵力: 左が近ければ右(+), 右が近ければ左(-), 正面が近ければより広い側へ */
    W steer_force = repulse_left - repulse_right;

    /* S3: target_steering_deg（音源方向）への引力を加算 */
    W const attractive_force = ((W) target_steering_deg * 1000) /
                               (W) CPU0_SENSOR_STEERING_MAX_DEG;
    steer_force += attractive_force;

    if (center_mm < 900U) {
        W const side_bias = (0 != avoidance.direction) ? avoidance.direction :
                            (target_steering_deg >= 0 ? 1 : -1);
        steer_force += (side_bias * repulse_center * 15) / 10;
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
    /* 前向き3眼では壁と平行に近づくほど測距が伸びる。側面を見失って直進に戻さない。 */
    if ((0 != avoidance.direction) && (avoidance.yaw_mdeg < CPU0_SENSOR_COMMIT_YAW_MDEG) &&
        (target_steer_w * avoidance.direction < CPU0_SENSOR_COMMIT_STEERING_DEG)) {
        target_steer_w = avoidance.direction * CPU0_SENSOR_COMMIT_STEERING_DEG;
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
    };
}
