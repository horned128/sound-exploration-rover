/** =================================================================*
 * @file   sound_follow_controller.c
 * @brief  停止聴取型の音源追従状態機械
 * ================================================================= */
#include "sound_follow_controller.h"                        /* 音源追従の入力、出力、状態 */
#include "config/control_config.h"                          /* 音量閾値、動作時間、走行値 */

/**< 音源追従ステートマシンの内部状態とDoA履歴 */
typedef struct st_sound_follow_context {
    sound_follow_state_t state;                             /**< 現在の追従状態 */
    UW state_elapsed_ms;                                    /**< 状態遷移からの経過時間[ms] */
    UW link_stable_ms;                                      /**< 音響リンク安定時間[ms] */
    UW trigger_elapsed_ms;                                  /**< 音源トリガからの経過時間[ms] */
    UW quiet_elapsed_ms;                                    /**< 無音継続時間[ms] */
    BOOL trigger_active;                                    /**< 音源トリガの有効状態 */
    BOOL desired_is_spin_turn;                              /**< 最小並進回頭を要求する状態 */
    BOOL spin_relisten_pending;                             /**< 連続音の再測定要求 */
    BOOL spin_imu_observed;                                 /**< 有効IMUを受信済み */
    BOOL spin_imu_update_valid;                             /**< 前回IMU更新回数を保持した状態 */
    B spin_direction;                                       /**< 期待ヨー方向（左負、右正） */
    UB doa_sample_count;                                    /**< 蓄積済みDoAサンプル数 */
    H desired_steering_deg;                                 /**< 内部操舵角目標[deg] */
    H desired_left_rpm;                                     /**< 内部左車輪目標RPM */
    H desired_right_rpm;                                    /**< 内部右車輪目標RPM */
    W spin_yaw_mdeg;                                        /**< 期待方向の相対ヨー[mdeg] */
    W spin_target_yaw_mdeg;                                 /**< DoAから決めた今回の目標ヨー[mdeg] */
    W spin_progress_yaw_mdeg;                               /**< 進行確認を始めたヨー[mdeg] */
    UW spin_last_imu_update_count;                          /**< 最後に積分したセンサー更新回数 */
    UW spin_progress_elapsed_ms;                            /**< 進行確認を始めてからの時間[ms] */
    H doa_samples_deg[CPU0_SOUND_DOA_SAMPLE_COUNT];         /**< DoA履歴[deg] */
} sound_follow_context_t;

LOCAL H sound_follow_angle_normalize(W angle_deg);          /* 角度を-180～179度へ正規化 */
LOCAL H sound_follow_angle_delta(H angle_deg, H reference_deg); /* 円周上の符号付き角度差 */
LOCAL H sound_follow_abs_i16(H value);                      /* int16_t絶対値 */
LOCAL BOOL sound_follow_observation_usable(const acoustic_observation_t * p_observation); /* DoA品質判定 */
LOCAL BOOL sound_follow_observation_quiet(const acoustic_observation_t * p_observation); /* release条件判定 */
LOCAL void sound_follow_detection_reset(void);              /* 音量・DoA履歴初期化 */
LOCAL void sound_follow_doa_push(H angle_deg);              /* DoA履歴追加 */
LOCAL BOOL sound_follow_doa_stable(H * p_mean_deg);         /* DoA安定性と平均算出 */
LOCAL H sound_follow_steering_from_doa(H doa_deg);          /* DoAから操舵角算出 */
LOCAL W sound_follow_spin_target_from_doa(H doa_deg);       /* DoAから旋回目標ヨー算出 */
LOCAL void sound_follow_motion_from_doa(H doa_deg);         /* DoAから操舵・走行方向を決定 */
LOCAL void sound_follow_state_enter(sound_follow_state_t state); /* 状態遷移 */
LOCAL void sound_follow_spin_yaw_update(const sound_follow_input_t * p_input, UW elapsed_ms); /* IMU旋回量積分 */
LOCAL void sound_follow_spin_command_update(void);          /* 残ヨーに応じた旋回RPM更新 */
LOCAL void sound_follow_output_update(sound_follow_output_t * p_output); /* 状態から指令生成 */

LOCAL sound_follow_context_t controller;                    /**< 音源追従状態 */

/** =================================================================*
 * @brief  角度を-180～179度へ正規化
 * @param[in] angle_deg 入力角度
 * @return 正規化角度
 * ================================================================= */
LOCAL H sound_follow_angle_normalize(W angle_deg) {
    while (angle_deg >= 180) {
        angle_deg -= 360;
    }
    while (angle_deg < -180) {
        angle_deg += 360;
    }
    return (H) angle_deg;
}

/** =================================================================*
 * @brief  DoAを車体座標へ変換
 * @details 正は右、負は左とし、取付け向きは設定値で補正する。
 * @param[in] doa_deg XVF3800の0～359度DoA
 * @return 車体正面基準の相対角度
 * ================================================================= */
EXPORT H sound_follow_doa_to_relative(UH doa_deg) {
    H angle = sound_follow_angle_normalize((W) doa_deg - CPU0_SOUND_DOA_ZERO_OFFSET_DEG);
    if (0U == CPU0_SOUND_DOA_CLOCKWISE_POSITIVE) {
        angle = (H) -angle;
        angle = sound_follow_angle_normalize(angle);
    }
    return angle;
}

/** =================================================================*
 * @brief  円周上の符号付き角度差
 * @param[in] angle_deg 対象角度
 * @param[in] reference_deg 基準角度
 * @return -180～179度の差
 * ================================================================= */
LOCAL H sound_follow_angle_delta(H angle_deg, H reference_deg) {
    return sound_follow_angle_normalize((W) angle_deg - reference_deg);
}

/** =================================================================*
 * @brief  int16_t絶対値
 * @param[in] value 入力値
 * @return 絶対値
 * ================================================================= */
LOCAL H sound_follow_abs_i16(H value) {
    return (value < 0) ? (H) -value : value;
}

/** =================================================================*
 * @brief  走行判断可能な観測判定
 * @param[in] p_observation 音響観測
 * @return DoA、XVF状態、音声品質が有効ならtrue
 * ================================================================= */
LOCAL BOOL sound_follow_observation_usable(const acoustic_observation_t * p_observation) {
    if ((NULL == p_observation) || (p_observation->doa_deg >= 360U) ||
        (ACOUSTIC_XVF_STATUS_READY != p_observation->xvf_status) ||
        (0U != (p_observation->audio_flags &
                (ACOUSTIC_AUDIO_FLAG_I2C_ERROR | ACOUSTIC_AUDIO_FLAG_MUTED | ACOUSTIC_AUDIO_FLAG_I2S_STALE)))) {
        return FALSE;
    }
    return TRUE;
}

/** =================================================================*
 * @brief  音源追従解除に十分な静音か判定
 * @param[in] p_observation 音響観測
 * @return VADが非検出かつrelease閾値以下ならtrue
 * ================================================================= */
LOCAL BOOL sound_follow_observation_quiet(const acoustic_observation_t * p_observation) {
    if (NULL == p_observation) {
        return TRUE;
    }

    return (0U == p_observation->vad) &&
           (p_observation->level_dbfs_x100 <= CPU0_SOUND_RELEASE_DBFS_X100);
}

/** =================================================================*
 * @brief  音量・DoA履歴初期化
 * ================================================================= */
LOCAL void sound_follow_detection_reset(void) {
    controller.trigger_elapsed_ms = 0U;
    controller.trigger_active = FALSE;
    controller.doa_sample_count = 0U;
}

/** =================================================================*
 * @brief  DoA履歴追加
 * @param[in] angle_deg 車体座標の相対角度
 * ================================================================= */
LOCAL void sound_follow_doa_push(H angle_deg) {
    if (controller.doa_sample_count < CPU0_SOUND_DOA_SAMPLE_COUNT) {
        controller.doa_samples_deg[controller.doa_sample_count++] = angle_deg;
    } else {
        for (UB index = 1U; index < CPU0_SOUND_DOA_SAMPLE_COUNT; index++) {
            controller.doa_samples_deg[index - 1U] = controller.doa_samples_deg[index];
        }
        controller.doa_samples_deg[CPU0_SOUND_DOA_SAMPLE_COUNT - 1U] = angle_deg;
    }
}

/** =================================================================*
 * @brief  DoA安定性と平均算出
 * @param[out] p_mean_deg 円周を考慮した平均角度
 * @return 規定数の角度幅が許容内ならtrue
 * ================================================================= */
LOCAL BOOL sound_follow_doa_stable(H * p_mean_deg) {
    if ((NULL == p_mean_deg) || (controller.doa_sample_count < CPU0_SOUND_DOA_SAMPLE_COUNT)) {
        return FALSE;
    }

    for (UB first = 0U; first < CPU0_SOUND_DOA_SAMPLE_COUNT; first++) {
        for (UB second = (UB) (first + 1U); second < CPU0_SOUND_DOA_SAMPLE_COUNT; second++) {
            H const delta =
                sound_follow_angle_delta(controller.doa_samples_deg[first], controller.doa_samples_deg[second]);
            if (sound_follow_abs_i16(delta) > CPU0_SOUND_DOA_STABLE_WIDTH_DEG) {
                return FALSE;
            }
        }
    }

    H const reference = controller.doa_samples_deg[0];
    W sum = reference;
    for (UB index = 1U; index < CPU0_SOUND_DOA_SAMPLE_COUNT; index++) {
        sum += reference + sound_follow_angle_delta(controller.doa_samples_deg[index], reference);
    }
    *p_mean_deg = sound_follow_angle_normalize(sum / (W) CPU0_SOUND_DOA_SAMPLE_COUNT);
    return TRUE;
}

/** =================================================================*
 * @brief  DoAから操舵角算出
 * @param[in] doa_deg 車体座標の相対角度
 * @return 右正・左負の操舵角
 * ================================================================= */
LOCAL H sound_follow_steering_from_doa(H doa_deg) {
    if (sound_follow_abs_i16(doa_deg) <= CPU0_SOUND_FRONT_TOLERANCE_DEG) {
        return 0;
    }

    H steering = doa_deg;
    if (steering > CPU0_SOUND_STEERING_MAX_DEG) {
        steering = CPU0_SOUND_STEERING_MAX_DEG;
    } else if (steering < -CPU0_SOUND_STEERING_MAX_DEG) {
        steering = -CPU0_SOUND_STEERING_MAX_DEG;
#if (CPU0_SOUND_STEERING_MIN_DEG > 1)
    } else if ((steering > 0) && (steering < CPU0_SOUND_STEERING_MIN_DEG)) {
        steering = CPU0_SOUND_STEERING_MIN_DEG;
    } else if ((steering < 0) && (steering > -CPU0_SOUND_STEERING_MIN_DEG)) {
        steering = -CPU0_SOUND_STEERING_MIN_DEG;
#endif
    }
    return steering;
}

/** =================================================================*
 * @brief  DoAから一回の最小並進回頭目標ヨーを算出
 * @details 前進操舵に渡す残角を残し、一回の目標を90度以内に制限する。
 * @param[in] doa_deg 車体正面基準の相対DoA
 * @return ジャイロZで確認する目標ヨー[mdeg]
 * ================================================================= */
LOCAL W sound_follow_spin_target_from_doa(H doa_deg) {
    W target_yaw_mdeg =
        ((W) sound_follow_abs_i16(doa_deg) - CPU0_SOUND_SPIN_FRONT_RESERVE_DEG) * 1000;
    return (target_yaw_mdeg > CPU0_SOUND_SPIN_MAX_YAW_MDEG) ? CPU0_SOUND_SPIN_MAX_YAW_MDEG :
                                                                target_yaw_mdeg;
}

/** =================================================================*
 * @brief  DoAから操舵と左右モーター指令を決定
 * @details 前方音源では内輪を減速した4輪逆相操舵で前進し、後方音源では
 *          X字操舵と左右逆回転による低速の最小並進回頭を選択する。
 * @param[in] doa_deg 車体正面基準の相対DoA
 * ================================================================= */
LOCAL void sound_follow_motion_from_doa(H doa_deg) {
    if (sound_follow_abs_i16(doa_deg) > CPU0_SOUND_SPIN_THRESHOLD_DEG) {
        controller.desired_steering_deg = 0;
        controller.desired_is_spin_turn = TRUE;
        controller.spin_direction = (doa_deg > 0) ? 1 : -1;
        controller.spin_target_yaw_mdeg = sound_follow_spin_target_from_doa(doa_deg);
        sound_follow_spin_command_update();
        return;
    }

    H const steering_deg = sound_follow_steering_from_doa(doa_deg);
    H left_rpm = CPU0_SOUND_MOVE_LEFT_RPM;
    H right_rpm = CPU0_SOUND_MOVE_RIGHT_RPM;
    W const steering_magnitude = sound_follow_abs_i16(steering_deg);
    if (steering_deg > 0) {
        W const rpm_range = (W) CPU0_SOUND_MOVE_RIGHT_RPM - CPU0_SOUND_TURN_INNER_RPM;
        right_rpm = (H) ((W) CPU0_SOUND_MOVE_RIGHT_RPM -
                         ((rpm_range * steering_magnitude) / CPU0_SOUND_STEERING_MAX_DEG));
    } else if (steering_deg < 0) {
        W const rpm_range = (W) CPU0_SOUND_MOVE_LEFT_RPM - CPU0_SOUND_TURN_INNER_RPM;
        left_rpm = (H) ((W) CPU0_SOUND_MOVE_LEFT_RPM -
                        ((rpm_range * steering_magnitude) / CPU0_SOUND_STEERING_MAX_DEG));
    }

    controller.desired_steering_deg = steering_deg;
    controller.desired_left_rpm = left_rpm;
    controller.desired_right_rpm = right_rpm;
    controller.desired_is_spin_turn = FALSE;
}

/** =================================================================*
 * @brief  状態遷移
 * @param[in] state 遷移先
 * ================================================================= */
LOCAL void sound_follow_state_enter(sound_follow_state_t state) {
    controller.state = state;
    controller.state_elapsed_ms = 0U;
    if (CPU0_THINK_STATE_SPIN_STEP == state) {
        controller.spin_yaw_mdeg = 0;
        controller.spin_progress_yaw_mdeg = 0;
        controller.spin_progress_elapsed_ms = 0U;
        controller.spin_imu_observed = FALSE;
        controller.spin_imu_update_valid = FALSE;
    }
    if ((CPU0_THINK_STATE_LISTEN == state) || (CPU0_THINK_STATE_COOLDOWN == state) ||
        (CPU0_THINK_STATE_SPIN_NO_PROGRESS == state) || (CPU0_THINK_STATE_WAIT_LINK == state) ||
        (CPU0_THINK_STATE_FAULT == state)) {
        controller.spin_relisten_pending = FALSE;
    }
    if ((CPU0_THINK_STATE_LISTEN == state) || (CPU0_THINK_STATE_WAIT_LINK == state) ||
        (CPU0_THINK_STATE_COOLDOWN == state) || (CPU0_THINK_STATE_MOVE_STEP == state) ||
        (CPU0_THINK_STATE_SPIN_STEP == state)) {
        sound_follow_detection_reset();
        controller.quiet_elapsed_ms = 0U;
    }
}

/** =================================================================*
 * @brief  生ジャイロZから最小並進回頭の期待方向ヨー量を積分
 * @details 車輪エンコーダを混ぜたオドメトリは使わず、車体中心の鉛直Z軸だけを
 *          使う。同じセンサー更新を二重積分せず、逆向きの回頭は差し引く。
 * @param[in] p_input 現在の追従入力
 * @param[in] elapsed_ms 前回制御からの経過時間[ms]
 * ================================================================= */
LOCAL void sound_follow_spin_yaw_update(const sound_follow_input_t * p_input, UW elapsed_ms) {
    if ((NULL == p_input) || !p_input->imu_valid || (elapsed_ms > CPU0_SOUND_SPIN_MAX_MS)) {
        return;
    }

    if (controller.spin_imu_update_valid &&
        (controller.spin_last_imu_update_count == p_input->imu_update_count)) {
        return;
    }

    controller.spin_imu_observed = TRUE;
    controller.spin_last_imu_update_count = p_input->imu_update_count;
    controller.spin_imu_update_valid = TRUE;
    H gyro = p_input->gyro_z_dps_x10;
    if (sound_follow_abs_i16(gyro) < CPU0_SENSOR_YAW_DEADBAND_DPS_X10) {
        return;
    }

    controller.spin_yaw_mdeg += (W) gyro * CPU0_SENSOR_YAW_RIGHT_SIGN * (W) controller.spin_direction *
                                (W) elapsed_ms / 10;
    if (controller.spin_yaw_mdeg > 360000) {
        controller.spin_yaw_mdeg = 360000;
    } else if (controller.spin_yaw_mdeg < -360000) {
        controller.spin_yaw_mdeg = -360000;
    }
}

/** =================================================================*
 * @brief  残ヨーに応じた最小並進回頭RPM更新
 * @details 目標へ近づいたら減速し、100 ms制御周期による行き過ぎを抑える。
 * ================================================================= */
LOCAL void sound_follow_spin_command_update(void) {
    W remaining_yaw_mdeg = (controller.spin_target_yaw_mdeg > controller.spin_yaw_mdeg) ?
        controller.spin_target_yaw_mdeg - controller.spin_yaw_mdeg : 0;
    H rpm = (remaining_yaw_mdeg <= CPU0_SOUND_SPIN_SLOWDOWN_MDEG) ? CPU0_SOUND_SPIN_SLOW_RPM :
                                                                           CPU0_SOUND_SPIN_RPM;
    controller.desired_left_rpm = (controller.spin_direction > 0) ? rpm : (H) -rpm;
    controller.desired_right_rpm = (H) -controller.desired_left_rpm;
}

/** =================================================================*
 * @brief  追従状態初期化
 * ================================================================= */
EXPORT void sound_follow_controller_init(void) {
    controller = (sound_follow_context_t){
        .state = CPU0_THINK_STATE_WAIT_LINK,
        .desired_steering_deg = 0,
        .desired_left_rpm = 0,
        .desired_right_rpm = 0,
        .desired_is_spin_turn = FALSE,
    };
}

/** =================================================================*
 * @brief  状態から指令生成
 * @param[out] p_output 最新追従指令
 * ================================================================= */
LOCAL void sound_follow_output_update(sound_follow_output_t * p_output) {
    *p_output = (sound_follow_output_t){
        .state = controller.state,
        .steering_deg = 0,
        .is_spin_turn = FALSE,
        .left_rpm = 0,
        .right_rpm = 0,
        .actuator_enable = TRUE,
        .emergency_stop = FALSE,
    };

    if ((CPU0_THINK_STATE_WAIT_LINK == controller.state) || (CPU0_THINK_STATE_FAULT == controller.state)) {
        p_output->actuator_enable = FALSE;
        p_output->emergency_stop = TRUE;
    } else if (CPU0_THINK_STATE_STEER_PREP == controller.state) {
        p_output->steering_deg = controller.desired_steering_deg;
    } else if (CPU0_THINK_STATE_SPIN_PREP == controller.state) {
        p_output->is_spin_turn = TRUE;
    } else if (CPU0_THINK_STATE_MOVE_STEP == controller.state) {
        p_output->steering_deg = controller.desired_steering_deg;
        p_output->left_rpm = controller.desired_left_rpm;
        p_output->right_rpm = controller.desired_right_rpm;
    } else if (CPU0_THINK_STATE_SPIN_STEP == controller.state) {
        p_output->is_spin_turn = TRUE;
        p_output->left_rpm = controller.desired_left_rpm;
        p_output->right_rpm = controller.desired_right_rpm;
    } else if (CPU0_THINK_STATE_SPIN_NO_PROGRESS == controller.state) {
        /* 停止中も舵角を保ち、ログ上で進行不足を明示する。 */
        p_output->is_spin_turn = TRUE;
    }
}

/** =================================================================*
 * @brief  追従状態更新
 * @details 走行を短時間に限定し、停止後にモーターノイズが収まってから再収音する。
 * @param[in] p_input 音響リンクと最新観測
 * @param[in] elapsed_ms 前回更新からの時間
 * @param[out] p_output 4輪操舵へ展開する追従指令
 * ================================================================= */
EXPORT void sound_follow_controller_step(const sound_follow_input_t * p_input, UW elapsed_ms,
                                  sound_follow_output_t * p_output) {
    if ((NULL == p_input) || (NULL == p_output)) {
        return;
    }

    if (p_input->fault_active) {
        sound_follow_state_enter(CPU0_THINK_STATE_FAULT);
    } else if (!p_input->link_ready) {
        controller.link_stable_ms = 0U;
        controller.quiet_elapsed_ms = 0U;
        sound_follow_state_enter(CPU0_THINK_STATE_WAIT_LINK);
    } else if (p_input->arrived) {
        if (CPU0_THINK_STATE_ARRIVED != controller.state) {
            sound_follow_state_enter(CPU0_THINK_STATE_ARRIVED);
        }
    } else if (p_input->arrival_verify) {
        if (CPU0_THINK_STATE_ARRIVAL_VERIFY != controller.state) {
            sound_follow_state_enter(CPU0_THINK_STATE_ARRIVAL_VERIFY);
        }
    } else if ((CPU0_THINK_STATE_ARRIVAL_VERIFY == controller.state) ||
               (CPU0_THINK_STATE_ARRIVED == controller.state)) {
        sound_follow_state_enter(CPU0_THINK_STATE_LISTEN);
    } else if (CPU0_THINK_STATE_WAIT_LINK == controller.state) {
        controller.link_stable_ms += elapsed_ms;
        if (controller.link_stable_ms >= CPU0_SOUND_LINK_STABLE_MS) {
            controller.link_stable_ms = CPU0_SOUND_LINK_STABLE_MS;
            sound_follow_state_enter(CPU0_THINK_STATE_LISTEN);
        }
    } else if (CPU0_THINK_STATE_FAULT != controller.state) {
        controller.state_elapsed_ms += elapsed_ms;

        if (!p_input->motion_allowed && ((CPU0_THINK_STATE_STEER_PREP == controller.state) ||
                                         (CPU0_THINK_STATE_SPIN_PREP == controller.state))) {
            sound_follow_state_enter(CPU0_THINK_STATE_COOLDOWN);
        } else if (!p_input->motion_allowed && ((CPU0_THINK_STATE_MOVE_STEP == controller.state) ||
                                                 (CPU0_THINK_STATE_SPIN_STEP == controller.state))) {
            sound_follow_state_enter(CPU0_THINK_STATE_SETTLE);
        } else if ((CPU0_THINK_STATE_LISTEN == controller.state) && p_input->new_observation) {
            BOOL const usable = sound_follow_observation_usable(&p_input->observation);
            BOOL const loud = p_input->observation.level_dbfs_x100 >= CPU0_SOUND_TRIGGER_DBFS_X100;
            BOOL const sound_allowed = !p_input->match_required || p_input->target_sound_matched;

            if (!controller.trigger_active) {
                if (usable && loud && (0U != p_input->observation.vad) && sound_allowed) {
                    controller.trigger_active = TRUE;
                    controller.trigger_elapsed_ms = 0U;
                    controller.doa_sample_count = 0U;
                }
            } else if (!usable || !sound_allowed) {
                sound_follow_detection_reset();
            } else {
                controller.trigger_elapsed_ms += elapsed_ms;
                if (controller.trigger_elapsed_ms >= CPU0_SOUND_DOA_SETTLE_MS) {
                    sound_follow_doa_push(sound_follow_doa_to_relative(p_input->observation.doa_deg));
                }
            }

            H mean_doa_deg = 0;
            if (controller.trigger_active && sound_follow_doa_stable(&mean_doa_deg)) {
                sound_follow_motion_from_doa(mean_doa_deg);
                if (p_input->motion_allowed && sound_allowed) {
                    sound_follow_state_enter(controller.desired_is_spin_turn ? CPU0_THINK_STATE_SPIN_PREP :
                                                                             CPU0_THINK_STATE_STEER_PREP);
                } else {
                    sound_follow_state_enter(CPU0_THINK_STATE_COOLDOWN);
                }
            } else if (controller.trigger_active &&
                       (controller.trigger_elapsed_ms >= CPU0_SOUND_DOA_ACQUIRE_TIMEOUT_MS)) {
                sound_follow_detection_reset();
            }
        } else if ((CPU0_THINK_STATE_STEER_PREP == controller.state) &&
                   (controller.state_elapsed_ms >= CPU0_SOUND_STEER_SETTLE_MS)) {
            sound_follow_state_enter(CPU0_THINK_STATE_MOVE_STEP);
        } else if ((CPU0_THINK_STATE_SPIN_PREP == controller.state) &&
                   (controller.state_elapsed_ms >= CPU0_SOUND_STEER_SETTLE_MS)) {
            sound_follow_state_enter(CPU0_THINK_STATE_SPIN_STEP);
        } else if (CPU0_THINK_STATE_MOVE_STEP == controller.state) {
            /* S4: 走行中も音源を追跡し、連続追従を行う（ながら動作） */
            BOOL const sound_allowed = !p_input->match_required || p_input->target_sound_matched;
            if (p_input->navigation_target_valid) {
                /* navigation bearingは操舵角の更新にのみ使用し、spin turnは発動しない。
                 * bearingが後方を示しても前進しながら最大旋回で追従する。推定精度が
                 * 低い段階で不正なspin turnを防止する。 */
                H clamped_bearing = p_input->navigation_bearing_deg;
                if (clamped_bearing > CPU0_SOUND_STEERING_MAX_DEG) {
                    clamped_bearing = CPU0_SOUND_STEERING_MAX_DEG;
                } else if (clamped_bearing < -CPU0_SOUND_STEERING_MAX_DEG) {
                    clamped_bearing = -CPU0_SOUND_STEERING_MAX_DEG;
                }
                controller.desired_steering_deg = sound_follow_steering_from_doa(clamped_bearing);
                controller.desired_is_spin_turn = FALSE;
                controller.quiet_elapsed_ms = 0U;
            }
            if (p_input->new_observation) {
                BOOL const usable = sound_follow_observation_usable(&p_input->observation);
                BOOL const loud = p_input->observation.level_dbfs_x100 >= CPU0_SOUND_TRIGGER_DBFS_X100;
                BOOL const quiet = sound_follow_observation_quiet(&p_input->observation);

                if (usable && loud && (0U != p_input->observation.vad) && sound_allowed) {
                    /* 有効音源の受信中: DoA履歴を更新し操舵角へ追従 */
                    sound_follow_doa_push(sound_follow_doa_to_relative(p_input->observation.doa_deg));
                    H mean_doa_deg = 0;
                    if (sound_follow_doa_stable(&mean_doa_deg)) {
                        sound_follow_motion_from_doa(mean_doa_deg);
                    }
                    controller.quiet_elapsed_ms = 0U;
                } else if (quiet || !sound_allowed) {
                    controller.quiet_elapsed_ms += elapsed_ms;
                }
            } else {
                controller.quiet_elapsed_ms += elapsed_ms;
            }

            if ((controller.quiet_elapsed_ms > 0U) && !p_input->navigation_target_valid) {
                /* 静音中: 直前の大舵角による円運動・壁突進を防ぐため、操舵角を徐々に0°へ復元 */
                if (controller.desired_steering_deg > 0) {
                    controller.desired_steering_deg = (controller.desired_steering_deg > 3) ?
                        (H) (controller.desired_steering_deg - 3) : 0;
                } else if (controller.desired_steering_deg < 0) {
                    controller.desired_steering_deg = (controller.desired_steering_deg < -3) ?
                        (H) (controller.desired_steering_deg + 3) : 0;
                }
                /* 舵角復元に合わせて左右RPMも直進値へ戻す */
                H const cur_steer = controller.desired_steering_deg;
                H left_rpm = CPU0_SOUND_MOVE_LEFT_RPM;
                H right_rpm = CPU0_SOUND_MOVE_RIGHT_RPM;
                W const steer_mag = sound_follow_abs_i16(cur_steer);
                if (cur_steer > 0) {
                    W const rpm_range = (W) CPU0_SOUND_MOVE_RIGHT_RPM - CPU0_SOUND_TURN_INNER_RPM;
                    right_rpm = (H) ((W) CPU0_SOUND_MOVE_RIGHT_RPM -
                                     ((rpm_range * steer_mag) / CPU0_SOUND_STEERING_MAX_DEG));
                } else if (cur_steer < 0) {
                    W const rpm_range = (W) CPU0_SOUND_MOVE_LEFT_RPM - CPU0_SOUND_TURN_INNER_RPM;
                    left_rpm = (H) ((W) CPU0_SOUND_MOVE_LEFT_RPM -
                                    ((rpm_range * steer_mag) / CPU0_SOUND_STEERING_MAX_DEG));
                }
                controller.desired_left_rpm = left_rpm;
                controller.desired_right_rpm = right_rpm;
            }

            /* 音源消失判定: 静音が規定時間継続したら減速停止へ遷移 */
            if (controller.quiet_elapsed_ms >= CPU0_SOUND_LOST_TIMEOUT_MS) {
                sound_follow_state_enter(CPU0_THINK_STATE_SETTLE);
            }
        } else if (CPU0_THINK_STATE_SPIN_STEP == controller.state) {
            sound_follow_spin_yaw_update(p_input, elapsed_ms);
            sound_follow_spin_command_update();
            controller.spin_progress_elapsed_ms += elapsed_ms;
            if (controller.spin_yaw_mdeg >= controller.spin_target_yaw_mdeg) {
                controller.spin_relisten_pending = TRUE;
                sound_follow_state_enter(CPU0_THINK_STATE_SETTLE);
            } else if (controller.state_elapsed_ms >= CPU0_SOUND_SPIN_MAX_MS) {
                controller.spin_relisten_pending = TRUE;
                sound_follow_state_enter(CPU0_THINK_STATE_SETTLE);
            } else if (controller.spin_progress_elapsed_ms >= CPU0_SOUND_SPIN_PROGRESS_MS) {
                if (!controller.spin_imu_observed ||
                    ((controller.spin_yaw_mdeg - controller.spin_progress_yaw_mdeg) <
                     CPU0_SOUND_SPIN_PROGRESS_MDEG)) {
                    sound_follow_state_enter(CPU0_THINK_STATE_SPIN_NO_PROGRESS);
                } else {
                    controller.spin_progress_elapsed_ms = 0U;
                    controller.spin_progress_yaw_mdeg = controller.spin_yaw_mdeg;
                }
            }
        } else if ((CPU0_THINK_STATE_SPIN_NO_PROGRESS == controller.state) &&
                   (controller.state_elapsed_ms >= CPU0_SOUND_SPIN_FAILURE_HOLD_MS)) {
            sound_follow_state_enter(CPU0_THINK_STATE_COOLDOWN);
        } else if ((CPU0_THINK_STATE_SETTLE == controller.state) &&
                   (controller.state_elapsed_ms >= CPU0_SOUND_LISTEN_SETTLE_MS)) {
            /* 成功した後方旋回は、音が続いていても新しいDoAをすぐ測る。 */
            if (controller.spin_relisten_pending) {
                sound_follow_state_enter(CPU0_THINK_STATE_LISTEN);
            } else {
                /* 音源消失後の静定を終えて静音確認へ */
                sound_follow_state_enter(CPU0_THINK_STATE_COOLDOWN);
            }
        } else if ((CPU0_THINK_STATE_COOLDOWN == controller.state) && p_input->new_observation) {
            BOOL const quiet = sound_follow_observation_quiet(&p_input->observation);
            controller.quiet_elapsed_ms = quiet ? controller.quiet_elapsed_ms + elapsed_ms : 0U;
            if (controller.quiet_elapsed_ms >= CPU0_SOUND_COOLDOWN_RELEASE_MS) {
                controller.quiet_elapsed_ms = 0U;
                sound_follow_state_enter(CPU0_THINK_STATE_LISTEN);
            }
        } else if (CPU0_THINK_STATE_COOLDOWN == controller.state) {
            controller.quiet_elapsed_ms = 0U;
        }
    }

    sound_follow_output_update(p_output);
}
