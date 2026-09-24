/** =================================================================*
 * @file   sound_follow_controller.c
 * @brief  音源追従状態機械
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
    BOOL move_relisten_pending;                             /**< 短距離移動後の再測定要求 */
    BOOL avoidance_reorient_pending;                        /**< 回避後に音源へ向き直す要求 */
    UB avoidance_reorient_attempts;                         /**< 回避後の向き直し試行回数 */
    BOOL spin_imu_observed;                                 /**< 有効IMUを受信済み */
    BOOL spin_imu_update_valid;                             /**< 前回IMU更新回数を保持した状態 */
    BOOL spin_doa_valid;                                    /**< 旋回中DoAの前回値を保持済み */
    B spin_direction;                                       /**< 期待ヨー方向（左負、右正） */
    UB doa_sample_count;                                    /**< 蓄積済みDoAサンプル数 */
    UB spin_doa_jump_count;                                 /**< 大きなDoA変化の連続確認数 */
    H desired_steering_deg;                                 /**< 内部操舵角目標[deg] */
    H target_bearing_deg;                                   /**< 静止中に確定した本来のDoA[deg] */
    H desired_left_rpm;                                     /**< 内部左車輪目標RPM */
    H desired_right_rpm;                                    /**< 内部右車輪目標RPM */
    W spin_yaw_mdeg;                                        /**< 期待方向の相対ヨー[mdeg] */
    W spin_target_yaw_mdeg;                                 /**< DoAから決めた今回の目標ヨー[mdeg] */
    W spin_progress_yaw_mdeg;                               /**< 進行確認を始めたヨー[mdeg] */
    UW spin_last_imu_update_count;                          /**< 最後に積分したセンサー更新回数 */
    UW spin_progress_elapsed_ms;                            /**< 進行確認を始めてからの時間[ms] */
    H spin_last_doa_deg;                                    /**< 旋回中の前回DoA（車体相対、右正） */
    H spin_doa_jump_candidate_deg;                          /**< 大きく変化したDoAの候補[deg] */
    BOOL motion_doa_reference_valid;                        /**< 移動中DoAの姿勢基準を取得済み */
    H motion_doa_reference_deg;                             /**< 最後に採用した車体相対DoA[deg] */
    W motion_heading_reference_mrad;                        /**< その時の車体方位[mrad] */
    H doa_samples_deg[CPU0_SOUND_DOA_SAMPLE_COUNT];         /**< DoA履歴[deg] */
} sound_follow_context_t;

LOCAL H sound_follow_angle_normalize(W angle_deg);          /* 角度を-180～179度へ正規化 */
LOCAL H sound_follow_angle_delta(H angle_deg, H reference_deg); /* 円周上の符号付き角度差 */
LOCAL H sound_follow_abs_i16(H value);                      /* int16_t絶対値 */
LOCAL H sound_follow_rear_seam_select(H doa_deg, B preferred_direction); /* 真後ろの旋回側を確定 */
LOCAL BOOL sound_follow_observation_usable(const acoustic_observation_t * p_observation); /* DoA品質判定 */
LOCAL BOOL sound_follow_observation_trackable(const acoustic_observation_t * p_observation); /* 走行に採用できるDoA品質判定 */
LOCAL BOOL sound_follow_observation_quiet(const acoustic_observation_t * p_observation); /* release条件判定 */
LOCAL void sound_follow_detection_reset(void);              /* 音量・DoA履歴初期化 */
LOCAL void sound_follow_doa_push(H angle_deg);              /* DoA履歴追加 */
LOCAL BOOL sound_follow_doa_stable(H * p_mean_deg);         /* DoA安定性と平均算出 */
LOCAL H sound_follow_moving_doa_select(H doa_deg, const sound_follow_input_t * p_input); /* 姿勢で移動中DoAを検証 */
LOCAL H sound_follow_steering_from_doa(H doa_deg);          /* DoAから操舵角算出 */
LOCAL W sound_follow_spin_target_from_doa(H doa_deg);       /* DoAから旋回目標ヨー算出 */
LOCAL void sound_follow_motion_from_doa(H doa_deg);         /* DoAから操舵・走行方向を決定 */
LOCAL void sound_follow_state_enter(sound_follow_state_t state); /* 状態遷移 */
LOCAL void sound_follow_spin_yaw_update(const sound_follow_input_t * p_input, UW elapsed_ms); /* IMU旋回量積分 */
LOCAL void sound_follow_spin_command_update(void);          /* 残ヨーに応じた旋回RPM更新 */
LOCAL BOOL sound_follow_spin_doa_update(const sound_follow_input_t * p_input); /* DoAで正面到達を監視 */
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
 * @brief  走行中DoAの姿勢照合
 * @details 音源が静止しているなら、左へ回頭した分だけ相対DoAは右へ移る。
 *          近距離の並進によるずれは設定した許容差内で受け入れる。姿勢が一時的に
 *          無効になった場合は新しいDoAを採用しない。
 * @param[in] doa_deg 最新の車体相対DoA[deg]
 * @param[in] p_input オドメトリ姿勢を含む追従入力
 * @return 姿勢予測で妥当性を確認した車体相対DoA[deg]
 * ================================================================= */
LOCAL H sound_follow_moving_doa_select(H doa_deg, const sound_follow_input_t * p_input) {
    if (NULL == p_input) {
        return doa_deg;
    }
    if (!p_input->pose_heading_valid) {
        return controller.motion_doa_reference_valid ? controller.target_bearing_deg : doa_deg;
    }
    if (!controller.motion_doa_reference_valid) {
        controller.motion_doa_reference_valid = TRUE;
        controller.motion_doa_reference_deg = controller.target_bearing_deg;
        controller.motion_heading_reference_mrad = p_input->pose_heading_mrad;
    }

    W delta_mrad = p_input->pose_heading_mrad - controller.motion_heading_reference_mrad;
    /* mradの方位差を±π（3142 mrad）内へ折り返してから度へ換算する。 */
    while (delta_mrad > 3142) {
        delta_mrad -= 6283;
    }
    while (delta_mrad < -3142) {
        delta_mrad += 6283;
    }
    W const yaw_deg = (delta_mrad * 180) / 3142;
    H const predicted_doa_deg = sound_follow_angle_normalize(
        (W) controller.motion_doa_reference_deg - yaw_deg);
    if (sound_follow_abs_i16(sound_follow_angle_delta(doa_deg, predicted_doa_deg)) >
        CPU0_SOUND_MOVE_DOA_INNOVATION_DEG) {
        return predicted_doa_deg;
    }
    controller.motion_doa_reference_deg = doa_deg;
    controller.motion_heading_reference_mrad = p_input->pose_heading_mrad;
    return doa_deg;
}

/** =================================================================*
 * @brief  DoAを車体座標へ変換
 * @details 正は右、負は左。四方実測で後方だけXVFの左右対応が
 *          前方と逆だったため、前方・後方を分けて較正する。
 * @param[in] doa_deg XVF3800の0～359度DoA
 * @return 車体正面基準の相対角度
 * ================================================================= */
EXPORT H sound_follow_doa_to_relative(UH doa_deg) {
    H const measured_angle = sound_follow_angle_normalize((W) doa_deg - CPU0_SOUND_DOA_ZERO_OFFSET_DEG);
    H angle = measured_angle;
    if (0U == CPU0_SOUND_DOA_CLOCKWISE_POSITIVE) {
        angle = (H) -angle;
    }
    if ((0U != CPU0_SOUND_DOA_REAR_LR_SWAP) &&
        (sound_follow_abs_i16(measured_angle) > 90)) {
        angle = (H) -angle;
    }
    return sound_follow_angle_normalize(angle);
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
 * @brief  真後ろ音源の旋回側選択
 * @details ±180°は物理的にはほぼ同方向。ToFで十分な左右差がある場合だけ
 *          旋回側を選ぶ。正面・斜め後方の明瞭な左右は変更しない。
 * @param[in] doa_deg 車体相対DoA[deg]
 * @param[in] preferred_direction 安全側（右+1、左-1、不明0）
 * @return 旋回側へ正規化した車体相対DoA[deg]
 * ================================================================= */
LOCAL H sound_follow_rear_seam_select(H doa_deg, B preferred_direction) {
    if ((0 == preferred_direction) ||
        (sound_follow_abs_i16(doa_deg) < CPU0_SOUND_REAR_SEAM_START_DEG)) {
        return doa_deg;
    }
    H const magnitude = sound_follow_abs_i16(doa_deg);
    return (preferred_direction > 0) ? ((magnitude == 180) ? 179 : magnitude) : (H) -magnitude;
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
 * @brief  走行判断に十分な品質のDoAか判定
 * @details XVF3800のフォールバックDoAはVADが断続的に0になるため、VADでは
 *          除外しない。学習済み音の一致と循環DoA品質で誤検知を防ぐ。
 * @param[in] p_observation 音響観測
 * @return 走行DoAとして採用できるならtrue
 * ================================================================= */
LOCAL BOOL sound_follow_observation_trackable(const acoustic_observation_t * p_observation) {
    return sound_follow_observation_usable(p_observation) &&
           (p_observation->raw_doa_deg < 360U) &&
           (p_observation->doa_confidence >= CPU0_SOUND_TRACK_MIN_CONFIDENCE) &&
           (sound_follow_abs_i16(sound_follow_angle_delta((H) p_observation->doa_deg,
                                                          (H) p_observation->raw_doa_deg)) <=
            CPU0_SOUND_RAW_FILTER_MAX_DELTA_DEG);
}

/** =================================================================*
 * @brief  音源追従解除に十分な静音か判定
 * @param[in] p_observation 音響観測
 * @return release閾値以下ならtrue
 * ================================================================= */
LOCAL BOOL sound_follow_observation_quiet(const acoustic_observation_t * p_observation) {
    if (NULL == p_observation) {
        return TRUE;
    }

    return p_observation->level_dbfs_x100 <= CPU0_SOUND_RELEASE_DBFS_X100;
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
 * @details 前進操舵へ残角を残し、一回の目標を設定した上限以内に制限する。
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
    controller.target_bearing_deg = doa_deg;
    /* 回避ピボットは空き側へ出ることが目的で、音源を向くこととは分ける。
     * 回避完了後は新しく静止取得したDoAが正面域に収まるまで、最大二回だけ
     * 左右符号へ向き直してから前進する。試行回数を制限して無限旋回を防ぐ。 */
    BOOL const outside_reorient_front = sound_follow_abs_i16(doa_deg) >
                                      CPU0_SOUND_AVOID_REORIENT_THRESHOLD_DEG;
    BOOL const reorient_after_avoidance = controller.avoidance_reorient_pending &&
                                          outside_reorient_front &&
                                          (controller.avoidance_reorient_attempts <
                                           CPU0_SOUND_AVOID_REORIENT_MAX_ATTEMPTS);
    if (reorient_after_avoidance) {
        controller.avoidance_reorient_attempts++;
    } else if (controller.avoidance_reorient_pending) {
        controller.avoidance_reorient_pending = FALSE;
        controller.avoidance_reorient_attempts = 0U;
    }
    if ((sound_follow_abs_i16(doa_deg) > CPU0_SOUND_SPIN_THRESHOLD_DEG) ||
        reorient_after_avoidance) {
        controller.desired_steering_deg = 0;
        controller.desired_is_spin_turn = TRUE;
        controller.spin_direction = (doa_deg > 0) ? 1 : -1;
        controller.spin_target_yaw_mdeg = sound_follow_spin_target_from_doa(doa_deg);
        controller.spin_doa_valid = TRUE;
        controller.spin_last_doa_deg = doa_deg;
        controller.spin_doa_jump_count = 0U;
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
    if (CPU0_THINK_STATE_MOVE_STEP == state) {
        controller.motion_doa_reference_valid = FALSE;
    }
    if ((CPU0_THINK_STATE_LISTEN == state) || (CPU0_THINK_STATE_COOLDOWN == state) ||
        (CPU0_THINK_STATE_SPIN_NO_PROGRESS == state) || (CPU0_THINK_STATE_WAIT_LINK == state) ||
        (CPU0_THINK_STATE_FAULT == state) || (CPU0_THINK_STATE_WAIT_RESTART == state)) {
        controller.spin_relisten_pending = FALSE;
        controller.move_relisten_pending = FALSE;
        if (CPU0_THINK_STATE_LISTEN != state) {
            controller.avoidance_reorient_pending = FALSE;
            controller.avoidance_reorient_attempts = 0U;
        }
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
 * @brief  最小並進旋回中の正面到達判定
 * @details IMUは旋回進行と上限を監視する。壁の反射や前後境界でDoAが
 *          揺れても駆動方向は変えず、安定して正面に来たときだけ終了する。
 * @param[in] p_input 最新の音響入力
 * @return DoAで正面域へ到達して旋回を終了した場合TRUE
 * ================================================================= */
LOCAL BOOL sound_follow_spin_doa_update(const sound_follow_input_t * p_input) {
    if ((NULL == p_input) || !p_input->new_observation ||
        !sound_follow_observation_trackable(&p_input->observation) ||
        (p_input->observation.level_dbfs_x100 < CPU0_SOUND_TRIGGER_DBFS_X100)) {
        return FALSE;
    }

    H const doa_deg = sound_follow_rear_seam_select(
        sound_follow_doa_to_relative(p_input->observation.doa_deg), controller.spin_direction);
    if (controller.spin_doa_valid) {
        H const delta_deg = sound_follow_angle_delta(doa_deg, controller.spin_last_doa_deg);
        /* 前後較正の境界や反射で一周期に大きく跳ねたDoAでは、
         * 正面到達判定をしない。IMUの旋回上限は維持する。 */
        if (sound_follow_abs_i16(delta_deg) > CPU0_SOUND_SPIN_DOA_MAX_STEP_DEG) {
            controller.doa_sample_count = 0U;
            if ((controller.spin_doa_jump_count > 0U) &&
                (sound_follow_abs_i16(sound_follow_angle_delta(doa_deg,
                    controller.spin_doa_jump_candidate_deg)) <= CPU0_SOUND_DOA_STABLE_WIDTH_DEG)) {
                controller.spin_doa_jump_count++;
            } else {
                controller.spin_doa_jump_count = 1U;
            }
            controller.spin_doa_jump_candidate_deg = doa_deg;
            if (controller.spin_doa_jump_count >= CPU0_SOUND_SPIN_DOA_JUMP_CONFIRM) {
                /* 持続した新方位は正面到達判定の新しい基準にする。 */
                controller.spin_last_doa_deg = doa_deg;
                controller.spin_doa_jump_count = 0U;
                sound_follow_doa_push(doa_deg);
            }
            return FALSE;
        }
        controller.spin_doa_jump_count = 0U;
    }
    controller.spin_last_doa_deg = doa_deg;
    controller.spin_doa_valid = TRUE;

    /* 5点の安定DoAで正面域を確認してから停止する。単発の反射や回転中の
     * 平滑化遅れで早期停止しないよう、LISTENと同じ円周安定判定を使う。 */
    sound_follow_doa_push(doa_deg);
    H stable_doa_deg = 0;
    return sound_follow_doa_stable(&stable_doa_deg) &&
           (sound_follow_abs_i16(stable_doa_deg) <= CPU0_SOUND_SPIN_DOA_FRONT_DEG);
}

/** =================================================================*
 * @brief  追従状態初期化
 * ================================================================= */
EXPORT void sound_follow_controller_init(void) {
    controller = (sound_follow_context_t){
        .state = CPU0_THINK_STATE_WAIT_LINK,
        .desired_steering_deg = 0,
        .target_bearing_deg = 0,
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
        .target_bearing_deg = controller.target_bearing_deg,
        .is_spin_turn = FALSE,
        .left_rpm = 0,
        .right_rpm = 0,
        .actuator_enable = FALSE,
        .emergency_stop = FALSE,
    };

    if ((CPU0_THINK_STATE_WAIT_LINK == controller.state) || (CPU0_THINK_STATE_FAULT == controller.state)) {
        p_output->actuator_enable = FALSE;
        p_output->emergency_stop = TRUE;
    } else if (CPU0_THINK_STATE_STEER_PREP == controller.state) {
        p_output->steering_deg = controller.desired_steering_deg;
        p_output->actuator_enable = TRUE;
    } else if (CPU0_THINK_STATE_SPIN_PREP == controller.state) {
        p_output->is_spin_turn = TRUE;
        p_output->actuator_enable = TRUE;
    } else if (CPU0_THINK_STATE_MOVE_STEP == controller.state) {
        p_output->steering_deg = controller.desired_steering_deg;
        p_output->is_spin_turn = controller.desired_is_spin_turn;
        p_output->left_rpm = controller.desired_left_rpm;
        p_output->right_rpm = controller.desired_right_rpm;
        p_output->actuator_enable = TRUE;
    } else if (CPU0_THINK_STATE_SPIN_STEP == controller.state) {
        p_output->is_spin_turn = TRUE;
        p_output->left_rpm = controller.desired_left_rpm;
        p_output->right_rpm = controller.desired_right_rpm;
        p_output->actuator_enable = TRUE;
    } else if (CPU0_THINK_STATE_SPIN_NO_PROGRESS == controller.state) {
        /* 出力を止め、ログ上で進行不足を明示する。 */
        p_output->is_spin_turn = TRUE;
    }
}

/** =================================================================*
 * @brief  追従状態更新
 * @details 対象音とセンサーの許可が続く間追従し、旋回・回避・音源喪失後は停止して再収音する。
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
    } else if (p_input->restart_request) {
        sound_follow_detection_reset();
        controller.quiet_elapsed_ms = 0U;
        controller.avoidance_reorient_pending = FALSE;
        controller.avoidance_reorient_attempts = 0U;
        sound_follow_state_enter(CPU0_THINK_STATE_LISTEN);
    } else if (p_input->avoidance_relisten) {
        /* 回避中の車体姿勢・モーター雑音で得た方位は使い回さない。 */
        sound_follow_detection_reset();
        controller.quiet_elapsed_ms = 0U;
        controller.move_relisten_pending = TRUE;
        controller.avoidance_reorient_pending = TRUE;
        controller.avoidance_reorient_attempts = 0U;
        sound_follow_state_enter(CPU0_THINK_STATE_SETTLE);
    } else if (CPU0_THINK_STATE_WAIT_RESTART == controller.state) {
        /* 停止シーケンス後は、外部からの明示再開まで停止を保つ。 */
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
        } else if (p_input->match_required && !p_input->target_sound_matched &&
                   ((CPU0_THINK_STATE_STEER_PREP == controller.state) ||
                    (CPU0_THINK_STATE_SPIN_PREP == controller.state) ||
                    (CPU0_THINK_STATE_MOVE_STEP == controller.state) ||
                    (CPU0_THINK_STATE_SPIN_STEP == controller.state))) {
            /* 見本照合が失効した周期に駆動を止め、別音への追従を防ぐ。 */
            sound_follow_state_enter(CPU0_THINK_STATE_SETTLE);
        } else if ((CPU0_THINK_STATE_LISTEN == controller.state) && p_input->new_observation) {
            BOOL const usable = sound_follow_observation_trackable(&p_input->observation);
            BOOL const loud = p_input->observation.level_dbfs_x100 >= CPU0_SOUND_TRIGGER_DBFS_X100;
            BOOL const sound_allowed = !p_input->match_required ||
                (p_input->target_sound_matched && p_input->target_sound_direction_valid);

            if (!controller.trigger_active) {
                if (usable && loud && sound_allowed) {
                    controller.trigger_active = TRUE;
                    controller.trigger_elapsed_ms = 0U;
                    controller.doa_sample_count = 0U;
                }
            } else if (!sound_follow_observation_usable(&p_input->observation) || !sound_allowed ||
                       sound_follow_observation_quiet(&p_input->observation)) {
                sound_follow_detection_reset();
            } else if (usable) {
                /* フォールバックDoAの一時的な品質低下（40未満）は、連続音の
                 * 取得を最初からやり直さず、この1サンプルだけ捨てる。 */
                controller.trigger_elapsed_ms += elapsed_ms;
                if (controller.trigger_elapsed_ms >= CPU0_SOUND_DOA_SETTLE_MS) {
                    sound_follow_doa_push(sound_follow_doa_to_relative(p_input->observation.doa_deg));
                }
            }

            H mean_doa_deg = 0;
            if (controller.trigger_active && sound_follow_doa_stable(&mean_doa_deg)) {
                sound_follow_motion_from_doa(sound_follow_rear_seam_select(
                    mean_doa_deg, p_input->rear_seam_turn_preference));
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
            if (p_input->pose_heading_valid) {
                controller.motion_doa_reference_valid = TRUE;
                controller.motion_doa_reference_deg = controller.target_bearing_deg;
                controller.motion_heading_reference_mrad = p_input->pose_heading_mrad;
            }
        } else if ((CPU0_THINK_STATE_SPIN_PREP == controller.state) &&
                   (controller.state_elapsed_ms >= CPU0_SOUND_STEER_SETTLE_MS)) {
            sound_follow_state_enter(CPU0_THINK_STATE_SPIN_STEP);
        } else if (CPU0_THINK_STATE_MOVE_STEP == controller.state) {
            /* 停止聴取シーケンス有効時は直前の静止DoAだけを使う。連続追従モードでは
             * 以下の新しい有効DoAで、今回の操舵・MLP入力を更新し続ける。 */
            BOOL const sound_allowed = !p_input->match_required ||
                (p_input->target_sound_matched && p_input->target_sound_direction_valid);
#if (CPU0_SOUND_USE_LOCALIZATION_FOR_STEERING != 0U)
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
#endif
            if (p_input->new_observation) {
                BOOL const usable = sound_follow_observation_trackable(&p_input->observation);
                BOOL const loud = p_input->observation.level_dbfs_x100 >= CPU0_SOUND_TRIGGER_DBFS_X100;
                BOOL const quiet = sound_follow_observation_quiet(&p_input->observation);

                if (usable && loud && sound_allowed) {
                    /* 連続追従モードでは安定DoAに合わせて今回の走行指令を更新する。 */
#if (CPU0_SOUND_STOP_AND_LISTEN_ENABLE == 0U)
                    sound_follow_doa_push(sound_follow_doa_to_relative(p_input->observation.doa_deg));
                    H mean_doa_deg = 0;
                    if (sound_follow_doa_stable(&mean_doa_deg)) {
                        H const selected_doa_deg = sound_follow_moving_doa_select(
                            sound_follow_rear_seam_select(mean_doa_deg,
                                p_input->rear_seam_turn_preference), p_input);
                        sound_follow_motion_from_doa(selected_doa_deg);
                        if (controller.desired_is_spin_turn) {
                            /* 移動中に方位が後方へ飛んだときは、まず停止して再聴取する。
                             * 別音への瞬間的なDoA切替でその場旋回しない。 */
                            controller.move_relisten_pending = TRUE;
                            sound_follow_state_enter(CPU0_THINK_STATE_SETTLE);
                        }
                    }
#endif
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
#if (CPU0_SOUND_STOP_AND_LISTEN_ENABLE != 0U)
            else if (controller.state_elapsed_ms >= CPU0_SOUND_MOVE_STEP_MS) {
                controller.move_relisten_pending = TRUE;
                sound_follow_state_enter(CPU0_THINK_STATE_SETTLE);
            }
#endif
        } else if (CPU0_THINK_STATE_SPIN_STEP == controller.state) {
            sound_follow_spin_yaw_update(p_input, elapsed_ms);
            sound_follow_spin_command_update();
            controller.spin_progress_elapsed_ms += elapsed_ms;
            BOOL const direction_valid = !p_input->match_required ||
                (p_input->target_sound_matched && p_input->target_sound_direction_valid);
            if (!direction_valid) {
                /* 未一致中はIMU目標だけで回し、復帰後に古いDoAとの差から
                 * 誤って駆動を反転・早期停止しないよう観測履歴を捨てる。 */
                controller.spin_doa_valid = FALSE;
                controller.doa_sample_count = 0U;
                controller.spin_doa_jump_count = 0U;
            }
            BOOL const doa_aligned = direction_valid && sound_follow_spin_doa_update(p_input);
            if (doa_aligned) {
                /* 目標ヨー値は安全上限であり、DoA正面化を優先して即座に再聴取する。 */
                controller.spin_relisten_pending = TRUE;
                sound_follow_state_enter(CPU0_THINK_STATE_SETTLE);
            } else if (controller.spin_yaw_mdeg >= controller.spin_target_yaw_mdeg) {
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
        } else if (CPU0_THINK_STATE_SETTLE == controller.state) {
            BOOL const sound_allowed = !p_input->match_required || p_input->target_sound_matched;
            if (p_input->new_observation) {
                BOOL const usable = sound_follow_observation_trackable(&p_input->observation);
                BOOL const loud = p_input->observation.level_dbfs_x100 >= CPU0_SOUND_TRIGGER_DBFS_X100;
                if (usable && loud && sound_allowed) {
                    controller.quiet_elapsed_ms = 0U;
                } else {
                    controller.quiet_elapsed_ms += elapsed_ms;
                }
            } else {
                controller.quiet_elapsed_ms += elapsed_ms;
            }
            if (controller.state_elapsed_ms >= CPU0_SOUND_LISTEN_SETTLE_MS) {
                /* 後方回頭・短距離移動・回避終了は、連続音でも新しいDoAを測る。 */
                if (controller.spin_relisten_pending || controller.move_relisten_pending) {
#if (CPU0_SOUND_STOP_AND_LISTEN_ENABLE != 0U)
                    if (controller.quiet_elapsed_ms >= CPU0_SOUND_LOST_TIMEOUT_MS) {
                        sound_follow_state_enter(CPU0_THINK_STATE_WAIT_RESTART);
                    } else {
                        sound_follow_state_enter(CPU0_THINK_STATE_LISTEN);
                    }
#else
                    sound_follow_state_enter(CPU0_THINK_STATE_LISTEN);
#endif
                } else {
                    /* 音源消失後の静定を終えて静音確認へ */
                    sound_follow_state_enter(CPU0_THINK_STATE_COOLDOWN);
                }
            }
        } else if ((CPU0_THINK_STATE_COOLDOWN == controller.state) && p_input->match_required) {
            /* 人声が続く部屋では静音待ちが永続する。見本照合が再始動を制限する。 */
            if (controller.state_elapsed_ms >= CPU0_SOUND_LISTEN_SETTLE_MS) {
                sound_follow_state_enter(CPU0_THINK_STATE_LISTEN);
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
