/** =================================================================*
 * @file   sound_follow_controller.c
 * @brief  停止聴取型の音源追従状態機械
 * ================================================================= */
#include "sound_follow_controller.h"                        /* 音源追従の入力、出力、状態 */
#include "config/control_config.h"                         /* 音量閾値、動作時間、走行値 */

typedef struct st_sound_follow_context {
    sound_follow_state_t state;
    UW state_elapsed_ms;
    UW link_stable_ms;
    UW trigger_elapsed_ms;
    UW quiet_elapsed_ms;
    BOOL trigger_active;
    UB doa_sample_count;
    H desired_steering_deg;
    H desired_left_rpm;
    H desired_right_rpm;
    H doa_samples_deg[CPU0_SOUND_DOA_SAMPLE_COUNT];
} sound_follow_context_t;

LOCAL H sound_follow_angle_normalize(W angle_deg); /* 角度を-180～179度へ正規化 */
LOCAL H sound_follow_relative_angle(UH doa_deg); /* DoAを車体座標へ変換 */
LOCAL H sound_follow_angle_delta(H angle_deg, H reference_deg); /* 円周上の符号付き角度差 */
LOCAL H sound_follow_abs_i16(H value);         /* int16_t絶対値 */
LOCAL BOOL sound_follow_observation_usable(const acoustic_observation_t * p_observation); /* DoA品質判定 */
LOCAL BOOL sound_follow_observation_quiet(const acoustic_observation_t * p_observation); /* release条件判定 */
LOCAL void sound_follow_detection_reset(void);             /* 音量・DoA履歴初期化 */
LOCAL void sound_follow_doa_push(H angle_deg);       /* DoA履歴追加 */
LOCAL BOOL sound_follow_doa_stable(H * p_mean_deg);  /* DoA安定性と平均算出 */
LOCAL H sound_follow_steering_from_doa(H doa_deg); /* DoAから操舵角算出 */
LOCAL void sound_follow_motion_from_doa(H doa_deg);  /* DoAから操舵・走行方向を決定 */
LOCAL void sound_follow_state_enter(sound_follow_state_t state); /* 状態遷移 */
LOCAL void sound_follow_output_update(sound_follow_output_t * p_output); /* 状態から指令生成 */

LOCAL sound_follow_context_t controller;                   /**< 音源追従状態 */

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
LOCAL H sound_follow_relative_angle(UH doa_deg) {
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
    } else if ((steering > 0) && (steering < CPU0_SOUND_STEERING_MIN_DEG)) {
        steering = CPU0_SOUND_STEERING_MIN_DEG;
    } else if ((steering < 0) && (steering > -CPU0_SOUND_STEERING_MIN_DEG)) {
        steering = -CPU0_SOUND_STEERING_MIN_DEG;
    }
    return steering;
}

/** =================================================================*
 * @brief  DoAから操舵と左右モーター指令を決定
 * @details 全方位で前進を選び、側方・後方では内輪を減速した最大45度の
 *          4輪逆相操舵で回頭する。
 * @param[in] doa_deg 車体正面基準の相対DoA
 * ================================================================= */
LOCAL void sound_follow_motion_from_doa(H doa_deg) {
    H const steering_deg = sound_follow_steering_from_doa(doa_deg);
    H left_rpm = CPU0_SOUND_MOVE_LEFT_RPM;
    H right_rpm = CPU0_SOUND_MOVE_RIGHT_RPM;
    if (steering_deg > 0) {
        right_rpm = CPU0_SOUND_TURN_INNER_RPM;
    } else if (steering_deg < 0) {
        left_rpm = CPU0_SOUND_TURN_INNER_RPM;
    }

    controller.desired_steering_deg = steering_deg;
    controller.desired_left_rpm = left_rpm;
    controller.desired_right_rpm = right_rpm;
}

/** =================================================================*
 * @brief  状態遷移
 * @param[in] state 遷移先
 * ================================================================= */
LOCAL void sound_follow_state_enter(sound_follow_state_t state) {
    controller.state = state;
    controller.state_elapsed_ms = 0U;
    if ((CPU0_THINK_STATE_LISTEN == state) || (CPU0_THINK_STATE_WAIT_LINK == state) ||
        (CPU0_THINK_STATE_COOLDOWN == state)) {
        sound_follow_detection_reset();
        controller.quiet_elapsed_ms = 0U;
    }
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
    } else if (CPU0_THINK_STATE_MOVE_STEP == controller.state) {
        p_output->steering_deg = controller.desired_steering_deg;
        p_output->left_rpm = controller.desired_left_rpm;
        p_output->right_rpm = controller.desired_right_rpm;
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
    } else if (CPU0_THINK_STATE_WAIT_LINK == controller.state) {
        controller.link_stable_ms += elapsed_ms;
        if (controller.link_stable_ms >= CPU0_SOUND_LINK_STABLE_MS) {
            controller.link_stable_ms = CPU0_SOUND_LINK_STABLE_MS;
            sound_follow_state_enter(CPU0_THINK_STATE_LISTEN);
        }
    } else if (CPU0_THINK_STATE_FAULT != controller.state) {
        controller.state_elapsed_ms += elapsed_ms;

        if (!p_input->motion_allowed && (CPU0_THINK_STATE_STEER_PREP == controller.state)) {
            sound_follow_state_enter(CPU0_THINK_STATE_COOLDOWN);
        } else if (!p_input->motion_allowed && (CPU0_THINK_STATE_MOVE_STEP == controller.state)) {
            sound_follow_state_enter(CPU0_THINK_STATE_SETTLE);
        } else if ((CPU0_THINK_STATE_LISTEN == controller.state) && p_input->new_observation) {
            BOOL const usable = sound_follow_observation_usable(&p_input->observation);
            BOOL const loud = p_input->observation.level_dbfs_x100 >= CPU0_SOUND_TRIGGER_DBFS_X100;

            if (!controller.trigger_active) {
                if (usable && loud && (0U != p_input->observation.vad)) {
                    controller.trigger_active = TRUE;
                    controller.trigger_elapsed_ms = 0U;
                    controller.doa_sample_count = 0U;
                }
            } else if (!usable) {
                sound_follow_detection_reset();
            } else {
                controller.trigger_elapsed_ms += elapsed_ms;
                if (controller.trigger_elapsed_ms >= CPU0_SOUND_DOA_SETTLE_MS) {
                    sound_follow_doa_push(sound_follow_relative_angle(p_input->observation.doa_deg));
                }
            }

            H mean_doa_deg = 0;
            if (controller.trigger_active && sound_follow_doa_stable(&mean_doa_deg)) {
                sound_follow_motion_from_doa(mean_doa_deg);
                if (p_input->motion_allowed) {
                    sound_follow_state_enter(CPU0_THINK_STATE_STEER_PREP);
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
        } else if ((CPU0_THINK_STATE_MOVE_STEP == controller.state) &&
                   (controller.state_elapsed_ms >= CPU0_SOUND_MOVE_STEP_MS)) {
            sound_follow_state_enter(CPU0_THINK_STATE_SETTLE);
        } else if ((CPU0_THINK_STATE_SETTLE == controller.state) &&
                   (controller.state_elapsed_ms >= CPU0_SOUND_LISTEN_SETTLE_MS)) {
            /* 1回の検出で1 stepだけ動かす。走行後のDoA揺れ（モーター音・
             * 反射音）を次の移動目標として再解釈しない。 */
            sound_follow_state_enter(CPU0_THINK_STATE_COOLDOWN);
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
