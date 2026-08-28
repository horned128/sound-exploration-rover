/** =================================================================*
 * @file   sound_follow_controller.c
 * @brief  停止聴取型の音源追従状態機械
 * ================================================================= */
#include "sound_follow_controller.h"                      /* 音源追従の入力、出力、状態 */
#include "../../cpu0_config.h"                            /* 音量閾値、動作時間、走行値 */

typedef struct st_sound_follow_context {
    cpu0_think_state_t state;
    uint32_t state_elapsed_ms;
    uint32_t link_stable_ms;
    uint32_t trigger_elapsed_ms;
    uint32_t quiet_elapsed_ms;
    uint8_t doa_sample_count;
    int16_t desired_steering_deg;
    int16_t desired_left_rpm;
    int16_t desired_right_rpm;
    int16_t doa_samples_deg[CPU0_SOUND_DOA_SAMPLE_COUNT];
} sound_follow_context_t;

static int16_t sound_follow_angle_normalize(
    int32_t angle_deg);                                    /* 角度を-180～179度へ正規化 */
static int16_t sound_follow_relative_angle(
    uint16_t doa_deg);                                     /* DoAを車体座標へ変換 */
static int16_t sound_follow_angle_delta(
    int16_t angle_deg,
    int16_t reference_deg);                                /* 円周上の符号付き角度差 */
static int16_t sound_follow_abs_i16(int16_t value);        /* int16_t絶対値 */
static bool sound_follow_observation_usable(
    const acoustic_observation_t * p_observation);         /* 走行判断可能な観測判定 */
static bool sound_follow_observation_quiet(
    const acoustic_observation_t * p_observation);         /* release条件判定 */
static void sound_follow_detection_reset(void);            /* 音量・DoA履歴初期化 */
static void sound_follow_doa_push(int16_t angle_deg);      /* DoA履歴追加 */
static bool sound_follow_doa_stable(
    int16_t * p_mean_deg);                                 /* DoA安定性と平均算出 */
static int16_t sound_follow_steering_from_doa(
    int16_t doa_deg);                                      /* DoAから操舵角算出 */
static void sound_follow_motion_from_doa(
    int16_t doa_deg);                                      /* DoAから操舵・走行方向を決定 */
static void sound_follow_state_enter(
    cpu0_think_state_t state);                             /* 状態遷移 */
static void sound_follow_output_update(
    sound_follow_output_t * p_output);                     /* 状態から指令生成 */

static sound_follow_context_t controller;                 /**< 音源追従状態 */

/** =================================================================*
 * @brief  角度を-180～179度へ正規化
 * @param[in] angle_deg 入力角度
 * @return 正規化角度
 * ================================================================= */
static int16_t sound_follow_angle_normalize(int32_t angle_deg) {
    while (angle_deg >= 180) {
        angle_deg -= 360;
    }
    while (angle_deg < -180) {
        angle_deg += 360;
    }
    return (int16_t) angle_deg;
}

/** =================================================================*
 * @brief  DoAを車体座標へ変換
 * @details 正は右、負は左とし、取付け向きは設定値で補正する。
 * @param[in] doa_deg XVF3800の0～359度DoA
 * @return 車体正面基準の相対角度
 * ================================================================= */
static int16_t sound_follow_relative_angle(uint16_t doa_deg) {
    int16_t angle = sound_follow_angle_normalize(
        (int32_t) doa_deg - CPU0_SOUND_DOA_ZERO_OFFSET_DEG);
    if (0U == CPU0_SOUND_DOA_CLOCKWISE_POSITIVE) {
        angle = (int16_t) -angle;
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
static int16_t sound_follow_angle_delta(int16_t angle_deg,
                                        int16_t reference_deg) {
    return sound_follow_angle_normalize(
        (int32_t) angle_deg - reference_deg);
}

/** =================================================================*
 * @brief  int16_t絶対値
 * @param[in] value 入力値
 * @return 絶対値
 * ================================================================= */
static int16_t sound_follow_abs_i16(int16_t value) {
    return (value < 0) ? (int16_t) -value : value;
}

/** =================================================================*
 * @brief  走行判断可能な観測判定
 * @param[in] p_observation 音響観測
 * @return DoA、XVF状態、音声品質が有効ならtrue
 * ================================================================= */
static bool sound_follow_observation_usable(
    const acoustic_observation_t * p_observation) {
    if ((NULL == p_observation) ||
        (p_observation->doa_deg >= 360U) ||
        (ACOUSTIC_XVF_STATUS_READY != p_observation->xvf_status) ||
        (0U != (p_observation->audio_flags &
                (ACOUSTIC_AUDIO_FLAG_I2C_ERROR |
                 ACOUSTIC_AUDIO_FLAG_MUTED |
                 ACOUSTIC_AUDIO_FLAG_I2S_STALE)))) {
        return false;
    }
    if ((0U != CPU0_SOUND_REQUIRE_VAD) &&
        (0U == p_observation->vad)) {
        return false;
    }
    return true;
}

/** =================================================================*
 * @brief  音源追従解除に十分な静音か判定
 * @param[in] p_observation 音響観測
 * @return release閾値以下、またはVAD必須時の非音声ならtrue
 * ================================================================= */
static bool sound_follow_observation_quiet(
    const acoustic_observation_t * p_observation) {
    if (NULL == p_observation) {
        return true;
    }

    return (p_observation->level_dbfs_x100 <=
            CPU0_SOUND_RELEASE_DBFS_X100) ||
           ((0U != CPU0_SOUND_REQUIRE_VAD) &&
            (0U == p_observation->vad));
}

/** =================================================================*
 * @brief  音量・DoA履歴初期化
 * ================================================================= */
static void sound_follow_detection_reset(void) {
    controller.trigger_elapsed_ms = 0U;
    controller.doa_sample_count = 0U;
}

/** =================================================================*
 * @brief  DoA履歴追加
 * @param[in] angle_deg 車体座標の相対角度
 * ================================================================= */
static void sound_follow_doa_push(int16_t angle_deg) {
    if (controller.doa_sample_count < CPU0_SOUND_DOA_SAMPLE_COUNT) {
        controller.doa_samples_deg[controller.doa_sample_count++] =
            angle_deg;
    } else {
        for (uint8_t index = 1U;
             index < CPU0_SOUND_DOA_SAMPLE_COUNT;
             index++) {
            controller.doa_samples_deg[index - 1U] =
                controller.doa_samples_deg[index];
        }
        controller.doa_samples_deg[CPU0_SOUND_DOA_SAMPLE_COUNT - 1U] =
            angle_deg;
    }
}

/** =================================================================*
 * @brief  DoA安定性と平均算出
 * @param[out] p_mean_deg 円周を考慮した平均角度
 * @return 規定数の角度幅が許容内ならtrue
 * ================================================================= */
static bool sound_follow_doa_stable(int16_t * p_mean_deg) {
    if ((NULL == p_mean_deg) ||
        (controller.doa_sample_count < CPU0_SOUND_DOA_SAMPLE_COUNT)) {
        return false;
    }

    for (uint8_t first = 0U;
         first < CPU0_SOUND_DOA_SAMPLE_COUNT;
         first++) {
        for (uint8_t second = (uint8_t) (first + 1U);
             second < CPU0_SOUND_DOA_SAMPLE_COUNT;
             second++) {
            int16_t const delta = sound_follow_angle_delta(
                controller.doa_samples_deg[first],
                controller.doa_samples_deg[second]);
            if (sound_follow_abs_i16(delta) >
                CPU0_SOUND_DOA_STABLE_WIDTH_DEG) {
                return false;
            }
        }
    }

    int16_t const reference = controller.doa_samples_deg[0];
    int32_t sum = reference;
    for (uint8_t index = 1U;
         index < CPU0_SOUND_DOA_SAMPLE_COUNT;
         index++) {
        sum += reference + sound_follow_angle_delta(
            controller.doa_samples_deg[index], reference);
    }
    *p_mean_deg = sound_follow_angle_normalize(
        sum / (int32_t) CPU0_SOUND_DOA_SAMPLE_COUNT);
    return true;
}

/** =================================================================*
 * @brief  DoAから操舵角算出
 * @param[in] doa_deg 車体座標の相対角度
 * @return 右正・左負の操舵角
 * ================================================================= */
static int16_t sound_follow_steering_from_doa(int16_t doa_deg) {
    if (sound_follow_abs_i16(doa_deg) <=
        CPU0_SOUND_FRONT_TOLERANCE_DEG) {
        return 0;
    }

    int16_t steering = doa_deg;
    if (steering > CPU0_SOUND_STEERING_MAX_DEG) {
        steering = CPU0_SOUND_STEERING_MAX_DEG;
    } else if (steering < -CPU0_SOUND_STEERING_MAX_DEG) {
        steering = -CPU0_SOUND_STEERING_MAX_DEG;
    } else if ((steering > 0) &&
               (steering < CPU0_SOUND_STEERING_MIN_DEG)) {
        steering = CPU0_SOUND_STEERING_MIN_DEG;
    } else if ((steering < 0) &&
               (steering > -CPU0_SOUND_STEERING_MIN_DEG)) {
        steering = -CPU0_SOUND_STEERING_MIN_DEG;
    }
    return steering;
}

/** =================================================================*
 * @brief  DoAから操舵と左右モーター指令を決定
 * @details 前半球は前進、後半球は後進を選ぶ。側方では内輪を減速して
 *          最大45度の4輪逆相操舵でも回頭量を確保する。
 * @param[in] doa_deg 車体正面基準の相対DoA
 * ================================================================= */
static void sound_follow_motion_from_doa(int16_t doa_deg) {
    bool const reverse = sound_follow_abs_i16(doa_deg) >=
                         CPU0_SOUND_REVERSE_ANGLE_DEG;
    int16_t travel_angle_deg = doa_deg;
    if (reverse) {
        travel_angle_deg = sound_follow_angle_normalize(
            (int32_t) doa_deg - 180);
    }

    int16_t steering_deg = sound_follow_steering_from_doa(
        travel_angle_deg);
    if (reverse) {
        /* 後進時は同じ車体回頭方向に対する操舵符号が前進時と反転する。 */
        steering_deg = (int16_t) -steering_deg;
    }

    int16_t const direction = reverse ? -1 : 1;
    int16_t left_rpm = (int16_t) (direction *
        CPU0_SOUND_MOVE_LEFT_RPM);
    int16_t right_rpm = (int16_t) (direction *
        CPU0_SOUND_MOVE_RIGHT_RPM);
    if (steering_deg > 0) {
        right_rpm = (int16_t) (direction *
            CPU0_SOUND_TURN_INNER_RPM);
    } else if (steering_deg < 0) {
        left_rpm = (int16_t) (direction *
            CPU0_SOUND_TURN_INNER_RPM);
    }

    controller.desired_steering_deg = steering_deg;
    controller.desired_left_rpm = left_rpm;
    controller.desired_right_rpm = right_rpm;
}

/** =================================================================*
 * @brief  状態遷移
 * @param[in] state 遷移先
 * ================================================================= */
static void sound_follow_state_enter(cpu0_think_state_t state) {
    controller.state = state;
    controller.state_elapsed_ms = 0U;
    if ((CPU0_THINK_STATE_LISTEN == state) ||
        (CPU0_THINK_STATE_WAIT_LINK == state) ||
        (CPU0_THINK_STATE_COOLDOWN == state)) {
        sound_follow_detection_reset();
        controller.quiet_elapsed_ms = 0U;
    }
}

/** =================================================================*
 * @brief  追従状態初期化
 * ================================================================= */
void sound_follow_controller_init(void) {
    controller = (sound_follow_context_t) {
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
static void sound_follow_output_update(
    sound_follow_output_t * p_output) {
    *p_output = (sound_follow_output_t) {
        .state = controller.state,
        .steering_deg = 0,
        .left_rpm = 0,
        .right_rpm = 0,
        .actuator_enable = true,
        .emergency_stop = false,
    };

    if ((CPU0_THINK_STATE_WAIT_LINK == controller.state) ||
        (CPU0_THINK_STATE_FAULT == controller.state)) {
        p_output->actuator_enable = false;
        p_output->emergency_stop = true;
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
void sound_follow_controller_step(
    const sound_follow_input_t * p_input,
    uint32_t elapsed_ms,
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
        if (controller.link_stable_ms >=
            CPU0_SOUND_LINK_STABLE_MS) {
            controller.link_stable_ms = CPU0_SOUND_LINK_STABLE_MS;
            sound_follow_state_enter(CPU0_THINK_STATE_LISTEN);
        }
    } else if (CPU0_THINK_STATE_FAULT != controller.state) {
        controller.state_elapsed_ms += elapsed_ms;

        if ((CPU0_THINK_STATE_LISTEN == controller.state) &&
            p_input->new_observation) {
            bool const usable = sound_follow_observation_usable(
                &p_input->observation);
            bool const loud = usable &&
                (p_input->observation.level_dbfs_x100 >=
                 CPU0_SOUND_TRIGGER_DBFS_X100);

            if (loud) {
                sound_follow_doa_push(sound_follow_relative_angle(
                    p_input->observation.doa_deg));
                controller.trigger_elapsed_ms += elapsed_ms;
            } else {
                sound_follow_detection_reset();
            }

            int16_t mean_doa_deg = 0;
            if ((controller.trigger_elapsed_ms >=
                 CPU0_SOUND_TRIGGER_HOLD_MS) &&
                sound_follow_doa_stable(&mean_doa_deg)) {
                sound_follow_motion_from_doa(mean_doa_deg);
                sound_follow_state_enter(
                    CPU0_THINK_STATE_STEER_PREP);
            }
        } else if ((CPU0_THINK_STATE_STEER_PREP ==
                    controller.state) &&
                   (controller.state_elapsed_ms >=
                    CPU0_SOUND_STEER_SETTLE_MS)) {
            sound_follow_state_enter(CPU0_THINK_STATE_MOVE_STEP);
        } else if ((CPU0_THINK_STATE_MOVE_STEP ==
                    controller.state) &&
                   (controller.state_elapsed_ms >=
                    CPU0_SOUND_MOVE_STEP_MS)) {
            sound_follow_state_enter(CPU0_THINK_STATE_SETTLE);
        } else if ((CPU0_THINK_STATE_SETTLE == controller.state) &&
                   (controller.state_elapsed_ms >=
                    CPU0_SOUND_LISTEN_SETTLE_MS)) {
            /* 1回の検出で1 stepだけ動かす。走行後のDoA揺れ（モーター音・
             * 反射音）を次の移動目標として再解釈しない。 */
            sound_follow_state_enter(CPU0_THINK_STATE_COOLDOWN);
        } else if ((CPU0_THINK_STATE_COOLDOWN == controller.state) &&
                   p_input->new_observation) {
            bool const quiet = sound_follow_observation_quiet(
                &p_input->observation);
            controller.quiet_elapsed_ms = quiet ?
                controller.quiet_elapsed_ms + elapsed_ms : 0U;
            if (controller.quiet_elapsed_ms >=
                CPU0_SOUND_COOLDOWN_RELEASE_MS) {
                controller.quiet_elapsed_ms = 0U;
                sound_follow_state_enter(CPU0_THINK_STATE_LISTEN);
            }
        } else if (CPU0_THINK_STATE_COOLDOWN == controller.state) {
            controller.quiet_elapsed_ms = 0U;
        }
    }

    sound_follow_output_update(p_output);
}
