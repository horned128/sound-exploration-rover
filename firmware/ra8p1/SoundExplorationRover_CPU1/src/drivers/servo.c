/** =================================================================*
 * @file   servo.c
 * @brief  4チャンネルRCサーボPWM制御
 * ================================================================= */
#include "servo.h"                                          /* RCサーボ制御API */

/**< サーボごとのFSP PWMインスタンス。未設定チャンネルは目標値だけ保持する。 */
LOCAL timer_instance_t const * const servo_timers[SERVO_COUNT] = {
    SERVO_PWM_INSTANCE_FR,
    SERVO_PWM_INSTANCE_FL,
    SERVO_PWM_INSTANCE_RR,
    SERVO_PWM_INSTANCE_RL,
};

/**< サーボごとのPWM出力端子 */
LOCAL UW const servo_outputs[SERVO_COUNT] = {
    SERVO_PWM_OUTPUT_FR,
    SERVO_PWM_OUTPUT_FL,
    SERVO_PWM_OUTPUT_RR,
    SERVO_PWM_OUTPUT_RL,
};

/**< 各サーボの操舵方向（+1または-1） */
LOCAL B const servo_directions[SERVO_COUNT] = {
    SERVO_DIRECTION_FR,
    SERVO_DIRECTION_FL,
    SERVO_DIRECTION_RR,
    SERVO_DIRECTION_RL,
};

/**< 各輪の中心パルス補正値（単位: us） */
EXPORT volatile H g_servo_center_trim_us[SERVO_COUNT] = {
    SERVO_CENTER_TRIM_US_FR,
    SERVO_CENTER_TRIM_US_FL,
    SERVO_CENTER_TRIM_US_RR,
    SERVO_CENTER_TRIM_US_RL,
};

/**< 各サーボへ設定したパルス幅（単位: us） */
EXPORT volatile UH g_servo_pulse_us[SERVO_COUNT] = {
    SERVO_PULSE_CENTER_US,
    SERVO_PULSE_CENTER_US,
    SERVO_PULSE_CENTER_US,
    SERVO_PULSE_CENTER_US,
};

LOCAL BOOL servo_running[SERVO_COUNT];                      /**< サーボPWM出力状態 */
/**< 各サーボ目標角度 */
LOCAL H servo_target_deg[SERVO_COUNT] = {
    STEERING_CENTER_DEG,
    STEERING_CENTER_DEG,
    STEERING_CENTER_DEG,
    STEERING_CENTER_DEG,
};

/** =================================================================*
 * @brief  RCサーボ初期化
 * @return FSPエラーコード
 * ================================================================= */
EXPORT fsp_err_t servo_init(void) {
    for (UW i = 0U; i < SERVO_COUNT; i++) {
        servo_running[i] = FALSE;
        servo_target_deg[i] = STEERING_CENTER_DEG;
        W center_us = (W) SERVO_PULSE_CENTER_US + (W) g_servo_center_trim_us[i];
        if (center_us < (W) SERVO_PULSE_MIN_SAFE_US) {
            center_us = (W) SERVO_PULSE_MIN_SAFE_US;
        } else if (center_us > (W) SERVO_PULSE_MAX_SAFE_US) {
            center_us = (W) SERVO_PULSE_MAX_SAFE_US;
        }
        g_servo_pulse_us[i] = (UH) center_us;

        if (NULL != servo_timers[i]) {
            fsp_err_t const err = servo_timers[i]->p_api->open(servo_timers[i]->p_ctrl, servo_timers[i]->p_cfg);
            if (FSP_SUCCESS != err) {
                return err;
            }
        }
    }

    return FSP_SUCCESS;
}

/** =================================================================*
 * @brief  RCサーボ目標角度設定
 * @param[in] servo_index サーボ番号（0～3）
 * @param[in] target_deg 目標角度（単位: 度）
 * @return FSPエラーコード
 * ================================================================= */
EXPORT fsp_err_t servo_set_target_deg(UW servo_index, H target_deg) {
    if (servo_index >= SERVO_COUNT) {
        return FSP_ERR_INVALID_ARGUMENT;
    }
    if ((target_deg < STEERING_MIN_DEG) || (target_deg > STEERING_MAX_DEG)) {
        return FSP_ERR_INVALID_ARGUMENT;
    }

    W const angle_span = STEERING_MAX_DEG - STEERING_MIN_DEG;
    W const pulse_span = STEERING_MAX_PULSE_US - STEERING_MIN_PULSE_US;
    W const angle_delta_us = ((W) target_deg * pulse_span) / angle_span;
    W pulse_value_us = (W) SERVO_PULSE_CENTER_US + (W) g_servo_center_trim_us[servo_index] +
                             ((W) servo_directions[servo_index] * angle_delta_us);
    if (pulse_value_us < (W) SERVO_PULSE_MIN_SAFE_US) {
        pulse_value_us = (W) SERVO_PULSE_MIN_SAFE_US;
    } else if (pulse_value_us > (W) SERVO_PULSE_MAX_SAFE_US) {
        pulse_value_us = (W) SERVO_PULSE_MAX_SAFE_US;
    }
    UH const pulse_us = (UH) pulse_value_us;

    timer_instance_t const * const p_timer = servo_timers[servo_index];
    if (NULL != p_timer) {
        timer_info_t info = {0};
        fsp_err_t err = p_timer->p_api->infoGet(p_timer->p_ctrl, &info);
        if (FSP_SUCCESS == err) {
            UW const duty_counts =
                (UW) ((((UD) info.period_counts * pulse_us) + (SERVO_PWM_PERIOD_US / 2U)) /
                            SERVO_PWM_PERIOD_US);
            err = p_timer->p_api->dutyCycleSet(p_timer->p_ctrl, duty_counts, servo_outputs[servo_index]);
        }
        if ((FSP_SUCCESS == err) && !servo_running[servo_index]) {
            err = p_timer->p_api->start(p_timer->p_ctrl);
            servo_running[servo_index] = (FSP_SUCCESS == err);
        }
        if (FSP_SUCCESS != err) {
            return err;
        }
    }

    servo_target_deg[servo_index] = target_deg;
    g_servo_pulse_us[servo_index] = pulse_us;
    return FSP_SUCCESS;
}

/** =================================================================*
 * @brief  RCサーボPWM停止
 * @param[in] servo_index サーボ番号（0～3）
 * @return FSPエラーコード
 * ================================================================= */
EXPORT fsp_err_t servo_disable(UW servo_index) {
    if (servo_index >= SERVO_COUNT) {
        return FSP_ERR_INVALID_ARGUMENT;
    }
    if (!servo_running[servo_index]) {
        return FSP_SUCCESS;
    }

    timer_instance_t const * const p_timer = servo_timers[servo_index];
    fsp_err_t const err = p_timer->p_api->stop(p_timer->p_ctrl);
    if (FSP_SUCCESS == err) {
        servo_running[servo_index] = FALSE;
    }

    return err;
}

/** =================================================================*
 * @brief  RCサーボ目標角度取得
 * @param[in] servo_index サーボ番号（0～3）
 * @return 目標角度（単位: 度）。番号が不正な場合は中央角度。
 * ================================================================= */
EXPORT H servo_target_deg_get(UW servo_index) {
    if (servo_index >= SERVO_COUNT) {
        return STEERING_CENTER_DEG;
    }

    return servo_target_deg[servo_index];
}
