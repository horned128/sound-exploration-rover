/** =================================================================*
 * @file   servo.c
 * @brief  4チャンネルRCサーボPWM制御
 * ================================================================= */
#include "servo.h"                                          /* RCサーボ制御API */

/**< サーボごとのFSP PWMインスタンス。未設定チャンネルは目標値だけ保持する。 */
static timer_instance_t const * const servo_timers[SERVO_COUNT] = {
    SERVO_PWM_INSTANCE_FR,
    SERVO_PWM_INSTANCE_FL,
    SERVO_PWM_INSTANCE_RR,
    SERVO_PWM_INSTANCE_RL,
};

/**< サーボごとのPWM出力端子 */
static uint32_t const servo_outputs[SERVO_COUNT] = {
    SERVO_PWM_OUTPUT_FR,
    SERVO_PWM_OUTPUT_FL,
    SERVO_PWM_OUTPUT_RR,
    SERVO_PWM_OUTPUT_RL,
};

/**< 各サーボの操舵方向（+1または-1） */
static int8_t const servo_directions[SERVO_COUNT] = {
    SERVO_DIRECTION_FR,
    SERVO_DIRECTION_FL,
    SERVO_DIRECTION_RR,
    SERVO_DIRECTION_RL,
};

/**< 各輪の中心パルス補正値（単位: us） */
volatile int16_t g_servo_center_trim_us[SERVO_COUNT] = {
    SERVO_CENTER_TRIM_US_FR,
    SERVO_CENTER_TRIM_US_FL,
    SERVO_CENTER_TRIM_US_RR,
    SERVO_CENTER_TRIM_US_RL,
};

/**< 各サーボへ設定したパルス幅（単位: us） */
volatile uint16_t g_servo_pulse_us[SERVO_COUNT] = {
    SERVO_PULSE_CENTER_US,
    SERVO_PULSE_CENTER_US,
    SERVO_PULSE_CENTER_US,
    SERVO_PULSE_CENTER_US,
};

static bool servo_running[SERVO_COUNT];                      /**< サーボPWM出力状態 */
/**< 各サーボ目標角度 */
static int16_t servo_target_deg[SERVO_COUNT] = {
    STEERING_CENTER_DEG,
    STEERING_CENTER_DEG,
    STEERING_CENTER_DEG,
    STEERING_CENTER_DEG,
};

/** =================================================================*
 * @brief  RCサーボ初期化
 * @return FSPエラーコード
 * ================================================================= */
fsp_err_t servo_init(void) {
    for (uint32_t i = 0U; i < SERVO_COUNT; i++) {
        servo_running[i] = false;
        servo_target_deg[i] = STEERING_CENTER_DEG;
        int32_t center_us = (int32_t) SERVO_PULSE_CENTER_US +
                            (int32_t) g_servo_center_trim_us[i];
        if (center_us < (int32_t) SERVO_PULSE_MIN_SAFE_US) {
            center_us = (int32_t) SERVO_PULSE_MIN_SAFE_US;
        } else if (center_us > (int32_t) SERVO_PULSE_MAX_SAFE_US) {
            center_us = (int32_t) SERVO_PULSE_MAX_SAFE_US;
        }
        g_servo_pulse_us[i] = (uint16_t) center_us;

        if (NULL != servo_timers[i]) {
            fsp_err_t const err = servo_timers[i]->p_api->open(servo_timers[i]->p_ctrl,
                                                               servo_timers[i]->p_cfg);
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
fsp_err_t servo_set_target_deg(uint32_t servo_index, int16_t target_deg) {
    if (servo_index >= SERVO_COUNT) {
        return FSP_ERR_INVALID_ARGUMENT;
    }
    if ((target_deg < STEERING_MIN_DEG) || (target_deg > STEERING_MAX_DEG)) {
        return FSP_ERR_INVALID_ARGUMENT;
    }

    int32_t const angle_span = STEERING_MAX_DEG - STEERING_MIN_DEG;
    int32_t const pulse_span = STEERING_MAX_PULSE_US - STEERING_MIN_PULSE_US;
    int32_t const angle_delta_us = ((int32_t) target_deg * pulse_span) / angle_span;
    int32_t pulse_value_us = (int32_t) SERVO_PULSE_CENTER_US +
                             (int32_t) g_servo_center_trim_us[servo_index] +
                             ((int32_t) servo_directions[servo_index] * angle_delta_us);
    if (pulse_value_us < (int32_t) SERVO_PULSE_MIN_SAFE_US) {
        pulse_value_us = (int32_t) SERVO_PULSE_MIN_SAFE_US;
    } else if (pulse_value_us > (int32_t) SERVO_PULSE_MAX_SAFE_US) {
        pulse_value_us = (int32_t) SERVO_PULSE_MAX_SAFE_US;
    }
    uint16_t const pulse_us = (uint16_t) pulse_value_us;

    timer_instance_t const * const p_timer = servo_timers[servo_index];
    if (NULL != p_timer) {
        timer_info_t info = {0};
        fsp_err_t err = p_timer->p_api->infoGet(p_timer->p_ctrl, &info);
        if (FSP_SUCCESS == err) {
            uint32_t const duty_counts =
                (uint32_t) ((((uint64_t) info.period_counts * pulse_us) +
                             (SERVO_PWM_PERIOD_US / 2U)) / SERVO_PWM_PERIOD_US);
            err = p_timer->p_api->dutyCycleSet(p_timer->p_ctrl,
                                               duty_counts,
                                               servo_outputs[servo_index]);
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
fsp_err_t servo_disable(uint32_t servo_index) {
    if (servo_index >= SERVO_COUNT) {
        return FSP_ERR_INVALID_ARGUMENT;
    }
    if (!servo_running[servo_index]) {
        return FSP_SUCCESS;
    }

    timer_instance_t const * const p_timer = servo_timers[servo_index];
    fsp_err_t const err = p_timer->p_api->stop(p_timer->p_ctrl);
    if (FSP_SUCCESS == err) {
        servo_running[servo_index] = false;
    }

    return err;
}

/** =================================================================*
 * @brief  RCサーボ目標角度取得
 * @param[in] servo_index サーボ番号（0～3）
 * @return 目標角度（単位: 度）。番号が不正な場合は中央角度。
 * ================================================================= */
int16_t servo_target_deg_get(uint32_t servo_index) {
    if (servo_index >= SERVO_COUNT) {
        return STEERING_CENTER_DEG;
    }

    return servo_target_deg[servo_index];
}
