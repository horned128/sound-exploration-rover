/** =================================================================*
 * @file   dc_motor.c
 * @brief  左右BTS7960モーターPWM制御
 * @details RPWMとLPWMを各方向へ独立出力し、方向切替時は両出力を一度停止する。
 * ================================================================= */
#include "dc_motor.h"                                       /* DCモーター制御API */
#include "../cpu1_config.h"                                 /* モーター制御設定 */
#include "encoder.h"                                        /* 左右代表エンコーダ速度 */

EXPORT volatile H g_drive_left_duty_permille = 0;            /**< 左モーター出力指令（単位: 1/1000） */
EXPORT volatile H g_drive_right_duty_permille = 0;           /**< 右モーター出力指令（単位: 1/1000） */

LOCAL H g_left_target_rpm;                           /**< 左モーター目標回転数（単位: RPM） */
LOCAL H g_right_target_rpm;                          /**< 右モーター目標回転数（単位: RPM） */
LOCAL H g_left_target_duty_permille;                 /**< 左モーター目標デューティ（単位: 1/1000） */
LOCAL H g_right_target_duty_permille;                /**< 右モーター目標デューティ（単位: 1/1000） */
LOCAL UW g_pwm_update_elapsed_ms;                    /**< PWM更新周期の経過時間（単位: ms） */
LOCAL UW g_speed_feedback_elapsed_ms;                /**< 指令変更後の速度観測待機時間（単位: ms） */
LOCAL BOOL g_pwm_running;                                  /**< PWMタイマの動作状態 */

/** =================================================================*
 * @brief  回転数をデューティへ変換
 * @param[in] target_rpm 目標回転数（単位: RPM）
 * @param[in] forward_sign 論理前進方向の極性
 * @return 符号付きデューティ（単位: 1/1000）
 * ================================================================= */
LOCAL H motor_rpm_to_duty_permille(H target_rpm, B forward_sign) {
    W const signed_rpm = (W) target_rpm * (W) forward_sign;
    if (0 == signed_rpm) {
        return 0;
    }

    W const magnitude = (signed_rpm < 0) ? -signed_rpm : signed_rpm;
    W duty = (magnitude * 1000) / JGA25_TARGET_RPM_MAX;
    if (duty < MOTOR_PWM_MIN_DUTY_PERMILLE) {
        duty = MOTOR_PWM_MIN_DUTY_PERMILLE;
    }
    if (duty > MOTOR_PWM_MAX_DUTY_PERMILLE) {
        duty = MOTOR_PWM_MAX_DUTY_PERMILLE;
    }

    return (signed_rpm < 0) ? (H) -duty : (H) duty;
}

/** =================================================================*
 * @brief  代表エンコーダの実測RPMを使ってPWMデューティを補正
 * @details 指令開始直後は前回停止時の0 RPMを使わないよう、観測待機時間が
 *          過ぎるまで基本デューティを維持する。
 * @param[in] target_rpm 車体前進正の目標RPM
 * @param[in] forward_sign BTS7960出力極性
 * @param[in] duty_scale_permille 側別の基本デューティ補正（単位: 1/1000）
 * @param[in] measured_rpm 車体前進正の代表モーター実測RPM
 * @param[in] feedback_ready 実測値を制御に使える場合true
 * @return BTS7960出力極性を反映した符号付きデューティ（単位: 1/1000）
 * ================================================================= */
LOCAL H motor_rpm_to_feedback_duty_permille(H target_rpm, B forward_sign,
                                                   UH duty_scale_permille, H measured_rpm,
                                                   BOOL feedback_ready) {
    H const raw_base_duty = motor_rpm_to_duty_permille(target_rpm, forward_sign);
    W base_magnitude = (raw_base_duty < 0) ? -(W) raw_base_duty : raw_base_duty;
    base_magnitude = (base_magnitude * duty_scale_permille) / 1000;
    if (base_magnitude > MOTOR_PWM_MAX_DUTY_PERMILLE) {
        base_magnitude = MOTOR_PWM_MAX_DUTY_PERMILLE;
    }
    H const base_duty = (raw_base_duty < 0) ? (H) -base_magnitude : (H) base_magnitude;
    if ((0U == MOTOR_SPEED_FEEDBACK_ENABLE) || !feedback_ready || (0 == target_rpm)) {
        return base_duty;
    }

    W const target_magnitude = (target_rpm < 0) ? -(W) target_rpm : target_rpm;
    W const measured_in_target_direction = (target_rpm < 0) ? -(W) measured_rpm : measured_rpm;
    W correction = (target_magnitude - measured_in_target_direction) * MOTOR_SPEED_FEEDBACK_KP_PERMILLE_PER_RPM;
    if (correction > MOTOR_SPEED_FEEDBACK_MAX_CORRECTION_PERMILLE) {
        correction = MOTOR_SPEED_FEEDBACK_MAX_CORRECTION_PERMILLE;
    } else if (correction < -MOTOR_SPEED_FEEDBACK_MAX_CORRECTION_PERMILLE) {
        correction = -MOTOR_SPEED_FEEDBACK_MAX_CORRECTION_PERMILLE;
    }

    W duty_magnitude = (base_duty < 0) ? -(W) base_duty : base_duty;
    duty_magnitude += correction;
    if (duty_magnitude < MOTOR_PWM_MIN_DUTY_PERMILLE) {
        duty_magnitude = MOTOR_PWM_MIN_DUTY_PERMILLE;
    } else if (duty_magnitude > MOTOR_PWM_MAX_DUTY_PERMILLE) {
        duty_magnitude = MOTOR_PWM_MAX_DUTY_PERMILLE;
    }

    return (base_duty < 0) ? (H) -duty_magnitude : (H) duty_magnitude;
}

/** =================================================================*
 * @brief  PWMデューティを目標値へ近づける
 * @param[in] current 現在値（単位: 1/1000）
 * @param[in] target 目標値（単位: 1/1000）
 * @return 更新後のデューティ（単位: 1/1000）
 * ================================================================= */
LOCAL H motor_ramp_value(H current, H target) {
    if (current < target) {
        W const next = (W) current + MOTOR_PWM_RAMP_PER_MS;
        return (H) ((next > target) ? target : next);
    }
    if (current > target) {
        W const next = (W) current - MOTOR_PWM_RAMP_PER_MS;
        return (H) ((next < target) ? target : next);
    }

    return current;
}

/** =================================================================*
 * @brief  4系統PWM出力更新
 * @return FSPエラーコード
 * @details 方向切替時は旧出力を先に0へ戻し、BTS7960のRPWMとLPWMを同時に
 *          有効にしない。デューティが0のときは共通ENも無効にする。
 * ================================================================= */
LOCAL fsp_err_t motor_pwm_apply(void) {
    timer_info_t rpwm_info = {0};
    timer_info_t lpwm_info = {0};
    fsp_err_t err = MOTOR_RPWM_INSTANCE->p_api->infoGet(MOTOR_RPWM_INSTANCE->p_ctrl, &rpwm_info);
    if (FSP_SUCCESS != err) {
        return err;
    }

    err = MOTOR_LPWM_INSTANCE->p_api->infoGet(MOTOR_LPWM_INSTANCE->p_ctrl, &lpwm_info);
    if (FSP_SUCCESS != err) {
        return err;
    }

    H const left_duty = g_drive_left_duty_permille;
    H const right_duty = g_drive_right_duty_permille;
    UH const left_magnitude = (UH) ((left_duty < 0) ? -left_duty : left_duty);
    UH const right_magnitude = (UH) ((right_duty < 0) ? -right_duty : right_duty);
    UH const left_rpwm = (left_duty > 0) ? left_magnitude : 0U;
    UH const left_lpwm = (left_duty < 0) ? left_magnitude : 0U;
    UH const right_rpwm = (right_duty > 0) ? right_magnitude : 0U;
    UH const right_lpwm = (right_duty < 0) ? right_magnitude : 0U;

    UW const left_rpwm_counts = (UW) (((UD) rpwm_info.period_counts * left_rpwm) / 1000U);
    UW const right_rpwm_counts = (UW) (((UD) rpwm_info.period_counts * right_rpwm) / 1000U);
    UW const left_lpwm_counts = (UW) (((UD) lpwm_info.period_counts * left_lpwm) / 1000U);
    UW const right_lpwm_counts = (UW) (((UD) lpwm_info.period_counts * right_lpwm) / 1000U);

    /* 方向を切り替える前に4出力を停止し、RPWMとLPWMの同時有効を防ぐ。 */
    err = MOTOR_RPWM_INSTANCE->p_api->dutyCycleSet(MOTOR_RPWM_INSTANCE->p_ctrl, 0U, MOTOR_LEFT_RPWM_OUTPUT);
    if (FSP_SUCCESS == err) {
        err = MOTOR_RPWM_INSTANCE->p_api->dutyCycleSet(MOTOR_RPWM_INSTANCE->p_ctrl, 0U, MOTOR_RIGHT_RPWM_OUTPUT);
    }
    if (FSP_SUCCESS == err) {
        err = MOTOR_LPWM_INSTANCE->p_api->dutyCycleSet(MOTOR_LPWM_INSTANCE->p_ctrl, 0U, MOTOR_LEFT_LPWM_OUTPUT);
    }
    if (FSP_SUCCESS == err) {
        err = MOTOR_LPWM_INSTANCE->p_api->dutyCycleSet(MOTOR_LPWM_INSTANCE->p_ctrl, 0U, MOTOR_RIGHT_LPWM_OUTPUT);
    }
    if (FSP_SUCCESS == err) {
        err = MOTOR_RPWM_INSTANCE->p_api->dutyCycleSet(MOTOR_RPWM_INSTANCE->p_ctrl, left_rpwm_counts,
                                                       MOTOR_LEFT_RPWM_OUTPUT);
    }
    if (FSP_SUCCESS == err) {
        err = MOTOR_RPWM_INSTANCE->p_api->dutyCycleSet(MOTOR_RPWM_INSTANCE->p_ctrl, right_rpwm_counts,
                                                       MOTOR_RIGHT_RPWM_OUTPUT);
    }
    if (FSP_SUCCESS == err) {
        err = MOTOR_LPWM_INSTANCE->p_api->dutyCycleSet(MOTOR_LPWM_INSTANCE->p_ctrl, left_lpwm_counts,
                                                       MOTOR_LEFT_LPWM_OUTPUT);
    }
    if (FSP_SUCCESS == err) {
        err = MOTOR_LPWM_INSTANCE->p_api->dutyCycleSet(MOTOR_LPWM_INSTANCE->p_ctrl, right_lpwm_counts,
                                                       MOTOR_RIGHT_LPWM_OUTPUT);
    }

    BOOL const should_run = (0U != left_magnitude) || (0U != right_magnitude);
    if ((FSP_SUCCESS == err) && should_run && !g_pwm_running) {
        err = MOTOR_RPWM_INSTANCE->p_api->start(MOTOR_RPWM_INSTANCE->p_ctrl);
        if (FSP_SUCCESS == err) {
            err = MOTOR_LPWM_INSTANCE->p_api->start(MOTOR_LPWM_INSTANCE->p_ctrl);
            if (FSP_SUCCESS != err) {
                (void) MOTOR_RPWM_INSTANCE->p_api->stop(MOTOR_RPWM_INSTANCE->p_ctrl);
            } else {
                g_pwm_running = TRUE;
            }
        }
    }
    if ((FSP_SUCCESS == err) && should_run) {
        err = g_ioport.p_api->pinWrite(g_ioport.p_ctrl, MOTOR_ENABLE_PIN, BSP_IO_LEVEL_HIGH);
    }
    if ((FSP_SUCCESS == err) && !should_run) {
        (void) g_ioport.p_api->pinWrite(g_ioport.p_ctrl, MOTOR_ENABLE_PIN, BSP_IO_LEVEL_LOW);
        if (g_pwm_running) {
            err = MOTOR_RPWM_INSTANCE->p_api->stop(MOTOR_RPWM_INSTANCE->p_ctrl);
            if (FSP_SUCCESS == err) {
                err = MOTOR_LPWM_INSTANCE->p_api->stop(MOTOR_LPWM_INSTANCE->p_ctrl);
                if (FSP_SUCCESS == err) {
                    g_pwm_running = FALSE;
                }
            }
        }
    }

    return err;
}

/** =================================================================*
 * @brief  DCモーター制御初期化
 * @return FSPエラーコード
 * ================================================================= */
EXPORT fsp_err_t dc_motor_init(void) {
    g_left_target_rpm = 0;
    g_right_target_rpm = 0;
    g_left_target_duty_permille = 0;
    g_right_target_duty_permille = 0;
    g_drive_left_duty_permille = 0;
    g_drive_right_duty_permille = 0;
    g_pwm_update_elapsed_ms = 0U;
    g_speed_feedback_elapsed_ms = 0U;
    g_pwm_running = FALSE;

    fsp_err_t err = MOTOR_RPWM_INSTANCE->p_api->open(MOTOR_RPWM_INSTANCE->p_ctrl, MOTOR_RPWM_INSTANCE->p_cfg);
    if (FSP_SUCCESS == err) {
        err = MOTOR_LPWM_INSTANCE->p_api->open(MOTOR_LPWM_INSTANCE->p_ctrl, MOTOR_LPWM_INSTANCE->p_cfg);
    }
    if (FSP_SUCCESS == err) {
        err = g_ioport.p_api->pinWrite(g_ioport.p_ctrl, MOTOR_ENABLE_PIN, BSP_IO_LEVEL_LOW);
    }
    return err;
}

/** =================================================================*
 * @brief  DCモーターを即時停止
 * @return FSPエラーコード
 * ================================================================= */
EXPORT fsp_err_t dc_motor_stop(void) {
    g_left_target_rpm = 0;
    g_right_target_rpm = 0;
    g_left_target_duty_permille = 0;
    g_right_target_duty_permille = 0;
    g_drive_left_duty_permille = 0;
    g_drive_right_duty_permille = 0;
    g_pwm_update_elapsed_ms = 0U;
    g_speed_feedback_elapsed_ms = 0U;

    (void) g_ioport.p_api->pinWrite(g_ioport.p_ctrl, MOTOR_ENABLE_PIN, BSP_IO_LEVEL_LOW);
    (void) MOTOR_RPWM_INSTANCE->p_api->dutyCycleSet(MOTOR_RPWM_INSTANCE->p_ctrl, 0U, MOTOR_LEFT_RPWM_OUTPUT);
    (void) MOTOR_RPWM_INSTANCE->p_api->dutyCycleSet(MOTOR_RPWM_INSTANCE->p_ctrl, 0U, MOTOR_RIGHT_RPWM_OUTPUT);
    (void) MOTOR_LPWM_INSTANCE->p_api->dutyCycleSet(MOTOR_LPWM_INSTANCE->p_ctrl, 0U, MOTOR_LEFT_LPWM_OUTPUT);
    (void) MOTOR_LPWM_INSTANCE->p_api->dutyCycleSet(MOTOR_LPWM_INSTANCE->p_ctrl, 0U, MOTOR_RIGHT_LPWM_OUTPUT);
    if (g_pwm_running) {
        fsp_err_t err = MOTOR_RPWM_INSTANCE->p_api->stop(MOTOR_RPWM_INSTANCE->p_ctrl);
        if (FSP_SUCCESS == err) {
            err = MOTOR_LPWM_INSTANCE->p_api->stop(MOTOR_LPWM_INSTANCE->p_ctrl);
        }
        if (FSP_SUCCESS != err) {
            return err;
        }
        g_pwm_running = FALSE;
    }

    return FSP_SUCCESS;
}

/** =================================================================*
 * @brief  左右モーターの目標回転数を設定
 * @param[in] left_rpm 左モーター目標回転数（単位: RPM）
 * @param[in] right_rpm 右モーター目標回転数（単位: RPM）
 * @return FSPエラーコード
 * ================================================================= */
EXPORT fsp_err_t dc_motor_request_rpm(H left_rpm, H right_rpm) {
    if ((left_rpm < -JGA25_TARGET_RPM_MAX) || (left_rpm > JGA25_TARGET_RPM_MAX) ||
        (right_rpm < -JGA25_TARGET_RPM_MAX) || (right_rpm > JGA25_TARGET_RPM_MAX)) {
        return FSP_ERR_INVALID_ARGUMENT;
    }

    if ((0 == left_rpm) && (0 == right_rpm)) {
        return dc_motor_stop();
    }

    BOOL const target_changed = (left_rpm != g_left_target_rpm) || (right_rpm != g_right_target_rpm);
    if (target_changed) {
        g_speed_feedback_elapsed_ms = 0U;
    }
    g_left_target_rpm = left_rpm;
    g_right_target_rpm = right_rpm;

    BOOL const feedback_ready = (g_speed_feedback_elapsed_ms >= MOTOR_SPEED_FEEDBACK_START_DELAY_MS);
    g_left_target_duty_permille = motor_rpm_to_feedback_duty_permille(
        left_rpm, MOTOR_LEFT_FORWARD_SIGN, MOTOR_LEFT_DUTY_SCALE_PERMILLE, encoder_left_rpm_get(), feedback_ready);
    g_right_target_duty_permille = motor_rpm_to_feedback_duty_permille(
        right_rpm, MOTOR_RIGHT_FORWARD_SIGN, MOTOR_RIGHT_DUTY_SCALE_PERMILLE, encoder_right_rpm_get(), feedback_ready);
    return FSP_SUCCESS;
}

/** =================================================================*
 * @brief  モーターPWMを1 ms周期で更新
 * @return FSPエラーコード
 * ================================================================= */
EXPORT fsp_err_t dc_motor_housekeeping_1ms(void) {
    if ((0 != g_left_target_rpm) || (0 != g_right_target_rpm)) {
        if (g_speed_feedback_elapsed_ms < UINT32_MAX) {
            g_speed_feedback_elapsed_ms++;
        }
    } else {
        g_speed_feedback_elapsed_ms = 0U;
    }

    g_drive_left_duty_permille = motor_ramp_value(g_drive_left_duty_permille, g_left_target_duty_permille);
    g_drive_right_duty_permille = motor_ramp_value(g_drive_right_duty_permille, g_right_target_duty_permille);

    g_pwm_update_elapsed_ms++;
    if (g_pwm_update_elapsed_ms < MOTOR_PWM_UPDATE_PERIOD_MS) {
        return FSP_SUCCESS;
    }

    g_pwm_update_elapsed_ms = 0U;
    return motor_pwm_apply();
}

/** =================================================================*
 * @brief  左モーター目標回転数取得
 * @return 目標回転数（単位: RPM）
 * ================================================================= */
EXPORT H dc_motor_left_target_rpm_get(void) {
    return g_left_target_rpm;
}

/** =================================================================*
 * @brief  右モーター目標回転数取得
 * @return 目標回転数（単位: RPM）
 * ================================================================= */
EXPORT H dc_motor_right_target_rpm_get(void) {
    return g_right_target_rpm;
}
