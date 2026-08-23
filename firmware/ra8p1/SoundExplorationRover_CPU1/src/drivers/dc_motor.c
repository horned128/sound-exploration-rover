/** =================================================================*
 * @file   dc_motor.c
 * @brief  左右BTS7960モーターPWM制御
 * @details RPWMとLPWMを各方向へ独立出力し、方向切替時は両出力を一度停止する。
 * ================================================================= */
#include "dc_motor.h"                                       /* DCモーター制御API */
#include "../cpu1_config.h"                                 /* モーター制御設定 */

volatile int16_t g_drive_left_duty_permille = 0;            /**< 左モーター出力指令（単位: 1/1000） */
volatile int16_t g_drive_right_duty_permille = 0;           /**< 右モーター出力指令（単位: 1/1000） */

static int16_t g_left_target_rpm;                           /**< 左モーター目標回転数（単位: RPM） */
static int16_t g_right_target_rpm;                          /**< 右モーター目標回転数（単位: RPM） */
static int16_t g_left_target_duty_permille;                 /**< 左モーター目標デューティ（単位: 1/1000） */
static int16_t g_right_target_duty_permille;                /**< 右モーター目標デューティ（単位: 1/1000） */
static uint32_t g_pwm_update_elapsed_ms;                    /**< PWM更新周期の経過時間（単位: ms） */
static bool g_pwm_running;                                  /**< PWMタイマの動作状態 */

/** =================================================================*
 * @brief  回転数をデューティへ変換
 * @param[in] target_rpm 目標回転数（単位: RPM）
 * @param[in] forward_sign 論理前進方向の極性
 * @return 符号付きデューティ（単位: 1/1000）
 * ================================================================= */
static int16_t motor_rpm_to_duty_permille(int16_t target_rpm, int8_t forward_sign) {
    int32_t const signed_rpm = (int32_t) target_rpm * (int32_t) forward_sign;
    if (0 == signed_rpm) {
        return 0;
    }

    int32_t const magnitude = (signed_rpm < 0) ? -signed_rpm : signed_rpm;
    int32_t duty = (magnitude * 1000) / JGA25_TARGET_RPM_MAX;
    if (duty < MOTOR_PWM_MIN_DUTY_PERMILLE) {
        duty = MOTOR_PWM_MIN_DUTY_PERMILLE;
    }
    if (duty > MOTOR_PWM_MAX_DUTY_PERMILLE) {
        duty = MOTOR_PWM_MAX_DUTY_PERMILLE;
    }

    return (signed_rpm < 0) ? (int16_t) -duty : (int16_t) duty;
}

/** =================================================================*
 * @brief  PWMデューティを目標値へ近づける
 * @param[in] current 現在値（単位: 1/1000）
 * @param[in] target 目標値（単位: 1/1000）
 * @return 更新後のデューティ（単位: 1/1000）
 * ================================================================= */
static int16_t motor_ramp_value(int16_t current, int16_t target) {
    if (current < target) {
        int32_t const next = (int32_t) current + MOTOR_PWM_RAMP_PER_MS;
        return (int16_t) ((next > target) ? target : next);
    }
    if (current > target) {
        int32_t const next = (int32_t) current - MOTOR_PWM_RAMP_PER_MS;
        return (int16_t) ((next < target) ? target : next);
    }

    return current;
}

/** =================================================================*
 * @brief  4系統のPWM出力を更新
 * @return FSPエラーコード
 * @details 方向切替時は旧出力を先に0へ戻し、BTS7960のRPWMとLPWMを同時に
 *          有効にしない。デューティが0のときは共通ENも無効にする。
 * ================================================================= */
static fsp_err_t motor_pwm_apply(void) {
    timer_info_t rpwm_info = {0};
    timer_info_t lpwm_info = {0};
    fsp_err_t err = MOTOR_RPWM_INSTANCE->p_api->infoGet(MOTOR_RPWM_INSTANCE->p_ctrl,
                                                         &rpwm_info);
    if (FSP_SUCCESS != err) {
        return err;
    }

    err = MOTOR_LPWM_INSTANCE->p_api->infoGet(MOTOR_LPWM_INSTANCE->p_ctrl,
                                               &lpwm_info);
    if (FSP_SUCCESS != err) {
        return err;
    }

    int16_t const left_duty = g_drive_left_duty_permille;
    int16_t const right_duty = g_drive_right_duty_permille;
    uint16_t const left_magnitude = (uint16_t) ((left_duty < 0) ? -left_duty : left_duty);
    uint16_t const right_magnitude = (uint16_t) ((right_duty < 0) ? -right_duty : right_duty);
    uint16_t const left_rpwm = (left_duty > 0) ? left_magnitude : 0U;
    uint16_t const left_lpwm = (left_duty < 0) ? left_magnitude : 0U;
    uint16_t const right_rpwm = (right_duty > 0) ? right_magnitude : 0U;
    uint16_t const right_lpwm = (right_duty < 0) ? right_magnitude : 0U;

    uint32_t const left_rpwm_counts =
        (uint32_t) (((uint64_t) rpwm_info.period_counts * left_rpwm) / 1000U);
    uint32_t const right_rpwm_counts =
        (uint32_t) (((uint64_t) rpwm_info.period_counts * right_rpwm) / 1000U);
    uint32_t const left_lpwm_counts =
        (uint32_t) (((uint64_t) lpwm_info.period_counts * left_lpwm) / 1000U);
    uint32_t const right_lpwm_counts =
        (uint32_t) (((uint64_t) lpwm_info.period_counts * right_lpwm) / 1000U);

    /* 方向を切り替える前に4出力を停止し、RPWMとLPWMの同時有効を防ぐ。 */
    err = MOTOR_RPWM_INSTANCE->p_api->dutyCycleSet(MOTOR_RPWM_INSTANCE->p_ctrl,
                                                    0U,
                                                    MOTOR_LEFT_RPWM_OUTPUT);
    if (FSP_SUCCESS == err) {
        err = MOTOR_RPWM_INSTANCE->p_api->dutyCycleSet(MOTOR_RPWM_INSTANCE->p_ctrl,
                                                        0U,
                                                        MOTOR_RIGHT_RPWM_OUTPUT);
    }
    if (FSP_SUCCESS == err) {
        err = MOTOR_LPWM_INSTANCE->p_api->dutyCycleSet(MOTOR_LPWM_INSTANCE->p_ctrl,
                                                        0U,
                                                        MOTOR_LEFT_LPWM_OUTPUT);
    }
    if (FSP_SUCCESS == err) {
        err = MOTOR_LPWM_INSTANCE->p_api->dutyCycleSet(MOTOR_LPWM_INSTANCE->p_ctrl,
                                                        0U,
                                                        MOTOR_RIGHT_LPWM_OUTPUT);
    }
    if (FSP_SUCCESS == err) {
        err = MOTOR_RPWM_INSTANCE->p_api->dutyCycleSet(MOTOR_RPWM_INSTANCE->p_ctrl,
                                                        left_rpwm_counts,
                                                        MOTOR_LEFT_RPWM_OUTPUT);
    }
    if (FSP_SUCCESS == err) {
        err = MOTOR_RPWM_INSTANCE->p_api->dutyCycleSet(MOTOR_RPWM_INSTANCE->p_ctrl,
                                                        right_rpwm_counts,
                                                        MOTOR_RIGHT_RPWM_OUTPUT);
    }
    if (FSP_SUCCESS == err) {
        err = MOTOR_LPWM_INSTANCE->p_api->dutyCycleSet(MOTOR_LPWM_INSTANCE->p_ctrl,
                                                        left_lpwm_counts,
                                                        MOTOR_LEFT_LPWM_OUTPUT);
    }
    if (FSP_SUCCESS == err) {
        err = MOTOR_LPWM_INSTANCE->p_api->dutyCycleSet(MOTOR_LPWM_INSTANCE->p_ctrl,
                                                        right_lpwm_counts,
                                                        MOTOR_RIGHT_LPWM_OUTPUT);
    }

    bool const should_run = (0U != left_magnitude) || (0U != right_magnitude);
    if ((FSP_SUCCESS == err) && should_run && !g_pwm_running) {
        err = MOTOR_RPWM_INSTANCE->p_api->start(MOTOR_RPWM_INSTANCE->p_ctrl);
        if (FSP_SUCCESS == err) {
            err = MOTOR_LPWM_INSTANCE->p_api->start(MOTOR_LPWM_INSTANCE->p_ctrl);
            if (FSP_SUCCESS != err) {
                (void) MOTOR_RPWM_INSTANCE->p_api->stop(MOTOR_RPWM_INSTANCE->p_ctrl);
            } else {
                g_pwm_running = true;
            }
        }
    }
    if ((FSP_SUCCESS == err) && should_run) {
        err = g_ioport.p_api->pinWrite(g_ioport.p_ctrl,
                                       MOTOR_ENABLE_PIN,
                                       BSP_IO_LEVEL_HIGH);
    }
    if ((FSP_SUCCESS == err) && !should_run) {
        (void) g_ioport.p_api->pinWrite(g_ioport.p_ctrl,
                                        MOTOR_ENABLE_PIN,
                                        BSP_IO_LEVEL_LOW);
        if (g_pwm_running) {
            err = MOTOR_RPWM_INSTANCE->p_api->stop(MOTOR_RPWM_INSTANCE->p_ctrl);
            if (FSP_SUCCESS == err) {
                err = MOTOR_LPWM_INSTANCE->p_api->stop(MOTOR_LPWM_INSTANCE->p_ctrl);
                if (FSP_SUCCESS == err) {
                    g_pwm_running = false;
                }
            }
        }
    }

    return err;
}

/** =================================================================*
 * @brief  DCモーター制御を初期化
 * @return FSPエラーコード
 * ================================================================= */
fsp_err_t dc_motor_init(void) {
    g_left_target_rpm = 0;
    g_right_target_rpm = 0;
    g_left_target_duty_permille = 0;
    g_right_target_duty_permille = 0;
    g_drive_left_duty_permille = 0;
    g_drive_right_duty_permille = 0;
    g_pwm_update_elapsed_ms = 0U;
    g_pwm_running = false;

    fsp_err_t err = MOTOR_RPWM_INSTANCE->p_api->open(MOTOR_RPWM_INSTANCE->p_ctrl,
                                                     MOTOR_RPWM_INSTANCE->p_cfg);
    if (FSP_SUCCESS == err) {
        err = MOTOR_LPWM_INSTANCE->p_api->open(MOTOR_LPWM_INSTANCE->p_ctrl,
                                               MOTOR_LPWM_INSTANCE->p_cfg);
    }
    if (FSP_SUCCESS == err) {
        err = g_ioport.p_api->pinWrite(g_ioport.p_ctrl,
                                       MOTOR_ENABLE_PIN,
                                       BSP_IO_LEVEL_LOW);
    }
    return err;
}

/** =================================================================*
 * @brief  DCモーターを即時停止
 * @return FSPエラーコード
 * ================================================================= */
fsp_err_t dc_motor_stop(void) {
    g_left_target_rpm = 0;
    g_right_target_rpm = 0;
    g_left_target_duty_permille = 0;
    g_right_target_duty_permille = 0;
    g_drive_left_duty_permille = 0;
    g_drive_right_duty_permille = 0;
    g_pwm_update_elapsed_ms = 0U;

    (void) g_ioport.p_api->pinWrite(g_ioport.p_ctrl,
                                    MOTOR_ENABLE_PIN,
                                    BSP_IO_LEVEL_LOW);
    (void) MOTOR_RPWM_INSTANCE->p_api->dutyCycleSet(MOTOR_RPWM_INSTANCE->p_ctrl,
                                                     0U,
                                                     MOTOR_LEFT_RPWM_OUTPUT);
    (void) MOTOR_RPWM_INSTANCE->p_api->dutyCycleSet(MOTOR_RPWM_INSTANCE->p_ctrl,
                                                     0U,
                                                     MOTOR_RIGHT_RPWM_OUTPUT);
    (void) MOTOR_LPWM_INSTANCE->p_api->dutyCycleSet(MOTOR_LPWM_INSTANCE->p_ctrl,
                                                     0U,
                                                     MOTOR_LEFT_LPWM_OUTPUT);
    (void) MOTOR_LPWM_INSTANCE->p_api->dutyCycleSet(MOTOR_LPWM_INSTANCE->p_ctrl,
                                                     0U,
                                                     MOTOR_RIGHT_LPWM_OUTPUT);
    if (g_pwm_running) {
        fsp_err_t err = MOTOR_RPWM_INSTANCE->p_api->stop(MOTOR_RPWM_INSTANCE->p_ctrl);
        if (FSP_SUCCESS == err) {
            err = MOTOR_LPWM_INSTANCE->p_api->stop(MOTOR_LPWM_INSTANCE->p_ctrl);
        }
        if (FSP_SUCCESS != err) {
            return err;
        }
        g_pwm_running = false;
    }

    return FSP_SUCCESS;
}

/** =================================================================*
 * @brief  左右モーターの目標回転数を設定
 * @param[in] left_rpm 左モーター目標回転数（単位: RPM）
 * @param[in] right_rpm 右モーター目標回転数（単位: RPM）
 * @return FSPエラーコード
 * ================================================================= */
fsp_err_t dc_motor_request_rpm(int16_t left_rpm, int16_t right_rpm) {
    if ((left_rpm < -JGA25_TARGET_RPM_MAX) ||
        (left_rpm > JGA25_TARGET_RPM_MAX) ||
        (right_rpm < -JGA25_TARGET_RPM_MAX) ||
        (right_rpm > JGA25_TARGET_RPM_MAX)) {
        return FSP_ERR_INVALID_ARGUMENT;
    }

    g_left_target_rpm = left_rpm;
    g_right_target_rpm = right_rpm;
    g_left_target_duty_permille = motor_rpm_to_duty_permille(left_rpm,
                                                              MOTOR_LEFT_FORWARD_SIGN);
    g_right_target_duty_permille = motor_rpm_to_duty_permille(right_rpm,
                                                               MOTOR_RIGHT_FORWARD_SIGN);
    return FSP_SUCCESS;
}

/** =================================================================*
 * @brief  モーターPWMを1 ms周期で更新
 * @return FSPエラーコード
 * ================================================================= */
fsp_err_t dc_motor_housekeeping_1ms(void) {
    g_drive_left_duty_permille = motor_ramp_value(g_drive_left_duty_permille,
                                                   g_left_target_duty_permille);
    g_drive_right_duty_permille = motor_ramp_value(g_drive_right_duty_permille,
                                                    g_right_target_duty_permille);

    g_pwm_update_elapsed_ms++;
    if (g_pwm_update_elapsed_ms < MOTOR_PWM_UPDATE_PERIOD_MS) {
        return FSP_SUCCESS;
    }

    g_pwm_update_elapsed_ms = 0U;
    return motor_pwm_apply();
}

/** =================================================================*
 * @brief  左モーター目標回転数を取得
 * @return 目標回転数（単位: RPM）
 * ================================================================= */
int16_t dc_motor_left_target_rpm_get(void) {
    return g_left_target_rpm;
}

/** =================================================================*
 * @brief  右モーター目標回転数を取得
 * @return 目標回転数（単位: RPM）
 * ================================================================= */
int16_t dc_motor_right_target_rpm_get(void) {
    return g_right_target_rpm;
}
