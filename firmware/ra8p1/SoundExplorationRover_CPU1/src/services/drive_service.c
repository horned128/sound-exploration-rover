/** =================================================================*
 * @file   drive_service.c
 * @brief  RPMからデューティへの変換、左右校正、変化率制限のサービス
 * ================================================================= */
#include "services/drive_service.h"                         /* 車体駆動サービスAPI */
#include "config/drive_config.h"                            /* RPM、デューティ、速度帰還設定 */
#include "drivers/bts7960.h"                                /* BTS7960正逆転PWM出力API */
#include "drivers/encoder.h"                                /* 左右モーター回転数取得API */

EXPORT volatile H g_drive_left_duty_permille = 0;            /**< 左モーターの現在デューティ[0.1%] */
EXPORT volatile H g_drive_right_duty_permille = 0;           /**< 右モーターの現在デューティ[0.1%] */

LOCAL H g_left_target_rpm;                                   /**< 左モーター目標回転数[RPM] */
LOCAL H g_right_target_rpm;                                  /**< 右モーター目標回転数[RPM] */
LOCAL H g_left_target_duty_permille;                         /**< 左モーター目標デューティ[0.1%] */
LOCAL H g_right_target_duty_permille;                        /**< 右モーター目標デューティ[0.1%] */
LOCAL UW g_drive_update_elapsed_ms;                          /**< PWM更新周期の経過時間[ms] */
LOCAL UW g_speed_feedback_elapsed_ms;                        /**< 目標変更後の速度帰還待機時間[ms] */

/** =================================================================*
 * @brief  目標回転数から符号付きデューティへ変換
 * @param[in] target_rpm 車体座標の目標回転数[RPM]
 * @param[in] forward_sign 車体前進へ変換するモーター極性
 * @return 符号付きデューティ[0.1%]
 * ================================================================= */
LOCAL H drive_service_rpm_to_duty_permille(H target_rpm, B forward_sign) {
    W const signed_rpm = (W) target_rpm * (W) forward_sign;
    if (0 == signed_rpm) {
        return 0;
    }

    W const magnitude = (signed_rpm < 0) ? -signed_rpm : signed_rpm;
    W duty = (magnitude * 1000) / DRIVE_TARGET_RPM_MAX;
    if (duty < DRIVE_DUTY_MIN_PERMILLE) {
        duty = DRIVE_DUTY_MIN_PERMILLE;
    }
    if (duty > DRIVE_DUTY_MAX_PERMILLE) {
        duty = DRIVE_DUTY_MAX_PERMILLE;
    }
    return (signed_rpm < 0) ? (H) -duty : (H) duty;
}

/** =================================================================*
 * @brief  速度帰還を反映した符号付きデューティ算出
 * @param[in] target_rpm 車体座標の目標回転数[RPM]
 * @param[in] forward_sign 車体前進へ変換するモーター極性
 * @param[in] duty_scale_permille 左右校正係数[0.1%]
 * @param[in] measured_rpm 実測回転数[RPM]
 * @param[in] feedback_ready 速度帰還の開始可否
 * @return 符号付きデューティ[0.1%]
 * ================================================================= */
LOCAL H drive_service_feedback_duty_permille(H target_rpm, B forward_sign, UH duty_scale_permille,
                                             H measured_rpm, BOOL feedback_ready) {
    H const raw_base_duty = drive_service_rpm_to_duty_permille(target_rpm, forward_sign);
    W base_magnitude = (raw_base_duty < 0) ? -(W) raw_base_duty : raw_base_duty;
    base_magnitude = (base_magnitude * duty_scale_permille) / 1000;
    if (base_magnitude > DRIVE_DUTY_MAX_PERMILLE) {
        base_magnitude = DRIVE_DUTY_MAX_PERMILLE;
    }
    H const base_duty = (raw_base_duty < 0) ? (H) -base_magnitude : (H) base_magnitude;
    if ((0U == DRIVE_SPEED_FEEDBACK_ENABLE) || !feedback_ready || (0 == target_rpm)) {
        return base_duty;
    }

    W const target_magnitude = (target_rpm < 0) ? -(W) target_rpm : target_rpm;
    W const measured_in_target_direction = (target_rpm < 0) ? -(W) measured_rpm : measured_rpm;
    W correction = (target_magnitude - measured_in_target_direction) * DRIVE_SPEED_FEEDBACK_KP_PERMILLE_PER_RPM;
    if (correction > DRIVE_SPEED_FEEDBACK_MAX_CORRECTION_PERMILLE) {
        correction = DRIVE_SPEED_FEEDBACK_MAX_CORRECTION_PERMILLE;
    } else if (correction < -DRIVE_SPEED_FEEDBACK_MAX_CORRECTION_PERMILLE) {
        correction = -DRIVE_SPEED_FEEDBACK_MAX_CORRECTION_PERMILLE;
    }

    W duty_magnitude = (base_duty < 0) ? -(W) base_duty : base_duty;
    duty_magnitude += correction;
    if (duty_magnitude < DRIVE_DUTY_MIN_PERMILLE) {
        duty_magnitude = DRIVE_DUTY_MIN_PERMILLE;
    } else if (duty_magnitude > DRIVE_DUTY_MAX_PERMILLE) {
        duty_magnitude = DRIVE_DUTY_MAX_PERMILLE;
    }
    return (base_duty < 0) ? (H) -duty_magnitude : (H) duty_magnitude;
}

/** =================================================================*
 * @brief  デューティ変化率制限
 * @param[in] current 現在デューティ[0.1%]
 * @param[in] target 目標デューティ[0.1%]
 * @param[in] elapsed_ms 前回更新からの実経過時間[ms]
 * @return 次周期のデューティ[0.1%]
 * ================================================================= */
LOCAL H drive_service_ramp_value(H current, H target, UW elapsed_ms) {
    UD const step = (UD) DRIVE_RAMP_PER_MS * elapsed_ms;
    W const distance = (W) target - current;
    if (distance > 0) {
        return (step >= (UD) distance) ? target : (H) ((W) current + (W) step);
    }
    if (distance < 0) {
        return (step >= (UD) -distance) ? target : (H) ((W) current - (W) step);
    }
    return current;
}

/** =================================================================*
 * @brief  車体駆動サービス初期化
 * @return FSPエラーコード
 * ================================================================= */
EXPORT fsp_err_t drive_service_init(void) {
    g_left_target_rpm = 0;
    g_right_target_rpm = 0;
    g_left_target_duty_permille = 0;
    g_right_target_duty_permille = 0;
    g_drive_left_duty_permille = 0;
    g_drive_right_duty_permille = 0;
    g_drive_update_elapsed_ms = 0U;
    g_speed_feedback_elapsed_ms = 0U;
    return bts7960_init();
}

/** =================================================================*
 * @brief  車体駆動サービス安全停止
 * @return FSPエラーコード
 * ================================================================= */
EXPORT fsp_err_t drive_service_stop(void) {
    g_left_target_rpm = 0;
    g_right_target_rpm = 0;
    g_left_target_duty_permille = 0;
    g_right_target_duty_permille = 0;
    g_drive_left_duty_permille = 0;
    g_drive_right_duty_permille = 0;
    g_drive_update_elapsed_ms = 0U;
    g_speed_feedback_elapsed_ms = 0U;
    return bts7960_stop();
}

/** =================================================================*
 * @brief  左右モーター目標回転数設定
 * @param[in] left_rpm 左モーター目標回転数[RPM]
 * @param[in] right_rpm 右モーター目標回転数[RPM]
 * @return FSPエラーコード
 * ================================================================= */
EXPORT fsp_err_t drive_service_set_target_rpm(H left_rpm, H right_rpm) {
    if ((left_rpm < -DRIVE_TARGET_RPM_MAX) || (left_rpm > DRIVE_TARGET_RPM_MAX) ||
        (right_rpm < -DRIVE_TARGET_RPM_MAX) || (right_rpm > DRIVE_TARGET_RPM_MAX)) {
        return FSP_ERR_INVALID_ARGUMENT;
    }
    if ((0 == left_rpm) && (0 == right_rpm)) {
        return drive_service_stop();
    }

    BOOL const target_changed = (left_rpm != g_left_target_rpm) || (right_rpm != g_right_target_rpm);
    if (target_changed) {
        g_speed_feedback_elapsed_ms = 0U;
    }
    g_left_target_rpm = left_rpm;
    g_right_target_rpm = right_rpm;

    BOOL const feedback_ready = (g_speed_feedback_elapsed_ms >= DRIVE_SPEED_FEEDBACK_START_DELAY_MS);
    g_left_target_duty_permille = drive_service_feedback_duty_permille(
        left_rpm, DRIVE_LEFT_FORWARD_SIGN, DRIVE_LEFT_DUTY_SCALE_PERMILLE, encoder_left_rpm_get(), feedback_ready);
    g_right_target_duty_permille = drive_service_feedback_duty_permille(
        right_rpm, DRIVE_RIGHT_FORWARD_SIGN, DRIVE_RIGHT_DUTY_SCALE_PERMILLE, encoder_right_rpm_get(), feedback_ready);
    return FSP_SUCCESS;
}

#if DRIVE_MEASUREMENT_TEST_ENABLE
/** =================================================================*
 * @brief  実機計測用の左右同値前進デューティ設定
 * @param[in] magnitude_permille 前進デューティの大きさ[0.1%]
 * @return FSPエラーコード
 * @details 出力の所有権はdrive_serviceに残し、通常のランプとPWM更新を通す。
 *          上限の保護は呼出し側でも行うが、このAPI単体でも検査する。
 * ================================================================= */
EXPORT fsp_err_t drive_service_set_test_duty_permille(UH magnitude_permille) {
    if (magnitude_permille > DRIVE_DUTY_MAX_PERMILLE) {
        return FSP_ERR_INVALID_ARGUMENT;
    }

    g_left_target_duty_permille =
        (DRIVE_LEFT_FORWARD_SIGN < 0) ? (H) -(W) magnitude_permille : (H) magnitude_permille;
    g_right_target_duty_permille =
        (DRIVE_RIGHT_FORWARD_SIGN < 0) ? (H) -(W) magnitude_permille : (H) magnitude_permille;
    return FSP_SUCCESS;
}
#endif

/** =================================================================*
 * @brief  車体駆動実時間更新
 * @param[in] elapsed_ms 前回更新からの実経過時間[ms]（初回0）
 * @return FSPエラーコード
 * ================================================================= */
EXPORT fsp_err_t drive_service_update(UW elapsed_ms) {
    if ((0 != g_left_target_rpm) || (0 != g_right_target_rpm)) {
        UW const remaining = UINT32_MAX - g_speed_feedback_elapsed_ms;
        g_speed_feedback_elapsed_ms += (elapsed_ms < remaining) ? elapsed_ms : remaining;
    } else {
        g_speed_feedback_elapsed_ms = 0U;
    }

    g_drive_left_duty_permille =
        drive_service_ramp_value(g_drive_left_duty_permille, g_left_target_duty_permille, elapsed_ms);
    g_drive_right_duty_permille =
        drive_service_ramp_value(g_drive_right_duty_permille, g_right_target_duty_permille, elapsed_ms);

    UD const update_elapsed_ms = (UD) g_drive_update_elapsed_ms + elapsed_ms;
    g_drive_update_elapsed_ms = (UW) (update_elapsed_ms % DRIVE_UPDATE_PERIOD_MS);
    if (update_elapsed_ms < DRIVE_UPDATE_PERIOD_MS) {
        return FSP_SUCCESS;
    }
    return bts7960_set_signed_duty(g_drive_left_duty_permille, g_drive_right_duty_permille);
}

/** =================================================================*
 * @brief  左モーター目標回転数取得
 * @return 左モーター目標回転数[RPM]
 * ================================================================= */
EXPORT H drive_service_left_target_rpm_get(void) {
    return g_left_target_rpm;
}

/** =================================================================*
 * @brief  右モーター目標回転数取得
 * @return 右モーター目標回転数[RPM]
 * ================================================================= */
EXPORT H drive_service_right_target_rpm_get(void) {
    return g_right_target_rpm;
}
