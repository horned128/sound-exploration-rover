/** =================================================================*
 * @file   bts7960.c
 * @brief  BTS7960のPWM・有効化ピンドライバ
 * @details 符号付きデューティにより、正値はRPWM、負値はLPWMを選択する。
 * ================================================================= */
#include "drivers/bts7960.h"                                /* BTS7960正逆転PWMドライバAPI */
#include "config/pin_config.h"                              /* BTS7960のFSP資源・出力ピン対応 */

LOCAL BOOL bts7960_pwm_running;                              /**< PWM開始状態 */

/** =================================================================*
 * @brief  BTS7960 PWM資源初期化
 * @details 正逆転用のPWMをopenし、初期出力を無効にする。
 * @return FSPエラーコード
 * ================================================================= */
EXPORT fsp_err_t bts7960_init(void) {
    bts7960_pwm_running = FALSE;

    fsp_err_t err = BTS7960_RPWM_INSTANCE->p_api->open(BTS7960_RPWM_INSTANCE->p_ctrl,
                                                        BTS7960_RPWM_INSTANCE->p_cfg);
    if (FSP_SUCCESS == err) {
        err = BTS7960_LPWM_INSTANCE->p_api->open(BTS7960_LPWM_INSTANCE->p_ctrl,
                                                  BTS7960_LPWM_INSTANCE->p_cfg);
    }
    if (FSP_SUCCESS == err) {
        err = g_ioport.p_api->pinWrite(g_ioport.p_ctrl, BTS7960_ENABLE_PIN, BSP_IO_LEVEL_LOW);
    }
    return err;
}

/** =================================================================*
 * @brief  BTS7960出力安全停止
 * @details 有効化ピンと全PWMデューティを0にし、開始済みのPWMを停止する。
 * @return FSPエラーコード
 * ================================================================= */
EXPORT fsp_err_t bts7960_stop(void) {
    (void) g_ioport.p_api->pinWrite(g_ioport.p_ctrl, BTS7960_ENABLE_PIN, BSP_IO_LEVEL_LOW);
    (void) BTS7960_RPWM_INSTANCE->p_api->dutyCycleSet(BTS7960_RPWM_INSTANCE->p_ctrl, 0U,
                                                       BTS7960_LEFT_RPWM_OUTPUT);
    (void) BTS7960_RPWM_INSTANCE->p_api->dutyCycleSet(BTS7960_RPWM_INSTANCE->p_ctrl, 0U,
                                                       BTS7960_RIGHT_RPWM_OUTPUT);
    (void) BTS7960_LPWM_INSTANCE->p_api->dutyCycleSet(BTS7960_LPWM_INSTANCE->p_ctrl, 0U,
                                                       BTS7960_LEFT_LPWM_OUTPUT);
    (void) BTS7960_LPWM_INSTANCE->p_api->dutyCycleSet(BTS7960_LPWM_INSTANCE->p_ctrl, 0U,
                                                       BTS7960_RIGHT_LPWM_OUTPUT);
    if (bts7960_pwm_running) {
        fsp_err_t err = BTS7960_RPWM_INSTANCE->p_api->stop(BTS7960_RPWM_INSTANCE->p_ctrl);
        if (FSP_SUCCESS == err) {
            err = BTS7960_LPWM_INSTANCE->p_api->stop(BTS7960_LPWM_INSTANCE->p_ctrl);
        }
        if (FSP_SUCCESS != err) {
            return err;
        }
        bts7960_pwm_running = FALSE;
    }

    return FSP_SUCCESS;
}

/** =================================================================*
 * @brief  左右モーター符号付きデューティ設定
 * @details 正値はRPWM、負値はLPWMへ出力し、正逆転の同時出力を防止する。
 * @param[in] left_duty_permille 左モーターのデューティ[0.1%]
 * @param[in] right_duty_permille 右モーターのデューティ[0.1%]
 * @return FSPエラーコード
 * ================================================================= */
EXPORT fsp_err_t bts7960_set_signed_duty(H left_duty_permille, H right_duty_permille) {
    timer_info_t rpwm_info = {0};
    timer_info_t lpwm_info = {0};
    fsp_err_t err = BTS7960_RPWM_INSTANCE->p_api->infoGet(BTS7960_RPWM_INSTANCE->p_ctrl, &rpwm_info);
    if (FSP_SUCCESS != err) {
        return err;
    }
    err = BTS7960_LPWM_INSTANCE->p_api->infoGet(BTS7960_LPWM_INSTANCE->p_ctrl, &lpwm_info);
    if (FSP_SUCCESS != err) {
        return err;
    }

    UH const left_magnitude = (UH) ((left_duty_permille < 0) ? -left_duty_permille : left_duty_permille);
    UH const right_magnitude = (UH) ((right_duty_permille < 0) ? -right_duty_permille : right_duty_permille);
    UH const left_rpwm = (left_duty_permille > 0) ? left_magnitude : 0U;
    UH const left_lpwm = (left_duty_permille < 0) ? left_magnitude : 0U;
    UH const right_rpwm = (right_duty_permille > 0) ? right_magnitude : 0U;
    UH const right_lpwm = (right_duty_permille < 0) ? right_magnitude : 0U;

    UW const left_rpwm_counts = (UW) (((UD) rpwm_info.period_counts * left_rpwm) / 1000U);
    UW const right_rpwm_counts = (UW) (((UD) rpwm_info.period_counts * right_rpwm) / 1000U);
    UW const left_lpwm_counts = (UW) (((UD) lpwm_info.period_counts * left_lpwm) / 1000U);
    UW const right_lpwm_counts = (UW) (((UD) lpwm_info.period_counts * right_lpwm) / 1000U);

    /* 既存のブレーク・ビフォア・メーク手順を維持する。 */
    err = BTS7960_RPWM_INSTANCE->p_api->dutyCycleSet(BTS7960_RPWM_INSTANCE->p_ctrl, 0U,
                                                     BTS7960_LEFT_RPWM_OUTPUT);
    if (FSP_SUCCESS == err) {
        err = BTS7960_RPWM_INSTANCE->p_api->dutyCycleSet(BTS7960_RPWM_INSTANCE->p_ctrl, 0U,
                                                         BTS7960_RIGHT_RPWM_OUTPUT);
    }
    if (FSP_SUCCESS == err) {
        err = BTS7960_LPWM_INSTANCE->p_api->dutyCycleSet(BTS7960_LPWM_INSTANCE->p_ctrl, 0U,
                                                         BTS7960_LEFT_LPWM_OUTPUT);
    }
    if (FSP_SUCCESS == err) {
        err = BTS7960_LPWM_INSTANCE->p_api->dutyCycleSet(BTS7960_LPWM_INSTANCE->p_ctrl, 0U,
                                                         BTS7960_RIGHT_LPWM_OUTPUT);
    }
    if (FSP_SUCCESS == err) {
        err = BTS7960_RPWM_INSTANCE->p_api->dutyCycleSet(BTS7960_RPWM_INSTANCE->p_ctrl, left_rpwm_counts,
                                                         BTS7960_LEFT_RPWM_OUTPUT);
    }
    if (FSP_SUCCESS == err) {
        err = BTS7960_RPWM_INSTANCE->p_api->dutyCycleSet(BTS7960_RPWM_INSTANCE->p_ctrl, right_rpwm_counts,
                                                         BTS7960_RIGHT_RPWM_OUTPUT);
    }
    if (FSP_SUCCESS == err) {
        err = BTS7960_LPWM_INSTANCE->p_api->dutyCycleSet(BTS7960_LPWM_INSTANCE->p_ctrl, left_lpwm_counts,
                                                         BTS7960_LEFT_LPWM_OUTPUT);
    }
    if (FSP_SUCCESS == err) {
        err = BTS7960_LPWM_INSTANCE->p_api->dutyCycleSet(BTS7960_LPWM_INSTANCE->p_ctrl, right_lpwm_counts,
                                                         BTS7960_RIGHT_LPWM_OUTPUT);
    }

    BOOL const should_run = (0U != left_magnitude) || (0U != right_magnitude);
    if ((FSP_SUCCESS == err) && should_run && !bts7960_pwm_running) {
        err = BTS7960_RPWM_INSTANCE->p_api->start(BTS7960_RPWM_INSTANCE->p_ctrl);
        if (FSP_SUCCESS == err) {
            err = BTS7960_LPWM_INSTANCE->p_api->start(BTS7960_LPWM_INSTANCE->p_ctrl);
            if (FSP_SUCCESS != err) {
                (void) BTS7960_RPWM_INSTANCE->p_api->stop(BTS7960_RPWM_INSTANCE->p_ctrl);
            } else {
                bts7960_pwm_running = TRUE;
            }
        }
    }
    if ((FSP_SUCCESS == err) && should_run) {
        err = g_ioport.p_api->pinWrite(g_ioport.p_ctrl, BTS7960_ENABLE_PIN, BSP_IO_LEVEL_HIGH);
    }
    if ((FSP_SUCCESS == err) && !should_run) {
        (void) g_ioport.p_api->pinWrite(g_ioport.p_ctrl, BTS7960_ENABLE_PIN, BSP_IO_LEVEL_LOW);
        if (bts7960_pwm_running) {
            err = BTS7960_RPWM_INSTANCE->p_api->stop(BTS7960_RPWM_INSTANCE->p_ctrl);
            if (FSP_SUCCESS == err) {
                err = BTS7960_LPWM_INSTANCE->p_api->stop(BTS7960_LPWM_INSTANCE->p_ctrl);
                if (FSP_SUCCESS == err) {
                    bts7960_pwm_running = FALSE;
                }
            }
        }
    }

    return err;
}
