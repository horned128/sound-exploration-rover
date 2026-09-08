/** =================================================================*
 * @file   encoder.c
 * @brief  左右代表モーターのA/B相エンコーダ取得
 * @details 前進を正、後進を負として累積カウントと推定回転数を更新する。
 * ================================================================= */
#include "encoder.h"                                        /* エンコーダAPI */
#include "config/drive_config.h"                           /* エンコーダ設定 */
#include "config/pin_config.h"                             /* エンコーダGPIO */

EXPORT volatile W g_encoder_left_count = 0;            /**< 左代表モーター累積カウント（前進正） */
EXPORT volatile W g_encoder_left_rpm_x10 = 0;                  /**< 左代表モーター推定回転数の10倍（前進正） */
EXPORT volatile W g_encoder_right_count = 0;           /**< 右代表モーター累積カウント（前進正） */
EXPORT volatile W g_encoder_right_rpm_x10 = 0;                 /**< 右代表モーター推定回転数の10倍（前進正） */

LOCAL UB g_left_encoder_previous_ab;                  /**< 左エンコーダ前回A/B状態 */
LOCAL UB g_right_encoder_previous_ab;                 /**< 右エンコーダ前回A/B状態 */
LOCAL UW g_speed_elapsed_ms;                         /**< 速度算出周期の経過時間（単位: ms） */
LOCAL W g_left_speed_previous_count;                 /**< 左速度算出時の前回カウント */
LOCAL W g_right_speed_previous_count;                /**< 右速度算出時の前回カウント */

/**< A/B相の遷移量テーブル（4逓倍） */
LOCAL B const encoder_transition_delta[16] = {
    0, 1, -1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, 1, 0,
};

/** =================================================================*
 * @brief  A/B相カウント更新
 * @param[in] left trueなら左代表モーター、falseなら右代表モーター
 * ================================================================= */
LOCAL void encoder_update(BOOL left) {
    bsp_io_port_pin_t const pin_a = left ? ENCODER_LEFT_A_PIN : ENCODER_RIGHT_A_PIN;
    bsp_io_port_pin_t const pin_b = left ? ENCODER_LEFT_B_PIN : ENCODER_RIGHT_B_PIN;
    bsp_io_level_t level_a = BSP_IO_LEVEL_LOW;
    bsp_io_level_t level_b = BSP_IO_LEVEL_LOW;

    if ((FSP_SUCCESS != g_ioport.p_api->pinRead(g_ioport.p_ctrl, pin_a, &level_a)) ||
        (FSP_SUCCESS != g_ioport.p_api->pinRead(g_ioport.p_ctrl, pin_b, &level_b))) {
        return;
    }

    UB const current_ab = (UB) (((UB) level_a << 1U) | (UB) level_b);
    UB * p_previous_ab = left ? &g_left_encoder_previous_ab : &g_right_encoder_previous_ab;
    UB const transition = (UB) (((*p_previous_ab) << 2U) | current_ab);
    B const forward_sign = left ? WHEEL_ENCODER_LEFT_FORWARD_SIGN : WHEEL_ENCODER_RIGHT_FORWARD_SIGN;
    W const delta = (W) encoder_transition_delta[transition] * forward_sign;
    if (left) {
        g_encoder_left_count += delta;
    } else {
        g_encoder_right_count += delta;
    }
    *p_previous_ab = current_ab;
}

/** =================================================================*
 * @brief  エンコーダカウントから回転数を算出
 * @param[in] count 現在の累積カウント
 * @param[in,out] p_previous_count 前回速度算出時の累積カウント
 * @param[out] p_rpm_x10 10倍RPM出力
 * @param[in] elapsed_ms サンプル経過時間（単位: ms）
 * ================================================================= */
LOCAL void encoder_speed_update(W count, W * p_previous_count, volatile W * p_rpm_x10,
                                 UW elapsed_ms) {
    W const delta = count - *p_previous_count;
    D const numerator = (D) delta * 600000LL;
    D const denominator = (D) WHEEL_ENCODER_COUNTS_PER_REV * elapsed_ms;
    *p_rpm_x10 = (W) (numerator / denominator);
    *p_previous_count = count;
}

/** =================================================================*
 * @brief  エンコーダ初期化
 * @return FSPエラーコード
 * ================================================================= */
EXPORT fsp_err_t encoder_init(void) {
    g_encoder_left_count = 0;
    g_encoder_left_rpm_x10 = 0;
    g_encoder_right_count = 0;
    g_encoder_right_rpm_x10 = 0;

    bsp_io_level_t left_a = BSP_IO_LEVEL_LOW;
    bsp_io_level_t left_b = BSP_IO_LEVEL_LOW;
    bsp_io_level_t right_a = BSP_IO_LEVEL_LOW;
    bsp_io_level_t right_b = BSP_IO_LEVEL_LOW;
    fsp_err_t err = g_ioport.p_api->pinRead(g_ioport.p_ctrl, ENCODER_LEFT_A_PIN, &left_a);
    if (FSP_SUCCESS == err) {
        err = g_ioport.p_api->pinRead(g_ioport.p_ctrl, ENCODER_LEFT_B_PIN, &left_b);
    }
    if (FSP_SUCCESS == err) {
        err = g_ioport.p_api->pinRead(g_ioport.p_ctrl, ENCODER_RIGHT_A_PIN, &right_a);
    }
    if (FSP_SUCCESS == err) {
        err = g_ioport.p_api->pinRead(g_ioport.p_ctrl, ENCODER_RIGHT_B_PIN, &right_b);
    }
    if (FSP_SUCCESS != err) {
        return err;
    }

    g_left_encoder_previous_ab = (UB) (((UB) left_a << 1U) | (UB) left_b);
    g_right_encoder_previous_ab = (UB) (((UB) right_a << 1U) | (UB) right_b);
    g_speed_elapsed_ms = 0U;
    g_left_speed_previous_count = 0;
    g_right_speed_previous_count = 0;

    err = g_encoder_left_a_irq.p_api->open(g_encoder_left_a_irq.p_ctrl, g_encoder_left_a_irq.p_cfg);
    if (FSP_SUCCESS == err) {
        err = g_encoder_left_a_irq.p_api->enable(g_encoder_left_a_irq.p_ctrl);
    }
    if (FSP_SUCCESS == err) {
        err = g_encoder_left_b_irq.p_api->open(g_encoder_left_b_irq.p_ctrl, g_encoder_left_b_irq.p_cfg);
    }
    if (FSP_SUCCESS == err) {
        err = g_encoder_left_b_irq.p_api->enable(g_encoder_left_b_irq.p_ctrl);
    }
    if (FSP_SUCCESS == err) {
        err = g_encoder_right_a_irq.p_api->open(g_encoder_right_a_irq.p_ctrl, g_encoder_right_a_irq.p_cfg);
    }
    if (FSP_SUCCESS == err) {
        err = g_encoder_right_a_irq.p_api->enable(g_encoder_right_a_irq.p_ctrl);
    }
    if (FSP_SUCCESS == err) {
        err = g_encoder_right_b_irq.p_api->open(g_encoder_right_b_irq.p_ctrl, g_encoder_right_b_irq.p_cfg);
    }
    if (FSP_SUCCESS == err) {
        err = g_encoder_right_b_irq.p_api->enable(g_encoder_right_b_irq.p_ctrl);
    }

    return err;
}

/** =================================================================*
 * @brief  エンコーダを1 ms周期で保守
 * ================================================================= */
EXPORT void encoder_housekeeping_1ms(void) {
    g_speed_elapsed_ms++;
    if (g_speed_elapsed_ms >= WHEEL_SPEED_SAMPLE_PERIOD_MS) {
        encoder_speed_update(g_encoder_left_count, &g_left_speed_previous_count, &g_encoder_left_rpm_x10,
                             g_speed_elapsed_ms);
        encoder_speed_update(g_encoder_right_count, &g_right_speed_previous_count, &g_encoder_right_rpm_x10,
                             g_speed_elapsed_ms);
        g_speed_elapsed_ms = 0U;
    }
}

/** =================================================================*
 * @brief  10倍RPMをint16_tへ変換
 * @param[in] p_rpm_x10 10倍RPM値
 * @return 回転数（単位: RPM）
 * ================================================================= */
LOCAL H encoder_rpm_from_x10(volatile W const * p_rpm_x10) {
    W rpm = *p_rpm_x10 / 10;
    if (rpm > INT16_MAX) {
        rpm = INT16_MAX;
    } else if (rpm < INT16_MIN) {
        rpm = INT16_MIN;
    }
    return (H) rpm;
}

/** =================================================================*
 * @brief  左代表モーター累積カウント取得
 * @return 累積カウント
 * ================================================================= */
EXPORT W encoder_left_count_get(void) {
    return g_encoder_left_count;
}

/** =================================================================*
 * @brief  右代表モーター累積カウント取得
 * @return 累積カウント
 * ================================================================= */
EXPORT W encoder_right_count_get(void) {
    return g_encoder_right_count;
}

/** =================================================================*
 * @brief  左代表モーター回転数取得
 * @return 回転数（単位: RPM）
 * ================================================================= */
EXPORT H encoder_left_rpm_get(void) {
    return encoder_rpm_from_x10(&g_encoder_left_rpm_x10);
}

/** =================================================================*
 * @brief  右代表モーター回転数取得
 * @return 回転数（単位: RPM）
 * ================================================================= */
EXPORT H encoder_right_rpm_get(void) {
    return encoder_rpm_from_x10(&g_encoder_right_rpm_x10);
}

/** =================================================================*
 * @brief  左A相割込み処理
 * @param[in] p_args FSP外部IRQコールバック情報
 * ================================================================= */
EXPORT void encoder_left_a_irq_callback(external_irq_callback_args_t * p_args) {
    FSP_PARAMETER_NOT_USED(p_args);
    encoder_update(TRUE);
}

/** =================================================================*
 * @brief  左B相割込み処理
 * @param[in] p_args FSP外部IRQコールバック情報
 * ================================================================= */
EXPORT void encoder_left_b_irq_callback(external_irq_callback_args_t * p_args) {
    FSP_PARAMETER_NOT_USED(p_args);
    encoder_update(TRUE);
}

/** =================================================================*
 * @brief  右A相割込み処理
 * @param[in] p_args FSP外部IRQコールバック情報
 * ================================================================= */
EXPORT void encoder_right_a_irq_callback(external_irq_callback_args_t * p_args) {
    FSP_PARAMETER_NOT_USED(p_args);
    encoder_update(FALSE);
}

/** =================================================================*
 * @brief  右B相割込み処理
 * @param[in] p_args FSP外部IRQコールバック情報
 * ================================================================= */
EXPORT void encoder_right_b_irq_callback(external_irq_callback_args_t * p_args) {
    FSP_PARAMETER_NOT_USED(p_args);
    encoder_update(FALSE);
}
