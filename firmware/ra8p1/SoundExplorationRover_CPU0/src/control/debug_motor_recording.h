/** =================================================================*
 * @file   debug_motor_recording.h
 * @brief  現場収録専用SW短押し・期限付き走行の純粋な状態機械
 * @details 駆動許可と周囲距離は呼出側で毎周期検査する。CPU1/安全調停は変更しない。
 * ================================================================= */
#ifndef SEROV_CPU0_DEBUG_MOTOR_RECORDING_H
#define SEROV_CPU0_DEBUG_MOTOR_RECORDING_H

#include <tk/tkernel.h>                                     /* μT-Kernel基本型とインライン定義 */

#define DEBUG_MOTOR_SHORT_PRESS_MIN_MS     (100U)           /**< 短押しとして扱う最小時間[ms] */
#define DEBUG_MOTOR_LONG_PRESS_MS          (2000U)          /**< 長押し停止を発火する時間[ms] */
#define DEBUG_MOTOR_RUN_LIMIT_MS           (6000U)          /**< 収録走行の最大継続時間[ms] */

/**< 現場収録スイッチから認識する操作 */
typedef enum e_debug_motor_button {
    DEBUG_MOTOR_BUTTON_NONE = 0,                            /**< 新しい操作なし */
    DEBUG_MOTOR_BUTTON_SHORT,                               /**< 短押し操作 */
    DEBUG_MOTOR_BUTTON_LONG,                                /**< 長押し操作 */
} debug_motor_button_t;

/**< 期限付き収録走行とスイッチ押下の状態 */
typedef struct st_debug_motor_recording {
    UW pressed_ms;                                          /**< 現在の押下継続時間[ms] */
    UW remaining_ms;                                        /**< 自動停止までの残り時間[ms] */
    BOOL long_handled;                                      /**< 現在の長押しを処理済み */
    BOOL press_started_during_drive;                        /**< 走行中に始まった押下 */
    BOOL active;                                            /**< 期限付き収録走行中 */
} debug_motor_recording_t;

/** =================================================================*
 * @brief 収録走行に必要な周囲クリアランス判定
 * @param[in] left_mm 左側距離[mm]
 * @param[in] center_mm 正面距離[mm]
 * @param[in] right_mm 右側距離[mm]
 * @param[in] side_min_mm 左右に必要な最小距離[mm]
 * @param[in] front_min_mm 正面に必要な最小距離[mm]
 * @return すべての距離条件を満たす場合TRUE
 * ================================================================= */
Inline BOOL debug_motor_recording_clearance(UH left_mm, UH center_mm, UH right_mm,
                                            UH side_min_mm, UH front_min_mm) {
    return (left_mm >= side_min_mm) && (center_mm >= front_min_mm) && (right_mm >= side_min_mm);
}

/** =================================================================*
 * @brief スイッチ押下状態から短押し・長押しイベントを判定
 * @param[in,out] p_state 押下時間と長押し処理状態
 * @param[in] pressed 現在のスイッチ押下状態
 * @param[in] elapsed_ms 前回判定からの経過時間[ms]
 * @return 今回発生した操作
 * @details 長押しは閾値到達時に一度だけ発火し、短押しは解放時に発火する。
 * ================================================================= */
Inline debug_motor_button_t debug_motor_button_step(debug_motor_recording_t * p_state,
                                                    BOOL pressed, UW elapsed_ms) {
    if (p_state == NULL) {
        return DEBUG_MOTOR_BUTTON_NONE;
    }
    if (pressed) {
        if (p_state->pressed_ms == 0U && !p_state->long_handled) {
            p_state->press_started_during_drive = p_state->active;
        }
        if (!p_state->long_handled) {
            if (p_state->pressed_ms < DEBUG_MOTOR_LONG_PRESS_MS) {
                UW const room = DEBUG_MOTOR_LONG_PRESS_MS - p_state->pressed_ms;
                p_state->pressed_ms += (elapsed_ms < room) ? elapsed_ms : room;
            }
            if (p_state->pressed_ms >= DEBUG_MOTOR_LONG_PRESS_MS) {
                p_state->long_handled = TRUE;
                return DEBUG_MOTOR_BUTTON_LONG;
            }
        }
        return DEBUG_MOTOR_BUTTON_NONE;
    }
    debug_motor_button_t const event = (!p_state->long_handled &&
        (p_state->pressed_ms >= DEBUG_MOTOR_SHORT_PRESS_MIN_MS)) ? DEBUG_MOTOR_BUTTON_SHORT :
        DEBUG_MOTOR_BUTTON_NONE;
    p_state->pressed_ms = 0U;
    p_state->long_handled = FALSE;
    p_state->press_started_during_drive = FALSE;
    return event;
}

/** =================================================================*
 * @brief 収録走行の開始・継続・停止を更新
 * @param[in,out] p_state 収録走行状態
 * @param[in] button 今回発生したスイッチ操作
 * @param[in] allowed 距離・安全条件による走行許可
 * @param[in] elapsed_ms 前回更新からの経過時間[ms]
 * @details 異常・距離不足・期限切れでは停止し、再押下なしに自動再開しない。
 * ================================================================= */
Inline void debug_motor_recording_step(debug_motor_recording_t * p_state,
                                       debug_motor_button_t button, BOOL allowed, UW elapsed_ms) {
    if (p_state == NULL) {
        return;
    }
    if ((button == DEBUG_MOTOR_BUTTON_LONG) || !allowed) {
        p_state->active = FALSE;
        p_state->remaining_ms = 0U;
        return;
    }
    if (button == DEBUG_MOTOR_BUTTON_SHORT) {
        if (p_state->active) {
            p_state->active = FALSE;
            p_state->remaining_ms = 0U;
        } else {
            p_state->active = TRUE;
            p_state->remaining_ms = DEBUG_MOTOR_RUN_LIMIT_MS;
        }
        return;
    }
    if (p_state->active) {
        if (p_state->remaining_ms <= elapsed_ms) {
            p_state->active = FALSE;
            p_state->remaining_ms = 0U;
        } else {
            p_state->remaining_ms -= elapsed_ms;
        }
    }
}

#endif                                                      /* SEROV_CPU0_DEBUG_MOTOR_RECORDING_H */
