/** =================================================================*
 * @file   debug_motor_recording.h
 * @brief  現場収録専用SW短押し・期限付き走行の純粋な状態機械
 * @details 駆動許可と周囲距離は呼出側で毎周期検査する。CPU1/安全調停は変更しない。
 * ================================================================= */
#ifndef SEROV_CPU0_DEBUG_MOTOR_RECORDING_H
#define SEROV_CPU0_DEBUG_MOTOR_RECORDING_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define DEBUG_MOTOR_SHORT_PRESS_MIN_MS (100U)
#define DEBUG_MOTOR_LONG_PRESS_MS (2000U)
#define DEBUG_MOTOR_RUN_LIMIT_MS (6000U)

typedef enum e_debug_motor_button {
    DEBUG_MOTOR_BUTTON_NONE = 0,
    DEBUG_MOTOR_BUTTON_SHORT,
    DEBUG_MOTOR_BUTTON_LONG,
} debug_motor_button_t;

typedef struct st_debug_motor_recording {
    uint32_t pressed_ms;
    uint32_t remaining_ms;
    bool long_handled;
    bool press_started_during_drive;
    bool active;
} debug_motor_recording_t;

static inline bool debug_motor_recording_clearance(uint16_t left_mm, uint16_t center_mm,
                                                    uint16_t right_mm, uint16_t side_min_mm,
                                                    uint16_t front_min_mm) {
    return (left_mm >= side_min_mm) && (center_mm >= front_min_mm) && (right_mm >= side_min_mm);
}

/* 長押しは閾値到達時に一度だけ発火、短押しは離した瞬間だけ発火する。 */
static inline debug_motor_button_t debug_motor_button_step(debug_motor_recording_t * p_state,
                                                              bool pressed, uint32_t elapsed_ms) {
    if (p_state == NULL) {
        return DEBUG_MOTOR_BUTTON_NONE;
    }
    if (pressed) {
        if (p_state->pressed_ms == 0U && !p_state->long_handled) {
            p_state->press_started_during_drive = p_state->active;
        }
        if (!p_state->long_handled) {
            if (p_state->pressed_ms < DEBUG_MOTOR_LONG_PRESS_MS) {
                uint32_t const room = DEBUG_MOTOR_LONG_PRESS_MS - p_state->pressed_ms;
                p_state->pressed_ms += (elapsed_ms < room) ? elapsed_ms : room;
            }
            if (p_state->pressed_ms >= DEBUG_MOTOR_LONG_PRESS_MS) {
                p_state->long_handled = true;
                return DEBUG_MOTOR_BUTTON_LONG;
            }
        }
        return DEBUG_MOTOR_BUTTON_NONE;
    }
    debug_motor_button_t const event = (!p_state->long_handled &&
        (p_state->pressed_ms >= DEBUG_MOTOR_SHORT_PRESS_MIN_MS)) ? DEBUG_MOTOR_BUTTON_SHORT :
        DEBUG_MOTOR_BUTTON_NONE;
    p_state->pressed_ms = 0U;
    p_state->long_handled = false;
    p_state->press_started_during_drive = false;
    return event;
}

/* 短押しでstart/stop。異常・距離不足・期限切れは即時停止。再押下なしに自動再開しない。 */
static inline void debug_motor_recording_step(debug_motor_recording_t * p_state,
                                               debug_motor_button_t button, bool allowed, uint32_t elapsed_ms) {
    if (p_state == NULL) {
        return;
    }
    if ((button == DEBUG_MOTOR_BUTTON_LONG) || !allowed) {
        p_state->active = false;
        p_state->remaining_ms = 0U;
        return;
    }
    if (button == DEBUG_MOTOR_BUTTON_SHORT) {
        if (p_state->active) {
            p_state->active = false;
            p_state->remaining_ms = 0U;
        } else {
            p_state->active = true;
            p_state->remaining_ms = DEBUG_MOTOR_RUN_LIMIT_MS;
        }
        return;
    }
    if (p_state->active) {
        if (p_state->remaining_ms <= elapsed_ms) {
            p_state->active = false;
            p_state->remaining_ms = 0U;
        } else {
            p_state->remaining_ms -= elapsed_ms;
        }
    }
}

#endif /* SEROV_CPU0_DEBUG_MOTOR_RECORDING_H */
