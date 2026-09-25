/** SW1 press semantics, bounded runtime and fail-closed gating on host. */
#include "control/debug_motor_recording.h"
#include <assert.h>

static debug_motor_button_t short_press(debug_motor_recording_t * p_state) {
    assert(DEBUG_MOTOR_BUTTON_NONE == debug_motor_button_step(p_state, true, 50U));
    assert(DEBUG_MOTOR_BUTTON_NONE == debug_motor_button_step(p_state, true, 50U));
    return debug_motor_button_step(p_state, false, 50U);
}

int main(void) {
    debug_motor_recording_t state = {0};
    assert(debug_motor_recording_clearance(380, 550, 380, 380, 550));
    assert(!debug_motor_recording_clearance(379, 550, 380, 380, 550));
    assert(!debug_motor_recording_clearance(380, 549, 380, 380, 550));
    assert(!debug_motor_recording_clearance(380, 550, 379, 380, 550));
    assert(DEBUG_MOTOR_BUTTON_NONE == debug_motor_button_step(&state, true, 50U));
    assert(DEBUG_MOTOR_BUTTON_NONE == debug_motor_button_step(&state, false, 50U));
    assert(DEBUG_MOTOR_BUTTON_SHORT == short_press(&state));
    debug_motor_recording_step(&state, DEBUG_MOTOR_BUTTON_SHORT, false, 50U);
    assert(!state.active);

    debug_motor_recording_step(&state, short_press(&state), true, 50U);
    assert(state.active && state.remaining_ms == DEBUG_MOTOR_RUN_LIMIT_MS);
    debug_motor_recording_step(&state, DEBUG_MOTOR_BUTTON_NONE, true, 1000U);
    assert(state.remaining_ms == DEBUG_MOTOR_RUN_LIMIT_MS - 1000U);
    debug_motor_recording_step(&state, short_press(&state), true, 50U);
    assert(!state.active && state.remaining_ms == 0U);

    debug_motor_recording_step(&state, short_press(&state), true, 50U);
    debug_motor_recording_step(&state, DEBUG_MOTOR_BUTTON_NONE, true, DEBUG_MOTOR_RUN_LIMIT_MS);
    assert(!state.active && state.remaining_ms == 0U);
    debug_motor_recording_step(&state, DEBUG_MOTOR_BUTTON_NONE, true, 50U);
    assert(!state.active); /* No automatic restart after timeout. */

    debug_motor_recording_step(&state, short_press(&state), true, 50U);
    debug_motor_recording_step(&state, DEBUG_MOTOR_BUTTON_NONE, false, 50U);
    assert(!state.active); /* Loss of clearance or sensor freshness stops immediately. */

    debug_motor_recording_step(&state, short_press(&state), true, 50U);
    debug_motor_button_t event = DEBUG_MOTOR_BUTTON_NONE;
    for (unsigned int i = 0; i < 40; i++) {
        event = debug_motor_button_step(&state, true, 50U);
        if (i == 10U) {
            debug_motor_recording_step(&state, DEBUG_MOTOR_BUTTON_NONE, false, 50U);
            assert(!state.active); /* Safety veto before the 2-second long press. */
        }
    }
    assert(event == DEBUG_MOTOR_BUTTON_LONG);
    assert(state.press_started_during_drive); /* Must not clear MRAM by starting learning. */
    debug_motor_recording_step(&state, event, true, 50U);
    assert(!state.active);
    assert(DEBUG_MOTOR_BUTTON_NONE == debug_motor_button_step(&state, true, 50U));
    assert(DEBUG_MOTOR_BUTTON_NONE == debug_motor_button_step(&state, false, 50U));
    return 0;
}
