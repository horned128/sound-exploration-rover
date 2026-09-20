#include "control/obstacle_avoidance_controller.h"
#include "control/smooth_avoidance_planner.h"
#include "control/safety_arbiter.h"
#include <assert.h>

int main(void)
{
    sound_follow_input_t sound_input = {0};
    sound_follow_output_t sound_output = {0};
    sensor_snapshot_t snapshot = {0};
    obstacle_avoidance_output_t avoidance_output = {0};

    sound_input.link_ready = TRUE;
    sound_input.motion_allowed = TRUE;
    sound_follow_controller_init();
    sound_follow_controller_step(&sound_input, 500U, &sound_output);

    snapshot.initialized = TRUE;
    snapshot.valid_flags = CPU0_SENSOR_VALID_ALL;
    snapshot.tof_distance_mm[CPU0_TOF_LEFT] = 1000U;
    snapshot.tof_distance_mm[CPU0_TOF_CENTER] = 1000U;
    snapshot.tof_distance_mm[CPU0_TOF_RIGHT] = 1000U;
    obstacle_avoidance_controller_step(&snapshot, FALSE, 0U, 0, &avoidance_output);
    /* Exercise timed recovery and signed gyro arithmetic under ASan/UBSan. */
    obstacle_avoidance_controller_init();
    for (UW now_ms = 0U; now_ms <= 4000U; now_ms += 100U) {
        snapshot.update_count++;
        snapshot.tof_distance_mm[CPU0_TOF_CENTER] = 430U;
        obstacle_avoidance_controller_step(&snapshot, FALSE, now_ms, 0, &avoidance_output);
    }
    assert(CPU0_SENSOR_RULE_BLOCKED_STOP == avoidance_output.rule);
    obstacle_avoidance_controller_init();
    for (UW now_ms = 0U; now_ms <= 8000U; now_ms += 100U) {
        snapshot.update_count++;
        snapshot.tof_distance_mm[CPU0_TOF_CENTER] = (now_ms == 0U) ? 430U : 1500U;
        snapshot.tof_distance_mm[CPU0_TOF_LEFT] = 1500U;
        snapshot.tof_distance_mm[CPU0_TOF_RIGHT] = 1500U;
        snapshot.gyro_dps_x10[2] = -200;
        obstacle_avoidance_controller_step(&snapshot, FALSE, now_ms, 0, &avoidance_output);
    }
    assert(CPU0_SENSOR_RULE_FORWARD == avoidance_output.rule);
    snapshot.gyro_dps_x10[2] = INT16_MIN;
    obstacle_avoidance_controller_step(&snapshot, FALSE, 8100U, 0, &avoidance_output);
    assert(CPU0_SENSOR_RULE_IMU_STOP == avoidance_output.rule);

    /* Exercise smooth_avoidance_plan under ASan/UBSan */
    smooth_avoidance_input_t planner_input = {
        .tof_distance_mm = {1000.0f, 1000.0f, 1000.0f},
        .tof_valid = {TRUE, TRUE, TRUE},
        .target_heading_deg = 30.0f,
        .current_steering_deg = 0.0f,
        .current_speed_scale = 0.5f,
        .dt_sec = 0.05f,
    };
    smooth_avoidance_output_t planner_output = {0};
    smooth_avoidance_plan(&planner_input, &planner_output);
    assert(!planner_output.is_blocked);
    assert(planner_output.speed_scale > 0.0f);

    /* Exercise hard stop under ASan/UBSan */
    planner_input.tof_distance_mm[1] = 200.0f;
    smooth_avoidance_plan(&planner_input, &planner_output);
    assert(planner_output.is_blocked);
    assert(planner_output.speed_scale == 0.0f);

    /* Exercise safety_arbiter under ASan/UBSan */
    safety_motion_command_t req = {
        .steering_deg = 20,
        .left_rpm = 100,
        .right_rpm = 100,
        .actuator_enable = TRUE,
        .emergency_stop = FALSE,
    };
    safety_motion_command_t arb = {0};
    snapshot.tof_distance_mm[CPU0_TOF_CENTER] = 1000U;
    snapshot.gyro_dps_x10[2] = 0;
    safety_arbiter_arbitrate(&req, &snapshot, TRUE, &arb);
    assert(arb.actuator_enable);
    assert(arb.left_rpm == 100);

    /* Exercise veto when front obstacle < 250mm */
    snapshot.tof_distance_mm[CPU0_TOF_CENTER] = 200U;
    safety_arbiter_arbitrate(&req, &snapshot, TRUE, &arb);
    assert(!arb.actuator_enable);
    assert(arb.left_rpm == 0);
    assert(!arb.emergency_stop);

    return 0;
}
