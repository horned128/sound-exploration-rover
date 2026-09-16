#include "control/obstacle_avoidance_controller.h"
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
    obstacle_avoidance_controller_step(&snapshot, FALSE, 0U, &avoidance_output);
    /* Exercise timed recovery and signed gyro arithmetic under ASan/UBSan. */
    obstacle_avoidance_controller_init();
    for (UW now_ms = 0U; now_ms <= 4000U; now_ms += 100U) {
        snapshot.update_count++;
        snapshot.tof_distance_mm[CPU0_TOF_CENTER] = 430U;
        obstacle_avoidance_controller_step(&snapshot, FALSE, now_ms, &avoidance_output);
    }
    assert(CPU0_SENSOR_RULE_BLOCKED_STOP == avoidance_output.rule);
    obstacle_avoidance_controller_init();
    for (UW now_ms = 0U; now_ms <= 8000U; now_ms += 100U) {
        snapshot.update_count++;
        snapshot.tof_distance_mm[CPU0_TOF_CENTER] = (now_ms == 0U) ? 430U : 1500U;
        snapshot.tof_distance_mm[CPU0_TOF_LEFT] = 1500U;
        snapshot.tof_distance_mm[CPU0_TOF_RIGHT] = 1500U;
        snapshot.gyro_dps_x10[2] = -200;
        obstacle_avoidance_controller_step(&snapshot, FALSE, now_ms, &avoidance_output);
    }
    assert(CPU0_SENSOR_RULE_FORWARD == avoidance_output.rule);
    snapshot.gyro_dps_x10[2] = INT16_MIN;
    obstacle_avoidance_controller_step(&snapshot, FALSE, 8100U, &avoidance_output);
    assert(CPU0_SENSOR_RULE_IMU_STOP == avoidance_output.rule);
    return 0;
}
