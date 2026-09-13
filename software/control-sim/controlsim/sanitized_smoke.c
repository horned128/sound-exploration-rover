#include "control/obstacle_avoidance_controller.h"

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
    obstacle_avoidance_controller_step(&snapshot, FALSE, &avoidance_output);
    return 0;
}
