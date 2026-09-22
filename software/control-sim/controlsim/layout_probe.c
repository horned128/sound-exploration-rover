#include <stddef.h>
#include <stdio.h>

#include "control/obstacle_avoidance_controller.h"
#include "services/acoustic_identifier.h"
#include "services/sound_source_localizer.h"

int main(void)
{
    (void) printf(
        "{"
        "\"sound_follow_input_t\":{\"size\":%zu,\"link_ready\":%zu,\"new_observation\":%zu,"
        "\"fault_active\":%zu,\"motion_allowed\":%zu,\"observation\":%zu,"
        "\"match_required\":%zu,\"target_sound_matched\":%zu,\"navigation_target_valid\":%zu,"
        "\"navigation_bearing_deg\":%zu,\"arrival_verify\":%zu,\"arrived\":%zu,\"avoidance_relisten\":%zu,"
        "\"restart_request\":%zu,\"imu_valid\":%zu,\"gyro_z_dps_x10\":%zu,\"imu_update_count\":%zu},"
        "\"sound_follow_output_t\":{\"size\":%zu,\"state\":%zu,\"steering_deg\":%zu,"
        "\"target_bearing_deg\":%zu,\"is_spin_turn\":%zu,\"left_rpm\":%zu,\"right_rpm\":%zu,\"actuator_enable\":%zu,"
        "\"emergency_stop\":%zu},"
        "\"acoustic_observation_t\":{\"size\":%zu,\"doa_deg\":%zu,\"raw_doa_deg\":%zu,"
        "\"level_dbfs_x100\":%zu,\"peak_dbfs_x100\":%zu,\"vad\":%zu,\"doa_confidence\":%zu,"
        "\"xvf_status\":%zu,\"audio_flags\":%zu,\"xvf_raw_status\":%zu,\"reserved\":%zu,"
        "\"audio_frame_count\":%zu,\"sample_sequence\":%zu},"
        "\"sensor_diagnostics_t\":{\"size\":%zu,\"tof_range_status\":%zu,\"tof_result\":%zu,"
        "\"failure_kind\":%zu,\"failure_device\":%zu,\"failure_stage\":%zu,\"failure_channel\":%zu},"
        "\"sensor_snapshot_t\":{\"size\":%zu,\"tof_distance_mm\":%zu,\"accel_mg\":%zu,"
        "\"gyro_dps_x10\":%zu,\"age_ms\":%zu,\"update_count\":%zu,\"error_flags\":%zu,"
        "\"last_error\":%zu,\"valid_flags\":%zu,\"initialized\":%zu,\"diagnostics\":%zu},"
        "\"obstacle_avoidance_output_t\":{\"size\":%zu,\"state\":%zu,\"rule\":%zu,"
        "\"steering_deg\":%zu,\"is_spin_turn\":%zu,\"left_rpm\":%zu,\"right_rpm\":%zu,\"actuator_enable\":%zu,"
        "\"emergency_stop\":%zu,\"avoidance_in_progress\":%zu,\"avoidance_completed\":%zu}}\n",
        sizeof(sound_follow_input_t),
        offsetof(sound_follow_input_t, link_ready),
        offsetof(sound_follow_input_t, new_observation), offsetof(sound_follow_input_t, fault_active),
        offsetof(sound_follow_input_t, motion_allowed), offsetof(sound_follow_input_t, observation),
        offsetof(sound_follow_input_t, match_required), offsetof(sound_follow_input_t, target_sound_matched),
        offsetof(sound_follow_input_t, navigation_target_valid),
        offsetof(sound_follow_input_t, navigation_bearing_deg), offsetof(sound_follow_input_t, arrival_verify),
        offsetof(sound_follow_input_t, arrived), offsetof(sound_follow_input_t, avoidance_relisten),
        offsetof(sound_follow_input_t, restart_request), offsetof(sound_follow_input_t, imu_valid),
        offsetof(sound_follow_input_t, gyro_z_dps_x10),
        offsetof(sound_follow_input_t, imu_update_count),
        sizeof(sound_follow_output_t), offsetof(sound_follow_output_t, state),
        offsetof(sound_follow_output_t, steering_deg), offsetof(sound_follow_output_t, target_bearing_deg),
        offsetof(sound_follow_output_t, is_spin_turn),
        offsetof(sound_follow_output_t, left_rpm), offsetof(sound_follow_output_t, right_rpm),
        offsetof(sound_follow_output_t, actuator_enable),
        offsetof(sound_follow_output_t, emergency_stop), sizeof(acoustic_observation_t),
        offsetof(acoustic_observation_t, doa_deg), offsetof(acoustic_observation_t, raw_doa_deg),
        offsetof(acoustic_observation_t, level_dbfs_x100),
        offsetof(acoustic_observation_t, peak_dbfs_x100), offsetof(acoustic_observation_t, vad),
        offsetof(acoustic_observation_t, doa_confidence),
        offsetof(acoustic_observation_t, xvf_status), offsetof(acoustic_observation_t, audio_flags),
        offsetof(acoustic_observation_t, xvf_raw_status), offsetof(acoustic_observation_t, reserved),
        offsetof(acoustic_observation_t, audio_frame_count), offsetof(acoustic_observation_t, sample_sequence),
        sizeof(sensor_diagnostics_t), offsetof(sensor_diagnostics_t, tof_range_status),
        offsetof(sensor_diagnostics_t, tof_result), offsetof(sensor_diagnostics_t, failure_kind),
        offsetof(sensor_diagnostics_t, failure_device), offsetof(sensor_diagnostics_t, failure_stage),
        offsetof(sensor_diagnostics_t, failure_channel), sizeof(sensor_snapshot_t),
        offsetof(sensor_snapshot_t, tof_distance_mm), offsetof(sensor_snapshot_t, accel_mg),
        offsetof(sensor_snapshot_t, gyro_dps_x10), offsetof(sensor_snapshot_t, age_ms),
        offsetof(sensor_snapshot_t, update_count), offsetof(sensor_snapshot_t, error_flags),
        offsetof(sensor_snapshot_t, last_error), offsetof(sensor_snapshot_t, valid_flags),
        offsetof(sensor_snapshot_t, initialized), offsetof(sensor_snapshot_t, diagnostics),
        sizeof(obstacle_avoidance_output_t), offsetof(obstacle_avoidance_output_t, state),
        offsetof(obstacle_avoidance_output_t, rule), offsetof(obstacle_avoidance_output_t, steering_deg),
        offsetof(obstacle_avoidance_output_t, is_spin_turn),
        offsetof(obstacle_avoidance_output_t, left_rpm), offsetof(obstacle_avoidance_output_t, right_rpm),
        offsetof(obstacle_avoidance_output_t, actuator_enable), offsetof(obstacle_avoidance_output_t, emergency_stop),
        offsetof(obstacle_avoidance_output_t, avoidance_in_progress),
        offsetof(obstacle_avoidance_output_t, avoidance_completed));
    return 0;
}
