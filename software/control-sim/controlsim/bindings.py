"""ctypes bindings for the control code compiled directly from the firmware."""

from __future__ import annotations

import ctypes
import subprocess
import sys
from functools import lru_cache
from pathlib import Path
from typing import Iterable


CONTROL_SIM_ROOT = Path(__file__).resolve().parents[1]
BUILD_SCRIPT = CONTROL_SIM_ROOT / "build.py"

BOOL = ctypes.c_int32

ACOUSTIC_XVF_STATUS_READY = 1
CPU0_SENSOR_VALID_ALL = 0x0F
CPU0_ACOUSTIC_FEATURE_BIN_COUNT = 32
CPU0_ACOUSTIC_SUMMARY_DIMENSION = 192
CPU0_BACKGROUND_MODEL_INPUT_DIMENSION = 32
CPU0_BACKGROUND_MODEL_HIDDEN_DIMENSION = 16


class AcousticObservation(ctypes.Structure):
    _fields_ = [
        ("doa_deg", ctypes.c_uint16),
        ("raw_doa_deg", ctypes.c_uint16),
        ("level_dbfs_x100", ctypes.c_int16),
        ("peak_dbfs_x100", ctypes.c_int16),
        ("vad", ctypes.c_uint8),
        ("doa_confidence", ctypes.c_uint8),
        ("xvf_status", ctypes.c_uint8),
        ("audio_flags", ctypes.c_uint8),
        ("xvf_raw_status", ctypes.c_uint8),
        ("reserved", ctypes.c_uint8),
        ("audio_frame_count", ctypes.c_uint32),
        ("sample_sequence", ctypes.c_uint32),
    ]


class SoundFollowInput(ctypes.Structure):
    _fields_ = [
        ("link_ready", BOOL),
        ("new_observation", BOOL),
        ("fault_active", BOOL),
        ("motion_allowed", BOOL),
        ("observation", AcousticObservation),
        ("match_required", BOOL),
        ("target_sound_matched", BOOL),
        ("target_sound_direction_valid", BOOL),
        ("navigation_target_valid", BOOL),
        ("navigation_bearing_deg", ctypes.c_int16),
        ("arrival_verify", BOOL),
        ("arrived", BOOL),
        ("avoidance_relisten", BOOL),
        ("restart_request", BOOL),
        ("imu_valid", BOOL),
        ("gyro_z_dps_x10", ctypes.c_int16),
        ("imu_update_count", ctypes.c_uint32),
        ("pose_heading_valid", BOOL),
        ("pose_heading_mrad", ctypes.c_int32),
        ("rear_seam_turn_preference", ctypes.c_int8),
    ]


class SoundFollowOutput(ctypes.Structure):
    _fields_ = [
        ("state", ctypes.c_int32),
        ("steering_deg", ctypes.c_int16),
        ("target_bearing_deg", ctypes.c_int16),
        ("is_spin_turn", BOOL),
        ("left_rpm", ctypes.c_int16),
        ("right_rpm", ctypes.c_int16),
        ("actuator_enable", BOOL),
        ("emergency_stop", BOOL),
    ]


class SensorDiagnostics(ctypes.Structure):
    _fields_ = [
        ("tof_range_status", ctypes.c_uint8 * 3),
        ("tof_result", ctypes.c_uint8 * 3),
        ("failure_kind", ctypes.c_int32),
        ("failure_device", ctypes.c_int32),
        ("failure_stage", ctypes.c_int32),
        ("failure_channel", ctypes.c_int8),
    ]


class SensorSnapshot(ctypes.Structure):
    _fields_ = [
        ("tof_distance_mm", ctypes.c_uint16 * 3),
        ("accel_mg", ctypes.c_int16 * 3),
        ("gyro_dps_x10", ctypes.c_int16 * 3),
        ("age_ms", ctypes.c_uint32),
        ("update_count", ctypes.c_uint32),
        ("error_flags", ctypes.c_uint32),
        ("last_error", ctypes.c_int32),
        ("valid_flags", ctypes.c_uint8),
        ("initialized", BOOL),
        ("diagnostics", SensorDiagnostics),
    ]


class ObstacleAvoidanceOutput(ctypes.Structure):
    _fields_ = [
        ("state", ctypes.c_int32),
        ("rule", ctypes.c_int32),
        ("steering_deg", ctypes.c_int16),
        ("is_spin_turn", BOOL),
        ("left_rpm", ctypes.c_int16),
        ("right_rpm", ctypes.c_int16),
        ("actuator_enable", BOOL),
        ("emergency_stop", BOOL),
        ("avoidance_in_progress", BOOL),
        ("avoidance_completed", BOOL),
    ]


class AcousticIdentifierSummaryOutput(ctypes.Structure):
    _fields_ = [
        ("active_frame_count", ctypes.c_uint8),
        ("sample_index", ctypes.c_uint32),
        ("minimum_cosine_distance", ctypes.c_float),
        ("threshold", ctypes.c_float),
        ("status", ctypes.c_int32),
    ]


class BackgroundModelState(ctypes.Structure):
    _fields_ = [
        ("decoder", ctypes.c_float * (CPU0_BACKGROUND_MODEL_INPUT_DIMENSION * CPU0_BACKGROUND_MODEL_HIDDEN_DIMENSION)),
        ("inverse_correlation", ctypes.c_float * (CPU0_BACKGROUND_MODEL_HIDDEN_DIMENSION * CPU0_BACKGROUND_MODEL_HIDDEN_DIMENSION)),
        ("encoder_seed", ctypes.c_uint32),
        ("mse_count", ctypes.c_uint32),
        ("mse_mean", ctypes.c_float),
        ("mse_m2", ctypes.c_float),
    ]


class OdometryPose(ctypes.Structure):
    _fields_ = [
        ("x_mm", ctypes.c_int32),
        ("y_mm", ctypes.c_int32),
        ("theta_mrad", ctypes.c_int32),
        ("theta_deg_x10", ctypes.c_int16),
        ("total_distance_mm", ctypes.c_uint32),
        ("linear_speed_mm_s", ctypes.c_int16),
        ("angular_speed_mrad_s", ctypes.c_int16),
        ("left_encoder_total", ctypes.c_int32),
        ("right_encoder_total", ctypes.c_int32),
        ("timestamp_ms", ctypes.c_uint32),
        ("valid", BOOL),
    ]


class OdometryContext(ctypes.Structure):
    _fields_ = [
        ("x_mm", ctypes.c_float),
        ("y_mm", ctypes.c_float),
        ("theta_rad", ctypes.c_float),
        ("total_distance_mm", ctypes.c_float),
        ("linear_speed_mm_s", ctypes.c_float),
        ("angular_speed_rad_s", ctypes.c_float),
        ("gyro_bias_dps", ctypes.c_float),
        ("prev_left_count", ctypes.c_uint32),
        ("prev_right_count", ctypes.c_uint32),
        ("prev_uptime_ms", ctypes.c_uint32),
        ("left_unwrapped", ctypes.c_int32),
        ("right_unwrapped", ctypes.c_int32),
        ("initialized", BOOL),
        ("gyro_calibrated", BOOL),
    ]


class SoundSourceLocalizerInput(ctypes.Structure):
    _fields_ = [
        ("pose", OdometryPose),
        ("relative_doa_deg", ctypes.c_int16),
        ("doa_confidence", ctypes.c_uint8),
        ("new_observation", BOOL),
        ("sound_valid", BOOL),
        ("pose_valid", BOOL),
        ("observation_sequence", ctypes.c_uint32),
        ("now_ms", ctypes.c_uint32),
    ]


class SoundSourceLocalizerOutput(ctypes.Structure):
    _fields_ = [
        ("source_x_mm", ctypes.c_int32),
        ("source_y_mm", ctypes.c_int32),
        ("source_range_mm", ctypes.c_uint32),
        ("source_bearing_deg", ctypes.c_int16),
        ("source_confidence", ctypes.c_uint8),
        ("observation_count", ctypes.c_uint8),
        ("source_position_valid", BOOL),
        ("localization_geometry_valid", BOOL),
        ("navigation_target_valid", BOOL),
        ("arrival_candidate", BOOL),
        ("localization_residual_mm", ctypes.c_uint16),
        ("bearing_crossing_angle_deg", ctypes.c_uint16),
        ("baseline_mm", ctypes.c_uint16),
        ("source_position_shift_mm", ctypes.c_uint16),
        ("arrival_confirm_count", ctypes.c_uint8),
        ("arrival_state", ctypes.c_int32),
    ]


class SmoothAvoidanceInput(ctypes.Structure):
    _fields_ = [
        ("tof_distance_mm", ctypes.c_float * 3),
        ("tof_valid", BOOL * 3),
        ("target_heading_deg", ctypes.c_float),
        ("current_steering_deg", ctypes.c_float),
        ("current_speed_scale", ctypes.c_float),
        ("dt_sec", ctypes.c_float),
    ]


class SmoothAvoidanceOutput(ctypes.Structure):
    _fields_ = [
        ("steering_deg", ctypes.c_float),
        ("speed_scale", ctypes.c_float),
        ("is_blocked", BOOL),
        ("left_clearance_mm", ctypes.c_float),
        ("center_clearance_mm", ctypes.c_float),
        ("right_clearance_mm", ctypes.c_float),
    ]


class ControlMlpOutput(ctypes.Structure):
    _fields_ = [
        ("steering_deg", ctypes.c_float),
        ("speed_scale", ctypes.c_float),
        ("is_blocked", BOOL),
        ("emergency_stop", BOOL),
        ("fallback_required", BOOL),
    ]


class SafetyMotionCommand(ctypes.Structure):
    _fields_ = [
        ("steering_deg", ctypes.c_int16),
        ("left_rpm", ctypes.c_int16),
        ("right_rpm", ctypes.c_int16),
        ("actuator_enable", BOOL),
        ("emergency_stop", BOOL),
    ]


class SafetyArbiterStatus(ctypes.Structure):
    _fields_ = [
        ("sensor_fresh", BOOL),
        ("tof_usable", BOOL),
        ("motion_allowed", BOOL),
        ("hard_stop_veto", BOOL),
        ("imu_safe", BOOL),
    ]


@lru_cache(maxsize=1)
def library() -> ctypes.CDLL:
    completed = subprocess.run(
        [sys.executable, str(BUILD_SCRIPT)], check=True, capture_output=True, text=True
    )
    native_library = completed.stdout.strip()
    handle = ctypes.CDLL(native_library)
    handle.sound_follow_controller_init.argtypes = []
    handle.sound_follow_controller_init.restype = None
    handle.sound_follow_doa_to_relative.argtypes = [ctypes.c_uint16]
    handle.sound_follow_doa_to_relative.restype = ctypes.c_int16
    handle.sound_follow_controller_step.argtypes = [
        ctypes.POINTER(SoundFollowInput),
        ctypes.c_uint32,
        ctypes.POINTER(SoundFollowOutput),
    ]
    handle.sound_follow_controller_step.restype = None
    handle.obstacle_avoidance_controller_init.argtypes = []
    handle.obstacle_avoidance_controller_init.restype = None
    handle.obstacle_avoidance_encoder_feedback_set.argtypes = [
        BOOL, ctypes.c_uint32, ctypes.c_int16, ctypes.c_int16,
    ]
    handle.obstacle_avoidance_encoder_feedback_set.restype = None
    handle.obstacle_avoidance_spin_space_available.argtypes = [ctypes.POINTER(SensorSnapshot)]
    handle.obstacle_avoidance_spin_space_available.restype = BOOL
    handle.obstacle_avoidance_rear_seam_turn_preference.argtypes = [ctypes.POINTER(SensorSnapshot)]
    handle.obstacle_avoidance_rear_seam_turn_preference.restype = ctypes.c_int8
    handle.obstacle_avoidance_controller_step.argtypes = [
        ctypes.POINTER(SensorSnapshot),
        BOOL,
        ctypes.c_uint32,
        ctypes.c_int16,
        ctypes.c_int16,
        ctypes.POINTER(ObstacleAvoidanceOutput),
    ]
    handle.obstacle_avoidance_controller_step.restype = None
    handle.acoustic_identifier_frame_is_active.argtypes = [ctypes.POINTER(ctypes.c_int8)]
    handle.acoustic_identifier_frame_is_active.restype = BOOL
    handle.acoustic_identifier_summary_create.argtypes = [
        ctypes.POINTER(ctypes.c_int8), ctypes.c_uint32, ctypes.POINTER(ctypes.c_int8), ctypes.POINTER(ctypes.c_uint8),
    ]
    handle.acoustic_identifier_summary_create.restype = BOOL
    handle.acoustic_identifier_find_peak_bin.argtypes = [
        ctypes.POINTER(ctypes.c_int8), ctypes.c_uint32,
    ]
    handle.acoustic_identifier_find_peak_bin.restype = ctypes.c_uint8
    handle.acoustic_identifier_consensus_peak_bin.argtypes = [
        ctypes.POINTER(ctypes.c_int8), ctypes.c_uint32,
    ]
    handle.acoustic_identifier_consensus_peak_bin.restype = ctypes.c_uint8
    handle.acoustic_identifier_build_weights.argtypes = [
        ctypes.c_uint8, ctypes.POINTER(ctypes.c_float),
    ]
    handle.acoustic_identifier_build_weights.restype = None
    handle.acoustic_identifier_weighted_cosine_distance.argtypes = [
        ctypes.POINTER(ctypes.c_int8), ctypes.POINTER(ctypes.c_int8),
        ctypes.POINTER(ctypes.c_float), ctypes.POINTER(ctypes.c_float),
    ]
    handle.acoustic_identifier_weighted_cosine_distance.restype = BOOL
    handle.acoustic_identifier_cosine_distance.argtypes = [
        ctypes.POINTER(ctypes.c_int8), ctypes.POINTER(ctypes.c_int8), ctypes.POINTER(ctypes.c_float),
    ]
    handle.acoustic_identifier_cosine_distance.restype = BOOL
    handle.acoustic_identifier_isolated_sample_find.argtypes = [
        ctypes.POINTER(ctypes.c_int8), ctypes.c_uint32, ctypes.POINTER(ctypes.c_uint32),
    ]
    handle.acoustic_identifier_isolated_sample_find.restype = BOOL
    handle.acoustic_identifier_leave_one_out_threshold.argtypes = [
        ctypes.POINTER(ctypes.c_int8), ctypes.c_uint32,
        ctypes.POINTER(ctypes.c_float), ctypes.POINTER(ctypes.c_float),
    ]
    handle.acoustic_identifier_leave_one_out_threshold.restype = BOOL
    handle.acoustic_identifier_summary_classify.argtypes = [
        ctypes.POINTER(ctypes.c_int8), BOOL, ctypes.c_uint8, ctypes.POINTER(ctypes.c_int8), ctypes.c_uint32,
        ctypes.POINTER(ctypes.c_float), ctypes.c_float, ctypes.POINTER(AcousticIdentifierSummaryOutput),
    ]
    handle.acoustic_identifier_summary_classify.restype = None
    handle.background_model_init.argtypes = [ctypes.POINTER(BackgroundModelState), ctypes.c_uint32]
    handle.background_model_init.restype = None
    handle.background_model_mse.argtypes = [
        ctypes.POINTER(BackgroundModelState), ctypes.POINTER(ctypes.c_int8), ctypes.POINTER(ctypes.c_float),
    ]
    handle.background_model_mse.restype = BOOL
    handle.background_model_observe.argtypes = [
        ctypes.POINTER(BackgroundModelState), ctypes.POINTER(ctypes.c_int8), BOOL, ctypes.POINTER(ctypes.c_float),
    ]
    handle.background_model_observe.restype = BOOL
    handle.background_model_mse_threshold.argtypes = [ctypes.POINTER(BackgroundModelState), ctypes.POINTER(ctypes.c_float)]
    handle.background_model_mse_threshold.restype = BOOL
    handle.background_model_reset_inverse_correlation.argtypes = [ctypes.POINTER(BackgroundModelState)]
    handle.background_model_reset_inverse_correlation.restype = None
    handle.odometry_init.argtypes = [ctypes.POINTER(OdometryContext)]
    handle.odometry_init.restype = None
    handle.odometry_update.argtypes = [
        ctypes.POINTER(OdometryContext),
        ctypes.c_uint32,
        ctypes.c_uint32,
        ctypes.c_uint32,
        ctypes.c_int16,
        ctypes.c_int16,
        ctypes.c_int16,
    ]
    handle.odometry_update.restype = None
    handle.odometry_get_pose.argtypes = [
        ctypes.POINTER(OdometryContext),
        ctypes.POINTER(OdometryPose),
    ]
    handle.odometry_get_pose.restype = None
    handle.sound_source_localizer_init.argtypes = []
    handle.sound_source_localizer_init.restype = None
    handle.sound_source_localizer_step.argtypes = [
        ctypes.POINTER(SoundSourceLocalizerInput),
        ctypes.POINTER(SoundSourceLocalizerOutput),
    ]
    handle.sound_source_localizer_step.restype = None
    handle.smooth_avoidance_plan.argtypes = [
        ctypes.POINTER(SmoothAvoidanceInput),
        ctypes.POINTER(SmoothAvoidanceOutput),
    ]
    handle.smooth_avoidance_plan.restype = None
    handle.safety_arbiter_tof_usable.argtypes = [ctypes.POINTER(SensorSnapshot)]
    handle.safety_arbiter_tof_usable.restype = BOOL
    handle.safety_arbiter_motion_allowed.argtypes = [
        ctypes.POINTER(SensorSnapshot),
        BOOL,
        ctypes.POINTER(SafetyArbiterStatus),
    ]
    handle.safety_arbiter_motion_allowed.restype = BOOL
    handle.safety_arbiter_arbitrate.argtypes = [
        ctypes.POINTER(SafetyMotionCommand),
        ctypes.POINTER(SensorSnapshot),
        BOOL,
        ctypes.POINTER(SafetyMotionCommand),
    ]
    handle.safety_arbiter_arbitrate.restype = None
    handle.control_mlp_planner_init.argtypes = []
    handle.control_mlp_planner_init.restype = BOOL
    handle.control_mlp_planner_reset.argtypes = []
    handle.control_mlp_planner_reset.restype = None
    handle.control_mlp_planner_is_ready.argtypes = []
    handle.control_mlp_planner_is_ready.restype = BOOL
    handle.control_mlp_planner_step.argtypes = [
        ctypes.POINTER(SensorSnapshot),
        ctypes.c_float,
        ctypes.POINTER(ControlMlpOutput),
    ]
    handle.control_mlp_planner_step.restype = None
    return handle


def safety_arbiter_arbitrate_step(
    command: SafetyMotionCommand,
    snapshot: SensorSnapshot,
    sensor_fresh: bool = True,
) -> SafetyMotionCommand:
    handle = library()
    arbitrated = SafetyMotionCommand()
    handle.safety_arbiter_arbitrate(
        ctypes.byref(command),
        ctypes.byref(snapshot),
        BOOL(sensor_fresh),
        ctypes.byref(arbitrated),
    )
    return arbitrated


def safety_arbiter_check_allowed(
    snapshot: SensorSnapshot,
    sensor_fresh: bool = True,
) -> tuple[bool, SafetyArbiterStatus]:
    handle = library()
    status = SafetyArbiterStatus()
    allowed = handle.safety_arbiter_motion_allowed(
        ctypes.byref(snapshot),
        BOOL(sensor_fresh),
        ctypes.byref(status),
    )
    return allowed, status


def smooth_avoidance_plan_step(
    input_val: SmoothAvoidanceInput,
) -> SmoothAvoidanceOutput:
    handle = library()
    output = SmoothAvoidanceOutput()
    handle.smooth_avoidance_plan(ctypes.byref(input_val), ctypes.byref(output))
    return output


def control_mlp_plan_step(
    snapshot: SensorSnapshot | None,
    target_heading_deg: float = 0.0,
) -> ControlMlpOutput:
    handle = library()
    output = ControlMlpOutput()
    p_snap = ctypes.byref(snapshot) if snapshot is not None else None
    handle.control_mlp_planner_step(p_snap, ctypes.c_float(target_heading_deg), ctypes.byref(output))
    return output


def sound_follow_trace(inputs: Iterable[tuple[SoundFollowInput, int]]) -> list[SoundFollowOutput]:
    """Run one trace from the controller's public initialization boundary."""

    handle = library()
    handle.sound_follow_controller_init()
    outputs: list[SoundFollowOutput] = []
    for input_value, elapsed_ms in inputs:
        output = SoundFollowOutput()
        handle.sound_follow_controller_step(ctypes.byref(input_value), elapsed_ms, ctypes.byref(output))
        outputs.append(output)
    return outputs


def obstacle_avoidance_step(
    snapshot: SensorSnapshot, *, fault_active: bool = False, target_steering_deg: int = 0,
    linear_speed_mm_s: int = 120,
) -> ObstacleAvoidanceOutput:
    return obstacle_avoidance_trace(
        [snapshot], fault_active=fault_active, target_steering_deg=target_steering_deg,
        linear_speed_mm_s=linear_speed_mm_s,
    )[0]


def obstacle_avoidance_trace(
    snapshots: Iterable[SensorSnapshot], *, fault_active: bool = False,
    timestamps_ms: Iterable[int] | None = None, target_steering_deg: int = 0,
    linear_speed_mm_s: int = 120,
) -> list[ObstacleAvoidanceOutput]:
    """Run at the firmware's 100 ms period unless measured timestamps are supplied.

    update_count is kept verbatim: repeated snapshots must not create IMU progress.
    """
    handle = library()
    handle.obstacle_avoidance_controller_init()
    outputs: list[ObstacleAvoidanceOutput] = []
    snapshots = list(snapshots)
    times = list(timestamps_ms) if timestamps_ms is not None else [i * 100 for i in range(len(snapshots))]
    if len(times) != len(snapshots):
        raise ValueError("one timestamp is required per snapshot")
    target = ctypes.c_int16(target_steering_deg)
    speed = ctypes.c_int16(linear_speed_mm_s)
    for snapshot, now_ms in zip(snapshots, times):
        output = ObstacleAvoidanceOutput()
        handle.obstacle_avoidance_controller_step(
            ctypes.byref(snapshot), BOOL(fault_active), now_ms, target, speed, ctypes.byref(output)
        )
        outputs.append(output)
    return outputs


def sound_output_values(output: SoundFollowOutput) -> tuple[int, int, int, int, int, int, int, int]:
    return (
        output.state,
        output.steering_deg,
        output.target_bearing_deg,
        output.is_spin_turn,
        output.left_rpm,
        output.right_rpm,
        output.actuator_enable,
        output.emergency_stop,
    )
