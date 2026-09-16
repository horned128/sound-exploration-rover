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
CPU0_ACOUSTIC_SUMMARY_DIMENSION = 96
CPU0_BACKGROUND_MODEL_INPUT_DIMENSION = 32
CPU0_BACKGROUND_MODEL_HIDDEN_DIMENSION = 16


class AcousticObservation(ctypes.Structure):
    _fields_ = [
        ("doa_deg", ctypes.c_uint16),
        ("level_dbfs_x100", ctypes.c_int16),
        ("peak_dbfs_x100", ctypes.c_int16),
        ("vad", ctypes.c_uint8),
        ("xvf_status", ctypes.c_uint8),
        ("audio_flags", ctypes.c_uint8),
        ("xvf_raw_status", ctypes.c_uint8),
        ("audio_frame_count", ctypes.c_uint32),
    ]


class SoundFollowInput(ctypes.Structure):
    _fields_ = [
        ("link_ready", BOOL),
        ("new_observation", BOOL),
        ("fault_active", BOOL),
        ("motion_allowed", BOOL),
        ("observation", AcousticObservation),
    ]


class SoundFollowOutput(ctypes.Structure):
    _fields_ = [
        ("state", ctypes.c_int32),
        ("steering_deg", ctypes.c_int16),
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
        ("left_rpm", ctypes.c_int16),
        ("right_rpm", ctypes.c_int16),
        ("actuator_enable", BOOL),
        ("emergency_stop", BOOL),
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


@lru_cache(maxsize=1)
def library() -> ctypes.CDLL:
    completed = subprocess.run(
        [sys.executable, str(BUILD_SCRIPT)], check=True, capture_output=True, text=True
    )
    native_library = completed.stdout.strip()
    handle = ctypes.CDLL(native_library)
    handle.sound_follow_controller_init.argtypes = []
    handle.sound_follow_controller_init.restype = None
    handle.sound_follow_controller_step.argtypes = [
        ctypes.POINTER(SoundFollowInput),
        ctypes.c_uint32,
        ctypes.POINTER(SoundFollowOutput),
    ]
    handle.sound_follow_controller_step.restype = None
    handle.obstacle_avoidance_controller_init.argtypes = []
    handle.obstacle_avoidance_controller_init.restype = None
    handle.obstacle_avoidance_controller_step.argtypes = [
        ctypes.POINTER(SensorSnapshot),
        BOOL,
        ctypes.c_uint32,
        ctypes.POINTER(ObstacleAvoidanceOutput),
    ]
    handle.obstacle_avoidance_controller_step.restype = None
    handle.acoustic_identifier_frame_is_active.argtypes = [ctypes.POINTER(ctypes.c_int8)]
    handle.acoustic_identifier_frame_is_active.restype = BOOL
    handle.acoustic_identifier_summary_create.argtypes = [
        ctypes.POINTER(ctypes.c_int8), ctypes.c_uint32, ctypes.POINTER(ctypes.c_int8), ctypes.POINTER(ctypes.c_uint8),
    ]
    handle.acoustic_identifier_summary_create.restype = BOOL
    handle.acoustic_identifier_cosine_distance.argtypes = [
        ctypes.POINTER(ctypes.c_int8), ctypes.POINTER(ctypes.c_int8), ctypes.POINTER(ctypes.c_float),
    ]
    handle.acoustic_identifier_cosine_distance.restype = BOOL
    handle.acoustic_identifier_leave_one_out_threshold.argtypes = [
        ctypes.POINTER(ctypes.c_int8), ctypes.c_uint32, ctypes.POINTER(ctypes.c_float),
    ]
    handle.acoustic_identifier_leave_one_out_threshold.restype = BOOL
    handle.acoustic_identifier_summary_classify.argtypes = [
        ctypes.POINTER(ctypes.c_int8), BOOL, ctypes.c_uint8, ctypes.POINTER(ctypes.c_int8), ctypes.c_uint32,
        ctypes.c_float, ctypes.POINTER(AcousticIdentifierSummaryOutput),
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
    return handle


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


def obstacle_avoidance_step(snapshot: SensorSnapshot, *, fault_active: bool = False) -> ObstacleAvoidanceOutput:
    return obstacle_avoidance_trace([snapshot], fault_active=fault_active)[0]


def obstacle_avoidance_trace(
    snapshots: Iterable[SensorSnapshot], *, fault_active: bool = False,
    timestamps_ms: Iterable[int] | None = None,
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
    for snapshot, now_ms in zip(snapshots, times):
        output = ObstacleAvoidanceOutput()
        handle.obstacle_avoidance_controller_step(
            ctypes.byref(snapshot), BOOL(fault_active), now_ms, ctypes.byref(output)
        )
        outputs.append(output)
    return outputs


def sound_output_values(output: SoundFollowOutput) -> tuple[int, int, int, int, int, int]:
    return (
        output.state,
        output.steering_deg,
        output.left_rpm,
        output.right_rpm,
        output.actuator_enable,
        output.emergency_stop,
    )
