"""Reject ctypes bindings that no longer match the C structure layout."""

from __future__ import annotations

import ctypes
import json
import subprocess
import sys

from .bindings import (
    AcousticObservation,
    ObstacleAvoidanceOutput,
    SensorDiagnostics,
    SensorSnapshot,
    SoundFollowInput,
    SoundFollowOutput,
)
from .bindings import BUILD_SCRIPT


STRUCTURES = {
    "sound_follow_input_t": SoundFollowInput,
    "sound_follow_output_t": SoundFollowOutput,
    "acoustic_observation_t": AcousticObservation,
    "sensor_diagnostics_t": SensorDiagnostics,
    "sensor_snapshot_t": SensorSnapshot,
    "obstacle_avoidance_output_t": ObstacleAvoidanceOutput,
}


def python_layout(structure: type[ctypes.Structure]) -> dict[str, int]:
    return {
        "size": ctypes.sizeof(structure),
        **{field_name: getattr(structure, field_name).offset for field_name, _ in structure._fields_},
    }


def native_layout() -> dict[str, dict[str, int]]:
    completed = subprocess.run(
        [sys.executable, str(BUILD_SCRIPT), "--layout"], check=True, capture_output=True, text=True
    )
    return json.loads(completed.stdout)


def assert_layout_matches() -> None:
    native = native_layout()
    expected = {name: python_layout(structure) for name, structure in STRUCTURES.items()}
    assert native == expected

