"""音源旋回を前進回避より優先するための安全なToF条件。"""

import ctypes

from controlsim.bindings import CPU0_SENSOR_VALID_ALL, SensorSnapshot, library


def spin_allowed(distances: tuple[int, int, int], *, valid: bool = True, age_ms: int = 0) -> bool:
    snapshot = SensorSnapshot()
    snapshot.initialized = 1
    snapshot.valid_flags = CPU0_SENSOR_VALID_ALL if valid else 0
    snapshot.age_ms = age_ms
    snapshot.tof_distance_mm[:] = distances
    snapshot.accel_mg[:] = (0, 0, 1000)
    return bool(library().obstacle_avoidance_spin_space_available(ctypes.byref(snapshot)))


def rear_seam_preference(distances: tuple[int, int, int], *, valid: bool = True) -> int:
    snapshot = SensorSnapshot()
    snapshot.initialized = 1
    snapshot.valid_flags = CPU0_SENSOR_VALID_ALL if valid else 0
    snapshot.tof_distance_mm[:] = distances
    return int(library().obstacle_avoidance_rear_seam_turn_preference(ctypes.byref(snapshot)))


def test_clear_rear_sound_can_spin_instead_of_forward_avoidance() -> None:
    assert spin_allowed((567, 691, 1230))
    assert spin_allowed((500, 500, 500))


def test_near_wall_or_invalid_sensor_must_use_recovery() -> None:
    assert not spin_allowed((839, 490, 375))
    assert not spin_allowed((281, 382, 276))
    assert not spin_allowed((150, 287, 981))
    assert not spin_allowed((800, 800, 800), valid=False)
    assert not spin_allowed((800, 800, 800), age_ms=10000)


def test_rear_seam_chooses_open_side_but_does_not_bypass_clearance() -> None:
    assert rear_seam_preference((790, 945, 994)) == 1
    assert rear_seam_preference((994, 945, 790)) == -1
    assert rear_seam_preference((370, 700, 390)) == 1
    assert rear_seam_preference((500, 500, 500)) == 0
    assert rear_seam_preference((790, 945, 994), valid=False) == 0
    assert not spin_allowed((234, 478, 577))
