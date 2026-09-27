"""音源旋回を前進回避より優先するための安全なToF条件。"""

import ctypes

from controlsim.bindings import BOOL, CPU0_SENSOR_VALID_ALL, ObstacleAvoidanceOutput, SensorSnapshot, library


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
    assert spin_allowed((839, 490, 375))
    assert not spin_allowed((839, 490, 99))
    assert spin_allowed((100, 500, 500))
    assert not spin_allowed((150, 287, 95))
    assert spin_allowed((281, 382, 276))
    assert spin_allowed((150, 287, 981))
    assert not spin_allowed((800, 800, 800), valid=False)
    assert not spin_allowed((800, 800, 800), age_ms=10000)


def test_rear_seam_chooses_open_side_but_does_not_bypass_clearance() -> None:
    assert rear_seam_preference((790, 945, 994)) == 1
    assert rear_seam_preference((994, 945, 790)) == -1
    assert rear_seam_preference((370, 700, 390)) == 0
    assert rear_seam_preference((500, 500, 500)) == 0
    assert rear_seam_preference((790, 945, 994), valid=False) == 0
    assert spin_allowed((234, 478, 577))


def test_sound_side_is_kept_when_front_obstacle_forces_backup() -> None:
    handle = library()
    handle.obstacle_avoidance_controller_init()

    def step(now_ms: int, update_count: int, tof_mm: tuple[int, int, int]) -> ObstacleAvoidanceOutput:
        snapshot = SensorSnapshot()
        snapshot.initialized = 1
        snapshot.valid_flags = CPU0_SENSOR_VALID_ALL
        snapshot.age_ms = 0
        snapshot.update_count = update_count
        snapshot.tof_distance_mm[:] = tof_mm
        snapshot.accel_mg[:] = (0, 0, 1000)
        output = ObstacleAvoidanceOutput()
        handle.obstacle_avoidance_controller_step(
            ctypes.byref(snapshot), BOOL(False), now_ms, ctypes.c_int16(-12), ctypes.c_int16(0),
            ctypes.byref(output),
        )
        return output

    front_obstacle_only = step(0, 1, (1500, 200, 1500))
    assert front_obstacle_only.rule == 7  # Clear left side: rotate toward the left source immediately.

    handle.obstacle_avoidance_controller_init()
    close_left = step(0, 1, (74, 189, 1642))
    assert close_left.rule == 9  # BACKUP: left source side lacks pivot clearance.
    assert close_left.left_rpm < 0 and close_left.right_rpm < 0

    backing = step(500, 2, (84, 196, 1647))
    assert backing.rule == 9

    source_side_clear = step(700, 3, (100, 217, 1644))
    assert source_side_clear.rule == 7  # 100 mm boundary allows PIVOT_LEFT toward the source.
    assert source_side_clear.is_spin_turn
