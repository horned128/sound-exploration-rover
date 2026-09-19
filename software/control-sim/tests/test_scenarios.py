import ctypes

import pytest

from controlsim.bindings import ObstacleAvoidanceOutput, library, obstacle_avoidance_trace
from controlsim.scenarios import frontal_wall_approach, rover_width_obstacle_approach, sensor_snapshot

BACKUP, PIVOTS, BLOCKED = 9, (7, 8), 5


class Controller:
    def __init__(self, *, start_ms=0):
        self.handle = library()
        self.handle.obstacle_avoidance_controller_init()
        self.now_ms = start_ms

    def step(self, distances=(1000, 1000, 1000), *, gyro=0, dt=100, snapshot=None, fault=False):
        if snapshot is None:
            snapshot = sensor_snapshot(left_mm=distances[0], center_mm=distances[1], right_mm=distances[2])
            snapshot.gyro_dps_x10[2] = gyro
        output = ObstacleAvoidanceOutput()
        self.handle.obstacle_avoidance_controller_step(
            ctypes.byref(snapshot), fault, self.now_ms & 0xFFFFFFFF, ctypes.c_int16(0), ctypes.byref(output)
        )
        self.now_ms += dt
        return output

    def enter_pivot(self, *, distances=(1000, 1000, 1000)):
        self.step((360, 430, 320))
        for _ in range(25):
            output = self.step(distances)
            if output.rule in PIVOTS and output.left_rpm > 0:
                return output
        raise AssertionError("clearance and steering settling did not reach a moving turn")


def test_front_wall_starts_backup_before_contact_and_settles_steering() -> None:
    outputs = obstacle_avoidance_trace(frontal_wall_approach())
    assert outputs[-1].rule == BACKUP
    assert outputs[-1].steering_deg == 0
    # Direction changes first command zero RPM while centering the servos.
    assert outputs[-1].left_rpm == outputs[-1].right_rpm == 0
    assert outputs[-1].actuator_enable and not outputs[-1].emergency_stop


def test_rover_width_obstacle_keeps_turn_direction_until_early_backup() -> None:
    outputs = obstacle_avoidance_trace(rover_width_obstacle_approach())
    assert all(0 < output.steering_deg <= 45 for output in outputs[:3])
    assert all(output.left_rpm > output.right_rpm > 0 for output in outputs[:3])
    assert all(output.rule == BACKUP for output in outputs[3:])


def test_measured_small_side_crossings_do_not_reverse_the_turn() -> None:
    # Beginning of the 2026-09-15 wall loop: the old controller chose L,R,R,L.
    snapshots = [sensor_snapshot(left_mm=l, center_mm=c, right_mm=r) for l, c, r in (
        (804, 899, 803), (711, 807, 716), (650, 747, 658), (505, 595, 497),
    )]
    outputs = obstacle_avoidance_trace(snapshots)
    assert all(output.steering_deg < 0 for output in outputs)


def test_backup_requires_measured_clearance_and_has_a_time_limit() -> None:
    controller = Controller()
    outputs = [controller.step((360, 430, 320)) for _ in range(45)]
    assert any(output.left_rpm < 0 for output in outputs)
    assert not any(output.rule in PIVOTS for output in outputs)
    assert all(output.rule == BLOCKED and not output.actuator_enable for output in outputs[30:])


def test_backup_distance_alone_does_not_complete_the_turn() -> None:
    controller = Controller()
    output = controller.enter_pivot()
    assert output.steering_deg == -45
    outputs = [controller.step() for _ in range(10)]
    assert all(output.rule == 7 and output.steering_deg == -45 for output in outputs)
    # Gyro remains zero even though CENTER is already above the exit threshold.
    output = controller.step()
    assert output.rule in (7, BACKUP)


def test_backup_waits_for_both_front_corners_and_forward_translation() -> None:
    controller = Controller()
    controller.step((360, 430, 320))
    outputs = [controller.step((900, 1100, 300)) for _ in range(35)]
    assert not any(output.rule in PIVOTS for output in outputs)
    assert outputs[-1].rule == BLOCKED


@pytest.mark.parametrize("direction", [-1, 1])
def test_real_heading_change_and_stable_clear_exit_resume_forward(direction) -> None:
    controller = Controller()
    if direction == 1:
        controller.step((400, 700, 1100))
    controller.enter_pivot()
    outputs = [controller.step((1500, 1500, 1500), gyro=direction * 200) for _ in range(26)]
    assert all(output.rule in PIVOTS for output in outputs[:21])
    assert outputs[-1].rule in (3, 4)
    assert outputs[-1].steering_deg * direction >= 30
    # Front-facing ToFs can look clear while the vehicle flank is still near
    # the wall. Keep turning, then settle into cruise once heading is clear.
    for _ in range(26):
        controller.step((1500, 1500, 1500), gyro=direction * 200)
    for _ in range(10):
        output = controller.step((1500, 1500, 1500))
    assert output.rule == 1
    assert output.left_rpm == output.right_rpm == 120


def test_exit_must_stay_clear_after_sufficient_heading_change() -> None:
    controller = Controller()
    controller.enter_pivot()
    for _ in range(23):
        controller.step((1000, 850, 1000), gyro=-200)
    assert controller.step((1500, 1500, 1500), gyro=-200).rule == 7
    assert controller.step((1500, 850, 1500), gyro=-200).rule == 7
    assert controller.step((1500, 1500, 1500), gyro=-200).rule == 7
    assert controller.step((1500, 1500, 1500), gyro=-200).rule == 7
    assert controller.step((1500, 1500, 1500), gyro=-200).rule == 3


def test_same_imu_sample_or_oscillation_cannot_fake_heading_progress() -> None:
    for repeat in (True, False):
        controller = Controller()
        controller.enter_pivot()
        frozen = sensor_snapshot(left_mm=1500, center_mm=1500, right_mm=1500)
        frozen.gyro_dps_x10[2] = -200
        outputs = [controller.step(
            (1500, 1500, 1500), gyro=-200 if i % 2 == 0 else 200,
            snapshot=frozen if repeat else None,
        ) for i in range(12)]
        assert not any(output.rule == 1 for output in outputs)
        assert outputs[-1].rule == BACKUP


def test_reapproach_keeps_direction_but_stalled_turn_changes_direction() -> None:
    controller = Controller()
    controller.enter_pivot()
    for _ in range(8):
        controller.step(gyro=-200)
    assert controller.step((360, 490, 350), gyro=-200).rule == BACKUP
    outputs = [controller.step() for _ in range(15)]
    assert outputs[-1].rule == 7

    controller = Controller()
    controller.enter_pivot()
    for _ in range(12):
        output = controller.step()
    assert output.rule == BACKUP
    outputs = [controller.step((1400, 1400, 1400)) for _ in range(15)]
    assert outputs[-1].rule == 8
    assert outputs[-1].steering_deg == 45


def test_repeated_reapproach_is_bounded_and_rearms_only_after_stable_clearance() -> None:
    controller = Controller()
    controller.enter_pivot()
    for _ in range(3):
        output = controller.step((360, 490, 350))
        if output.rule == BLOCKED:
            break
        for _ in range(15):
            controller.step()
    assert output.rule == BLOCKED
    assert all(controller.step((600, 800, 600)).rule == BLOCKED for _ in range(25))
    outputs = [controller.step((1500, 1500, 1500)) for _ in range(10)]
    assert all(output.rule == BLOCKED for output in outputs[:9])
    assert outputs[-1].rule == 1


@pytest.mark.parametrize("interruption", ["fault", "stale", "invalid", "imu", "critical", "clock_gap"])
def test_safety_interrupts_recovery(interruption) -> None:
    controller = Controller()
    controller.enter_pivot()
    snapshot = sensor_snapshot(left_mm=1000, center_mm=1000, right_mm=1000)
    if interruption == "stale":
        snapshot.age_ms = 201
    elif interruption == "invalid":
        snapshot.valid_flags = 7
    elif interruption == "imu":
        snapshot.accel_mg[0] = 701
    elif interruption == "critical":
        snapshot.tof_distance_mm[0] = 150
    elif interruption == "clock_gap":
        controller.now_ms += 1000
    output = controller.step(snapshot=snapshot, fault=interruption == "fault")
    assert not output.actuator_enable
    assert output.left_rpm == output.right_rpm == 0


def test_wall_clock_and_wraparound_control_backup_duration() -> None:
    for start in (0, 0xFFFFFF00):
        controller = Controller(start_ms=start)
        outputs = [controller.step((360, 430, 320), dt=50) for _ in range(60)]
        assert outputs[7].left_rpm == 0
        assert outputs[8].left_rpm == -100  # 400 ms, independent of call count
        assert outputs[57].rule == BACKUP
        assert outputs[58].rule == BLOCKED  # 400 ms settling + 2500 ms reverse cap


def test_direction_is_held_above_exit_distance_until_corridor_is_stably_clear() -> None:
    controller = Controller()
    assert controller.step((800, 850, 800)).steering_deg < 0
    # CENTER crossing 900 alone must not release the committed turn direction.
    outputs = [controller.step((400, 950, 1500)) for _ in range(5)]
    assert all(output.steering_deg <= 0 for output in outputs)
    for _ in range(20):
        controller.step((1500, 1500, 1500), gyro=-1000)
    assert controller.step((500, 800, 1500)).steering_deg > 0


def test_continuous_turn_is_time_bounded_even_with_imu_progress() -> None:
    controller = Controller()
    controller.enter_pivot()
    outputs = [controller.step((1000, 850, 1000), gyro=-200) for _ in range(45)]
    assert all(output.rule == 7 for output in outputs[:44])
    assert outputs[-1].rule == BACKUP


def test_committed_turn_without_gyro_progress_cannot_circle_forever() -> None:
    controller = Controller()
    controller.step((800, 850, 800))
    outputs = [controller.step((1500, 1500, 1500)) for _ in range(50)]
    assert any(output.rule == BACKUP for output in outputs)
    assert outputs[-1].rule == BLOCKED and not outputs[-1].actuator_enable
