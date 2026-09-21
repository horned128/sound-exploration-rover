import ctypes

import pytest

from controlsim.bindings import ObstacleAvoidanceOutput, library, obstacle_avoidance_trace
from controlsim.scenarios import frontal_wall_approach, rover_width_obstacle_approach, sensor_snapshot

PIVOT_LEFT, PIVOT_RIGHT, BLOCKED = 7, 8, 5
PIVOTS = (PIVOT_LEFT, PIVOT_RIGHT)


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

    def enter_moving_turn(self, *, distances=(1000, 1000, 1000)):
        self.step((360, 430, 320))
        for _ in range(10):
            output = self.step(distances)
            if output.rule in PIVOTS and output.left_rpm != 0:
                return output
        raise AssertionError("steering settling did not reach a moving heading-change turn")


def assert_no_autonomous_backup(outputs) -> None:
    assert all(not (output.left_rpm < 0 and output.right_rpm < 0) for output in outputs)
    assert all(output.rule != 9 for output in outputs)


def test_front_wall_starts_stopped_steering_settle_without_reverse() -> None:
    outputs = obstacle_avoidance_trace(frontal_wall_approach())
    assert outputs[-1].rule in PIVOTS
    assert outputs[-1].is_spin_turn
    assert outputs[-1].left_rpm == outputs[-1].right_rpm == 0
    assert outputs[-1].actuator_enable and not outputs[-1].emergency_stop
    assert_no_autonomous_backup(outputs)


def test_rover_width_obstacle_never_commands_both_wheels_negative() -> None:
    outputs = obstacle_avoidance_trace(rover_width_obstacle_approach())
    assert all(0 < output.steering_deg <= 45 for output in outputs[:3])
    assert all(output.left_rpm > output.right_rpm > 0 for output in outputs[:3])
    assert all(output.rule in PIVOTS for output in outputs[3:])
    assert_no_autonomous_backup(outputs)


def test_settle_is_time_based_then_opposite_wheel_turn_starts() -> None:
    controller = Controller()
    outputs = [controller.step((360, 430, 320), dt=50) for _ in range(12)]
    assert all(output.left_rpm == output.right_rpm == 0 for output in outputs[:8])
    assert outputs[8].left_rpm == -outputs[8].right_rpm
    assert outputs[8].is_spin_turn
    assert_no_autonomous_backup(outputs)


def test_no_heading_progress_retries_opposite_direction_without_distance_clearance_stop() -> None:
    controller = Controller()
    outputs = [controller.step((360, 430, 320)) for _ in range(55)]
    moving = [output for output in outputs if output.left_rpm != 0]
    assert any(output.rule == PIVOT_LEFT for output in moving)
    assert any(output.rule == PIVOT_RIGHT for output in moving)
    assert outputs[-1].rule in PIVOTS
    assert outputs[-1].actuator_enable
    assert_no_autonomous_backup(outputs)


@pytest.mark.parametrize("direction", [-1, 1])
def test_real_heading_change_and_stable_clear_exit_resume_forward(direction) -> None:
    controller = Controller()
    trigger = (360, 430, 320) if direction < 0 else (320, 430, 360)
    controller.step(trigger)
    for _ in range(5):
        controller.step()
    # 実機の現行配線では左旋回指令のZジャイロが正、右旋回指令が負。
    outputs = [controller.step((1500, 1500, 1500), gyro=-direction * 200) for _ in range(28)]
    assert any(output.rule in PIVOTS for output in outputs)
    assert outputs[-1].rule in (1, 3, 4)
    assert_no_autonomous_backup(outputs)


def test_exit_uses_heading_change_without_clearance_hold() -> None:
    controller = Controller()
    controller.enter_moving_turn()
    for _ in range(23):
        controller.step((1000, 850, 1000), gyro=200)
    assert controller.step((1500, 1500, 1500), gyro=200).rule in (1, 3, 4)
    assert controller.step((1500, 850, 1500), gyro=200).rule in (1, 3, 4)
    assert controller.step((1500, 1500, 1500), gyro=200).rule in (1, 3, 4)
    assert controller.step((1500, 1500, 1500), gyro=200).rule in (1, 3, 4)
    assert controller.step((1500, 1500, 1500), gyro=200).rule in (1, 3, 4)


def test_same_imu_sample_or_oscillation_cannot_fake_heading_progress() -> None:
    for repeat in (True, False):
        controller = Controller()
        controller.enter_moving_turn()
        frozen = sensor_snapshot(left_mm=1500, center_mm=1500, right_mm=1500)
        frozen.gyro_dps_x10[2] = -200
        outputs = [controller.step(
            (1500, 1500, 1500), gyro=-200 if i % 2 == 0 else 200,
            snapshot=frozen if repeat else None,
        ) for i in range(18)]
        assert not any(output.rule == 1 for output in outputs)
        assert outputs[-1].rule in PIVOTS
        assert_no_autonomous_backup(outputs)


def test_repeated_no_progress_does_not_latch_blocked() -> None:
    controller = Controller()
    outputs = [controller.step((360, 490, 350)) for _ in range(55)]
    assert outputs[-1].rule in PIVOTS
    assert all(output.actuator_enable for output in outputs)
    assert_no_autonomous_backup(outputs)


def test_repeated_true_frontal_collision_candidate_latches_blocked() -> None:
    controller = Controller()
    outputs = [controller.step((1000, 120, 1000)) for _ in range(3)]
    assert outputs[0].rule in PIVOTS
    assert outputs[1].rule in PIVOTS
    assert outputs[2].rule == BLOCKED
    assert not outputs[2].actuator_enable
    assert controller.step((1500, 1500, 1500)).rule == BLOCKED


@pytest.mark.parametrize("interruption", ["fault", "stale", "invalid", "imu", "critical", "clock_gap"])
def test_safety_interrupts_recovery(interruption) -> None:
    controller = Controller()
    controller.enter_moving_turn()
    snapshot = sensor_snapshot(left_mm=1000, center_mm=1000, right_mm=1000)
    if interruption == "stale":
        snapshot.age_ms = 201
    elif interruption == "invalid":
        snapshot.valid_flags = 7
    elif interruption == "imu":
        snapshot.accel_mg[0] = 701
    elif interruption == "critical":
        snapshot.tof_distance_mm[1] = 120
    elif interruption == "clock_gap":
        controller.now_ms += 1000
    output = controller.step(snapshot=snapshot, fault=interruption == "fault")
    if interruption == "critical":
        output = controller.step(snapshot=sensor_snapshot(left_mm=1000, center_mm=120, right_mm=1000))
        output = controller.step(snapshot=sensor_snapshot(left_mm=1000, center_mm=120, right_mm=1000))
    assert not output.actuator_enable
    assert output.left_rpm == output.right_rpm == 0


def test_wall_clock_wraparound_preserves_settle_and_retry_timing() -> None:
    for start in (0, 0xFFFFFF00):
        controller = Controller(start_ms=start)
        outputs = [controller.step((360, 430, 320), dt=50) for _ in range(35)]
        assert outputs[7].left_rpm == 0
        assert outputs[8].left_rpm == -outputs[8].right_rpm
        assert outputs[-1].rule in PIVOTS
        assert_no_autonomous_backup(outputs)


def test_committed_turn_without_gyro_progress_cannot_circle_forever() -> None:
    controller = Controller()
    controller.step((800, 850, 800))
    outputs = [controller.step((1500, 1500, 1500)) for _ in range(180)]
    assert not any(output.rule == BLOCKED for output in outputs)
    assert all(output.actuator_enable for output in outputs)
    assert_no_autonomous_backup(outputs)
