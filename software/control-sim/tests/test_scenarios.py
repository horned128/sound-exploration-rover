import ctypes

import pytest

from controlsim.bindings import ObstacleAvoidanceOutput, library, obstacle_avoidance_trace
from controlsim.scenarios import frontal_wall_approach, rover_width_obstacle_approach, sensor_snapshot

PIVOT_LEFT, PIVOT_RIGHT, BLOCKED, BACKUP = 7, 8, 5, 9
PIVOTS = (PIVOT_LEFT, PIVOT_RIGHT)


class Controller:
    def __init__(self, *, start_ms=0):
        self.handle = library()
        self.handle.obstacle_avoidance_controller_init()
        self.now_ms = start_ms

    def step(self, distances=(1000, 1000, 1000), *, gyro=0, dt=100, snapshot=None, fault=False, target=0):
        if snapshot is None:
            snapshot = sensor_snapshot(left_mm=distances[0], center_mm=distances[1], right_mm=distances[2])
            snapshot.gyro_dps_x10[2] = gyro
        output = ObstacleAvoidanceOutput()
        self.handle.obstacle_avoidance_controller_step(
            ctypes.byref(snapshot), fault, self.now_ms & 0xFFFFFFFF, ctypes.c_int16(target), ctypes.byref(output)
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


def test_front_wall_backs_up_after_two_close_samples() -> None:
    outputs = obstacle_avoidance_trace(frontal_wall_approach())
    assert outputs[-2].rule in PIVOTS
    assert outputs[-2].is_spin_turn
    assert outputs[-1].rule == BACKUP
    assert outputs[-1].left_rpm == outputs[-1].right_rpm < 0
    assert outputs[-1].actuator_enable and not outputs[-1].emergency_stop


def test_rover_width_obstacle_never_commands_both_wheels_negative() -> None:
    outputs = obstacle_avoidance_trace(rover_width_obstacle_approach())
    # 0.7 mより外側は音源方向を不要に打ち消さず、実際に接近してから回避する。
    assert all(output.steering_deg == 0 for output in outputs[:2])
    assert all(output.left_rpm == output.right_rpm > 0 for output in outputs[:2])
    assert 0 < outputs[2].steering_deg <= 45
    assert outputs[2].left_rpm > outputs[2].right_rpm > 0
    assert all(output.rule in PIVOTS for output in outputs[3:])
    assert_no_autonomous_backup(outputs)


def test_settle_is_time_based_then_opposite_wheel_turn_starts() -> None:
    controller = Controller()
    outputs = [controller.step((360, 430, 320), dt=50) for _ in range(12)]
    assert all(output.left_rpm == output.right_rpm == 0 for output in outputs[:8])
    assert outputs[8].left_rpm == -outputs[8].right_rpm
    assert outputs[8].is_spin_turn
    assert_no_autonomous_backup(outputs)


def test_no_heading_progress_backs_up_then_retries_the_clearer_direction() -> None:
    controller = Controller()
    outputs = [controller.step((360, 430, 320)) for _ in range(55)]
    moving = [output for output in outputs if output.left_rpm != 0]
    assert any(output.rule == PIVOT_LEFT for output in moving)
    assert not any(output.rule == PIVOT_RIGHT for output in moving)
    assert any(output.rule == BACKUP for output in outputs)
    assert outputs[-1].rule in PIVOTS
    assert outputs[-1].actuator_enable


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


def test_clearance_completion_is_reported_once_after_a_started_avoidance() -> None:
    controller = Controller()
    controller.step((360, 430, 320))
    for _ in range(5):
        controller.step()

    outputs = [controller.step((1500, 1500, 1500), gyro=200) for _ in range(28)]
    completed = [output for output in outputs if output.avoidance_completed]

    assert len(completed) == 1
    assert not completed[0].avoidance_in_progress
    assert_no_autonomous_backup(outputs)


def test_committed_avoidance_keeps_the_clear_sound_side_after_heading_change() -> None:
    controller = Controller()
    first = controller.step((1500, 650, 550), target=45)
    # 音源側(右)に550mmの余裕があれば、左がさらに広くても右へ抜ける。
    assert first.steering_deg >= 28

    outputs = [controller.step((1500, 650, 550), gyro=-200, target=45) for _ in range(18)]
    assert all(output.steering_deg > 0 for output in outputs)
    # 回頭後も音源引力が同方向に働くため、弱めずに右へ進める。
    assert outputs[-1].steering_deg >= 12


def test_escape_prefers_the_sound_side_when_it_is_narrower_but_still_clear() -> None:
    controller = Controller()
    # 右は左より狭いが、旋回掃引に必要な380mmを満たす。音源が右なら
    # 反対側の壁沿いへ逃げず、後退後も右へ抜ける方向を保つ。
    first = controller.step((900, 450, 430), target=35)
    assert first.steering_deg > 0


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


def test_repeated_no_progress_uses_bounded_backups_before_stopping() -> None:
    controller = Controller()
    outputs = [controller.step((360, 490, 350)) for _ in range(100)]
    backups = [output for output in outputs if output.rule == BACKUP]
    assert backups
    assert all(output.left_rpm == output.right_rpm < 0 for output in backups)
    assert outputs[-1].rule == BLOCKED
    assert not outputs[-1].actuator_enable


def test_repeated_true_frontal_collision_backs_up_before_bounded_stop() -> None:
    controller = Controller()
    outputs = [controller.step((1000, 120, 1000)) for _ in range(40)]
    assert outputs[0].rule in PIVOTS
    assert outputs[1].rule == BACKUP
    assert outputs[1].left_rpm == outputs[1].right_rpm < 0
    assert sum(output.rule == BACKUP for output in outputs) > 1
    assert outputs[-1].rule == BLOCKED
    assert not outputs[-1].actuator_enable


def test_backup_uses_sound_direction_when_escape_sides_are_tied() -> None:
    controller = Controller()
    for _ in range(2):
        output = controller.step((900, 120, 900), target=-30)
    assert output.rule == BACKUP

    for _ in range(7):
        output = controller.step((900, 300, 900), target=-30)

    assert output.rule == PIVOT_LEFT
    assert output.left_rpm == output.right_rpm == 0


def test_backup_does_not_turn_toward_a_narrower_sound_side() -> None:
    controller = Controller()
    for _ in range(2):
        output = controller.step((240, 120, 80), target=30)
    assert output.rule == BACKUP

    for _ in range(7):
        output = controller.step((240, 300, 80), target=30)

    assert output.rule == PIVOT_LEFT
    assert output.left_rpm == output.right_rpm == 0


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
        assert output.rule == BACKUP
        assert output.left_rpm == output.right_rpm < 0
        assert output.actuator_enable
        return
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
