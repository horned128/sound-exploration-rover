from controlsim.bindings import (
    ACOUSTIC_XVF_STATUS_READY,
    AcousticObservation,
    SoundFollowInput,
    sound_follow_trace,
)


STATE_STEER_PREP = 2
STATE_MOVE_STEP = 3
STATE_SETTLE = 4
STATE_SPIN_PREP = 17
STATE_SPIN_STEP = 18


def loud_observation(relative_doa_deg: int) -> AcousticObservation:
    """車体座標の右正DoAを、XVF3800の実機向きの生DoAへ変換する。"""
    raw_doa_deg = (-relative_doa_deg) % 360
    return AcousticObservation(
        doa_deg=raw_doa_deg,
        raw_doa_deg=raw_doa_deg,
        level_dbfs_x100=-3000,
        peak_dbfs_x100=-2800,
        vad=1,
        doa_confidence=90,
        xvf_status=ACOUSTIC_XVF_STATUS_READY,
    )


def stable_trace(
    relative_doa_deg: int, samples: int, *, gyro_z_dps_x10: int | None = None
) -> list[tuple[SoundFollowInput, int]]:
    observation = loud_observation(relative_doa_deg)
    if relative_doa_deg > 180:
        relative_doa_deg -= 360
    elif relative_doa_deg < -180:
        relative_doa_deg += 360
    if gyro_z_dps_x10 is None:
        # Current vehicle wiring reports the opposite Z sign from the
        # logical spin direction used by the controller.
        gyro_z_dps_x10 = -100 if relative_doa_deg > 0 else 100
    trace = [(SoundFollowInput(link_ready=1, motion_allowed=1), 500)]
    for update_count in range(1, samples + 1):
        trace.append(
            (
                SoundFollowInput(
                    link_ready=1,
                    new_observation=1,
                    motion_allowed=1,
                    observation=observation,
                    imu_valid=1,
                    gyro_z_dps_x10=gyro_z_dps_x10,
                    imu_update_count=update_count,
                ),
                100,
            )
        )
    return trace


def test_front_sound_keeps_the_existing_steer_then_move_sequence() -> None:
    outputs = sound_follow_trace(stable_trace(30, 16))

    assert any(output.state == STATE_STEER_PREP for output in outputs)
    assert any(output.state == STATE_MOVE_STEP for output in outputs)
    assert all(not output.is_spin_turn for output in outputs)


def test_listen_and_settle_disable_actuators_until_motion_is_prepared() -> None:
    outputs = sound_follow_trace(
        [
            (SoundFollowInput(link_ready=1, motion_allowed=1), 500),
            (SoundFollowInput(link_ready=1, motion_allowed=1, avoidance_relisten=1), 0),
            (SoundFollowInput(link_ready=1, motion_allowed=1), 500),
        ]
    )

    passive_states = {1, 4, 5, 19, 20, 21, 22}
    passive_outputs = [output for output in outputs if output.state in passive_states]
    assert passive_outputs
    assert all(not output.actuator_enable for output in passive_outputs)
    assert all(output.left_rpm == output.right_rpm == 0 for output in passive_outputs)


def test_avoidance_relisten_spins_toward_the_new_sound_before_moving() -> None:
    observation = loud_observation(75)
    trace = [
        (SoundFollowInput(link_ready=1, motion_allowed=1), 500),
        (SoundFollowInput(link_ready=1, motion_allowed=1, avoidance_relisten=1), 0),
    ]
    trace.extend(
        (
            SoundFollowInput(
                link_ready=1,
                new_observation=1,
                motion_allowed=1,
                observation=observation,
                imu_valid=1,
                gyro_z_dps_x10=-100,
                imu_update_count=update_count,
            ),
            100,
        )
        for update_count in range(1, 28)
    )

    outputs = sound_follow_trace(trace)
    spin_prep_index = next(index for index, output in enumerate(outputs) if output.state == STATE_SPIN_PREP)
    spin_step = next(output for output in outputs[spin_prep_index:] if output.state == STATE_SPIN_STEP)

    assert not any(output.state == STATE_MOVE_STEP for output in outputs[:spin_prep_index])
    assert spin_step.left_rpm > 0 > spin_step.right_rpm
    assert spin_step.actuator_enable


def test_avoidance_relisten_rechecks_heading_before_it_allows_forward_motion() -> None:
    """回避後は、同じ横方向DoAが残るなら正面域まで最大二回向き直す。"""
    observation = loud_observation(75)
    trace = [
        (SoundFollowInput(link_ready=1, motion_allowed=1), 500),
        (SoundFollowInput(link_ready=1, motion_allowed=1, avoidance_relisten=1), 0),
    ]
    trace.extend(
        (
            SoundFollowInput(
                link_ready=1,
                new_observation=1,
                motion_allowed=1,
                observation=observation,
                imu_valid=1,
                gyro_z_dps_x10=-700,
                imu_update_count=update_count,
            ),
            100,
        )
        for update_count in range(1, 80)
    )

    outputs = sound_follow_trace(trace)
    spin_prep_indices = [index for index, output in enumerate(outputs) if output.state == STATE_SPIN_PREP]
    first_move_index = next(index for index, output in enumerate(outputs) if output.state == STATE_MOVE_STEP)

    assert len(spin_prep_indices) >= 2
    assert spin_prep_indices[1] < first_move_index


def test_rear_sound_uses_gyro_target_and_reduces_rpm_near_it() -> None:
    outputs = sound_follow_trace(stable_trace(135, 35, gyro_z_dps_x10=-700))

    spin_prep_index = next(index for index, output in enumerate(outputs) if output.state == STATE_SPIN_PREP)
    spin_step_index = next(index for index, output in enumerate(outputs) if output.state == STATE_SPIN_STEP)
    assert spin_step_index > spin_prep_index
    assert outputs[spin_prep_index].is_spin_turn
    assert outputs[spin_prep_index].left_rpm == 0
    assert outputs[spin_prep_index].right_rpm == 0

    spin_outputs = [output for output in outputs if output.state == STATE_SPIN_STEP]
    assert spin_outputs
    assert 10 <= len(spin_outputs) < 22
    assert all(output.is_spin_turn for output in spin_outputs)
    assert all(output.left_rpm == -output.right_rpm for output in spin_outputs)
    assert any(abs(output.left_rpm) == 100 for output in spin_outputs)
    assert any(abs(output.left_rpm) == 85 for output in spin_outputs)
    assert any(output.state == STATE_SETTLE for output in outputs[spin_step_index + 1 :])


def test_rear_sound_on_each_side_selects_opposite_spin_directions() -> None:
    clockwise_outputs = sound_follow_trace(stable_trace(135, 17))
    counterclockwise_outputs = sound_follow_trace(stable_trace(210, 17))

    clockwise = next(output for output in clockwise_outputs if output.state == STATE_SPIN_STEP)
    counterclockwise = next(output for output in counterclockwise_outputs if output.state == STATE_SPIN_STEP)
    assert clockwise.left_rpm == -counterclockwise.left_rpm
    assert clockwise.right_rpm == -counterclockwise.right_rpm


def test_spin_reverses_once_when_live_doa_moves_away_from_the_target() -> None:
    """実機配線の左右差でDoAが逆進行しても、同じ向きに回り続けない。"""
    trace = stable_trace(135, 16, gyro_z_dps_x10=-100)
    opposite_motion = loud_observation(-160)
    trace.extend(
        (
            SoundFollowInput(
                link_ready=1,
                new_observation=1,
                motion_allowed=1,
                observation=opposite_motion,
                imu_valid=1,
                gyro_z_dps_x10=-100,
                imu_update_count=update_count,
            ),
            100,
        )
        for update_count in range(17, 22)
    )

    outputs = sound_follow_trace(trace)
    spin_outputs = [output for output in outputs if output.state == STATE_SPIN_STEP]
    assert spin_outputs[0].left_rpm > 0 > spin_outputs[0].right_rpm
    assert any(output.left_rpm < 0 < output.right_rpm for output in spin_outputs[1:])


def test_spin_stops_from_stable_front_doa_before_the_imu_yaw_limit() -> None:
    trace = stable_trace(135, 16, gyro_z_dps_x10=-100)
    front = loud_observation(10)
    trace.extend(
        (
            SoundFollowInput(
                link_ready=1,
                new_observation=1,
                motion_allowed=1,
                observation=front,
                imu_valid=1,
                gyro_z_dps_x10=-100,
                imu_update_count=update_count,
            ),
            100,
        )
        for update_count in range(17, 24)
    )

    outputs = sound_follow_trace(trace)
    spin_start = next(index for index, output in enumerate(outputs) if output.state == STATE_SPIN_STEP)
    assert any(output.state == STATE_SETTLE for output in outputs[spin_start + 1 :])


def test_rear_sound_stops_and_reports_no_progress_when_gyro_confirms_no_turn() -> None:
    outputs = sound_follow_trace(stable_trace(135, 40, gyro_z_dps_x10=0))

    no_progress = next(output for output in outputs if output.state == 19)
    assert no_progress.is_spin_turn
    assert no_progress.left_rpm == 0
    assert no_progress.right_rpm == 0
    assert len([output for output in outputs if output.state == STATE_SPIN_STEP]) == 25


def test_rear_sound_does_not_double_integrate_a_repeated_gyro_sample() -> None:
    trace = stable_trace(135, 16, gyro_z_dps_x10=-700)
    observation = loud_observation(135)
    trace.extend(
        (
            SoundFollowInput(
                link_ready=1,
                new_observation=1,
                motion_allowed=1,
                observation=observation,
                imu_valid=1,
                gyro_z_dps_x10=-700,
                imu_update_count=16,
            ),
            100,
        )
        for _ in range(60)
    )

    outputs = sound_follow_trace(trace)

    assert any(output.state == 19 for output in outputs)


def test_successful_rear_spin_relistens_without_waiting_for_silence() -> None:
    # 後方の連続音では、ジャイロ目標または時間上限の後に静音待ちへ入らずDoA測定へ戻る。
    outputs = sound_follow_trace(stable_trace(135, 80, gyro_z_dps_x10=-500))
    spin_start_indices = [
        index
        for index, output in enumerate(outputs)
        if output.state == STATE_SPIN_STEP and (0 == index or outputs[index - 1].state != STATE_SPIN_STEP)
    ]

    assert len(spin_start_indices) >= 2
    first_end = next(
        index
        for index in range(spin_start_indices[0], len(outputs))
        if outputs[index].state == STATE_SETTLE
    )
    assert any(output.state == 1 for output in outputs[first_end : spin_start_indices[1]])
