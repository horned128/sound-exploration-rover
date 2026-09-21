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


def loud_observation(doa_deg: int) -> AcousticObservation:
    return AcousticObservation(
        doa_deg=doa_deg,
        level_dbfs_x100=-3000,
        peak_dbfs_x100=-2800,
        vad=1,
        xvf_status=ACOUSTIC_XVF_STATUS_READY,
    )


def stable_trace(
    doa_deg: int, samples: int, *, gyro_z_dps_x10: int | None = None
) -> list[tuple[SoundFollowInput, int]]:
    observation = loud_observation(doa_deg)
    relative_doa_deg = -doa_deg
    if relative_doa_deg < -180:
        relative_doa_deg += 360
    if gyro_z_dps_x10 is None:
        gyro_z_dps_x10 = 100 if relative_doa_deg > 0 else -100
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
    assert any(abs(output.left_rpm) == 300 for output in spin_outputs)
    assert any(abs(output.left_rpm) == 220 for output in spin_outputs)
    assert any(output.state == STATE_SETTLE for output in outputs[spin_step_index + 1 :])


def test_rear_sound_on_each_side_selects_opposite_spin_directions() -> None:
    clockwise_outputs = sound_follow_trace(stable_trace(210, 17))
    counterclockwise_outputs = sound_follow_trace(stable_trace(135, 17))

    clockwise = next(output for output in clockwise_outputs if output.state == STATE_SPIN_STEP)
    counterclockwise = next(output for output in counterclockwise_outputs if output.state == STATE_SPIN_STEP)
    assert clockwise.left_rpm == -counterclockwise.left_rpm
    assert clockwise.right_rpm == -counterclockwise.right_rpm


def test_rear_sound_stops_and_reports_no_progress_when_gyro_confirms_no_turn() -> None:
    outputs = sound_follow_trace(stable_trace(135, 25, gyro_z_dps_x10=0))

    no_progress = next(output for output in outputs if output.state == 19)
    assert no_progress.is_spin_turn
    assert no_progress.left_rpm == 0
    assert no_progress.right_rpm == 0
    assert len([output for output in outputs if output.state == STATE_SPIN_STEP]) == 5


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
        for _ in range(12)
    )

    outputs = sound_follow_trace(trace)

    assert any(output.state == 19 for output in outputs)


def test_successful_rear_spin_relistens_without_waiting_for_silence() -> None:
    # 後方の連続音では、ジャイロ目標または時間上限の後に静音待ちへ入らずDoA測定へ戻る。
    outputs = sound_follow_trace(stable_trace(135, 80, gyro_z_dps_x10=-100))
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
