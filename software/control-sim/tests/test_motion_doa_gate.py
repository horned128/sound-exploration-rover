"""実走行ログで発生したDoA飛びを、実際のC追従器で回帰検証する。"""

from controlsim.bindings import (
    ACOUSTIC_XVF_STATUS_READY,
    AcousticObservation,
    SoundFollowInput,
    sound_follow_trace,
)


def observation(raw_doa_deg: int) -> AcousticObservation:
    return AcousticObservation(
        doa_deg=raw_doa_deg,
        raw_doa_deg=raw_doa_deg,
        level_dbfs_x100=-2200,
        peak_dbfs_x100=-1800,
        doa_confidence=70,
        xvf_status=ACOUSTIC_XVF_STATUS_READY,
    )


def step(raw_doa_deg: int, heading_mrad: int) -> tuple[SoundFollowInput, int]:
    return (
        SoundFollowInput(
            link_ready=1,
            new_observation=1,
            motion_allowed=1,
            observation=observation(raw_doa_deg),
            pose_heading_valid=1,
            pose_heading_mrad=heading_mrad,
        ),
        100,
    )


def test_left_front_source_rejects_doa_that_moves_further_left_during_left_turn() -> None:
    """09:39:08の15→61度は左回頭と矛盾する。正しい343度で更新する。"""
    trace = [(SoundFollowInput(link_ready=1, motion_allowed=1), 500)]
    trace.extend(step(15, 0) for _ in range(16))
    trace.extend(step(61, -170) for _ in range(5))
    trace.extend(step(343, -740) for _ in range(5))

    outputs = sound_follow_trace(trace)
    assert outputs[16].state == 3
    assert outputs[21].state == 3
    assert -15 <= outputs[21].target_bearing_deg <= 5
    assert outputs[21].target_bearing_deg != -61
    assert outputs[-1].state == 3
    assert 10 <= outputs[-1].target_bearing_deg <= 25


def test_heading_wrap_does_not_turn_a_consistent_doa_into_a_jump() -> None:
    trace = [(SoundFollowInput(link_ready=1, motion_allowed=1), 500)]
    trace.extend(step(340, 3100) for _ in range(16))
    trace.extend(step(345, -3100) for _ in range(5))

    outputs = sound_follow_trace(trace)
    assert outputs[-1].state == 3
    assert outputs[-1].target_bearing_deg == 15


def test_transient_pose_loss_keeps_the_last_target_bearing() -> None:
    trace = [(SoundFollowInput(link_ready=1, motion_allowed=1), 500)]
    trace.extend(step(15, 0) for _ in range(16))
    trace.extend((
        SoundFollowInput(
            link_ready=1,
            new_observation=1,
            motion_allowed=1,
            observation=observation(61),
            pose_heading_valid=0,
        ),
        100,
    ) for _ in range(5))

    outputs = sound_follow_trace(trace)
    assert outputs[-1].state == 3
    assert outputs[-1].target_bearing_deg == -15
