from controlsim.bindings import (
    ACOUSTIC_XVF_STATUS_READY,
    AcousticObservation,
    SoundFollowInput,
    sound_follow_trace,
)
from controlsim.invariants import assert_same_sound_outputs, assert_steering_within


def usable_loud_observation() -> AcousticObservation:
    return AcousticObservation(
        doa_deg=90,
        level_dbfs_x100=-3000,
        peak_dbfs_x100=-2800,
        vad=1,
        xvf_status=ACOUSTIC_XVF_STATUS_READY,
    )


def acquisition_trace() -> list[tuple[SoundFollowInput, int]]:
    observation = usable_loud_observation()
    trace = [
        (SoundFollowInput(link_ready=1, motion_allowed=1), 500),
        (SoundFollowInput(link_ready=1, new_observation=1, motion_allowed=1, observation=observation), 0),
    ]
    trace.extend(
        (SoundFollowInput(link_ready=1, new_observation=1, motion_allowed=1, observation=observation), 100)
        for _ in range(9)
    )
    return trace


def test_steering_is_bounded_by_existing_sound_follow_limit() -> None:
    outputs = sound_follow_trace(acquisition_trace())
    assert_steering_within(outputs, maximum_degrees=45)
    assert any(output.steering_deg != 0 for output in outputs)


def test_same_initialized_input_trace_has_same_output_trace() -> None:
    first = sound_follow_trace(acquisition_trace())
    second = sound_follow_trace(acquisition_trace())
    assert_same_sound_outputs(first, second)


def test_continuous_sound_tracking_maintains_motion_beyond_1_second() -> None:
    """S4検証: 音源が継続している間、1秒バーストで強制停止せず連続走行を維持すること"""
    obs = usable_loud_observation()
    # リンク確立 (500ms)
    trace = [(SoundFollowInput(link_ready=1, motion_allowed=1), 500)]
    # 50ms周期で60ステップ (3000ms) 音源観測を継続
    trace.extend(
        (SoundFollowInput(link_ready=1, new_observation=1, motion_allowed=1, observation=obs), 50)
        for _ in range(60)
    )
    outputs = sound_follow_trace(trace)
    # STEER_PREP (500ms) 後に MOVE_STEP (state=3) に遷移
    # 3000msの間音が鳴り続けているので、1000ms(20ステップ)を超えても MOVE_STEP を維持
    move_step_outputs = [o for o in outputs if o.state == 3]
    # MOVE_STEP が 30 ステップ以上 (1500ms以上) 継続すること
    assert len(move_step_outputs) >= 30
    # 最後のステップでも依然として走行中（MOVE_STEP）であること
    assert outputs[-1].state == 3
    assert outputs[-1].actuator_enable == 1
    assert outputs[-1].left_rpm > 0 and outputs[-1].right_rpm > 0


def test_identifier_match_required_blocks_motion_when_unmatched() -> None:
    """Issue #9検証: match_required有効時、非対象音（unmatched）は大音量でも走行開始しない"""
    obs = usable_loud_observation()
    # リンク確立 (500ms)
    trace = [(SoundFollowInput(link_ready=1, motion_allowed=1), 500)]
    # match_required=1 だが target_sound_matched=0 の音響観測列
    trace.extend(
        (
            SoundFollowInput(
                link_ready=1,
                new_observation=1,
                motion_allowed=1,
                observation=obs,
                match_required=1,
                target_sound_matched=0,
            ),
            50,
        )
        for _ in range(40)
    )
    outputs = sound_follow_trace(trace)
    # 一度も MOVE_STEP (state=3) または STEER_PREP (state=2) に入らず LISTEN (state=1) を維持
    assert all(o.state not in (2, 3) for o in outputs)
    assert outputs[-1].left_rpm == 0

def test_trigger_active_resets_immediately_when_sound_becomes_unmatched() -> None:
    """非対象音検知による即時トリガー解除の検証:
    LISTEN 中に対象音で DoA 蓄積を開始しても、蓄積途中で非対象音（target_sound_matched=0）
    になると直ちに検知がリセットされ、STEER_PREP や MOVE_STEP へ遷移しない。"""
    obs = usable_loud_observation()
    trace = [(SoundFollowInput(link_ready=1, motion_allowed=1), 500)]
    # 最初に対象音一致が1ステップ入り、トリガー開始 (50ms)
    trace.append(
        (
            SoundFollowInput(
                link_ready=1,
                new_observation=1,
                motion_allowed=1,
                observation=obs,
                match_required=1,
                target_sound_matched=1,
            ),
            50,
        )
    )
    # その直後に非対象音（target_sound_matched=0）が継続 (30ステップ = 1500ms)
    trace.extend(
        (
            SoundFollowInput(
                link_ready=1,
                new_observation=1,
                motion_allowed=1,
                observation=obs,
                match_required=1,
                target_sound_matched=0,
            ),
            50,
        )
        for _ in range(30)
    )
    outputs = sound_follow_trace(trace)
    # 一度も走行（STEER_PREP: state=2, MOVE_STEP: state=3）に入らず、LISTEN (state=1) を維持すること
    assert all(o.state == 1 for o in outputs)
    assert all(o.left_rpm == 0 and o.right_rpm == 0 for o in outputs)
def test_identifier_match_required_allows_motion_when_matched() -> None:
    """Issue #9検証: match_required有効時、対象音（matched）で正常に追従走行を開始する"""
    obs = usable_loud_observation()
    # リンク確立 (500ms)
    trace = [(SoundFollowInput(link_ready=1, motion_allowed=1), 500)]
    # match_required=1 かつ target_sound_matched=1
    trace.extend(
        (
            SoundFollowInput(
                link_ready=1,
                new_observation=1,
                motion_allowed=1,
                observation=obs,
                match_required=1,
                target_sound_matched=1,
            ),
            50,
        )
        for _ in range(40)
    )
    outputs = sound_follow_trace(trace)
    # STEER_PREP を経て MOVE_STEP (state=3) に遷移して走行していること
    move_step_outputs = [o for o in outputs if o.state == 3]
    assert len(move_step_outputs) > 0
    assert outputs[-1].state == 3
    assert outputs[-1].left_rpm > 0 and outputs[-1].right_rpm > 0


def test_identifier_match_lost_initiates_settle_after_timeout() -> None:
    """Issue #9検証: match_required有効時、走行中に対象音一致が途絶えると静定停止へ遷移する"""
    obs = usable_loud_observation()
    trace = [(SoundFollowInput(link_ready=1, motion_allowed=1), 500)]
    # まず対象音一致で走行開始 (30ステップ = 1500ms)
    trace.extend(
        (
            SoundFollowInput(
                link_ready=1,
                new_observation=1,
                motion_allowed=1,
                observation=obs,
                match_required=1,
                target_sound_matched=1,
            ),
            50,
        )
        for _ in range(30)
    )
    # 続いて大音量は鳴っているが対象音一致が消失 (target_sound_matched=0) が 35ステップ (1750ms > 1500ms) 継続
    trace.extend(
        (
            SoundFollowInput(
                link_ready=1,
                new_observation=1,
                motion_allowed=1,
                observation=obs,
                match_required=1,
                target_sound_matched=0,
            ),
            50,
        )
        for _ in range(35)
    )
    outputs = sound_follow_trace(trace)
    # 最終的に SETTLE (state=4) または COOLDOWN (state=5) または LISTEN (state=1) へ遷移し走行停止
    assert outputs[-1].state in (1, 4, 5)
    assert outputs[-1].left_rpm == 0
    assert outputs[-1].right_rpm == 0
