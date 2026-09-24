from controlsim.bindings import (
    ACOUSTIC_XVF_STATUS_READY,
    AcousticObservation,
    SoundFollowInput,
    library,
    sound_follow_trace,
)


STATE_STEER_PREP = 2
STATE_MOVE_STEP = 3
STATE_SETTLE = 4
STATE_SPIN_PREP = 17
STATE_SPIN_STEP = 18


def loud_observation(relative_doa_deg: int) -> AcousticObservation:
    """四方実測に基づき、右正の車体角をXVF3800のDoAへ変換する。"""
    relative_doa_deg = ((relative_doa_deg + 180) % 360) - 180
    raw_doa_deg = (relative_doa_deg if abs(relative_doa_deg) > 90 else -relative_doa_deg) % 360
    return AcousticObservation(
        doa_deg=raw_doa_deg,
        raw_doa_deg=raw_doa_deg,
        level_dbfs_x100=-3000,
        peak_dbfs_x100=-2800,
        vad=1,
        doa_confidence=90,
        xvf_status=ACOUSTIC_XVF_STATUS_READY,
    )


def test_four_quadrant_doa_mapping_matches_open_space_trial() -> None:
    convert = library().sound_follow_doa_to_relative
    assert [(raw, convert(raw)) for raw in (303, 58, 141, 237)] == [
        (303, 57),   # 右前
        (58, -58),   # 左前
        (141, 141),  # 右後
        (237, -123), # 左後
    ]


def test_front_trial_commands_forward_steer_toward_sound_on_each_side() -> None:
    for bearing_deg, expected_sign in ((57, 1), (-58, -1)):
        observation = loud_observation(bearing_deg)
        trace = [(SoundFollowInput(link_ready=1, motion_allowed=1), 500)]
        trace.extend((SoundFollowInput(
            link_ready=1, new_observation=1, motion_allowed=1,
            observation=observation, match_required=1,
            target_sound_matched=1, target_sound_direction_valid=1,
        ), 100) for _ in range(17))
        outputs = sound_follow_trace(trace)
        move = next(output for output in outputs if output.state == STATE_MOVE_STEP)
        assert not move.is_spin_turn
        assert move.target_bearing_deg * expected_sign > 0
        assert move.steering_deg * expected_sign > 0


def test_rear_trial_commands_turn_toward_sound_on_each_side() -> None:
    for raw_doa_deg, expected_sign in ((141, 1), (237, -1)):
        observation = AcousticObservation(
            doa_deg=raw_doa_deg,
            raw_doa_deg=raw_doa_deg,
            level_dbfs_x100=-3000,
            peak_dbfs_x100=-2800,
            vad=1,
            doa_confidence=90,
            xvf_status=ACOUSTIC_XVF_STATUS_READY,
        )
        trace = [(SoundFollowInput(link_ready=1, motion_allowed=1), 500)]
        trace.extend((SoundFollowInput(
            link_ready=1, new_observation=1, motion_allowed=1,
            observation=observation, match_required=1,
            target_sound_matched=1, target_sound_direction_valid=1,
            imu_valid=1, gyro_z_dps_x10=-100 * expected_sign,
            imu_update_count=index,
        ), 100) for index in range(1, 19))
        outputs = sound_follow_trace(trace)
        spin = next(output for output in outputs if output.state == STATE_SPIN_STEP)
        assert spin.target_bearing_deg * expected_sign > 90
        assert spin.left_rpm * expected_sign > 0
        assert spin.right_rpm * expected_sign < 0


def test_rear_seam_uses_open_side_without_changing_clear_rear_quadrants() -> None:
    for raw_doa_deg, preference, expected_sign in (
        (185, 1, 1),   # ログ12:56:44: 左790/右994mmなら右後方へ向く
        (175, -1, -1), # 左が広い場合はほぼ真後ろを左回りで向く
        (185, 0, -1),  # 差がない場合は従来のDoA符号を維持
        (141, -1, 1),  # 明瞭な右後方はToF差で左右反転させない
        (237, 1, -1),  # 明瞭な左後方も同様
        (180, 1, 1),  # 真後ろのちょうど180°でも右側を選べる
    ):
        observation = AcousticObservation(
            doa_deg=raw_doa_deg, raw_doa_deg=raw_doa_deg,
            level_dbfs_x100=-3000, peak_dbfs_x100=-2800,
            vad=1, doa_confidence=90, xvf_status=ACOUSTIC_XVF_STATUS_READY,
        )
        trace = [(SoundFollowInput(link_ready=1, motion_allowed=1), 500)]
        trace.extend((SoundFollowInput(
            link_ready=1, new_observation=1, motion_allowed=1,
            observation=observation, rear_seam_turn_preference=preference,
            imu_valid=1, gyro_z_dps_x10=-100 * expected_sign,
            imu_update_count=index,
        ), 100) for index in range(1, 19))
        outputs = sound_follow_trace(trace)
        spin = next(output for output in outputs if output.state == STATE_SPIN_STEP)
        assert spin.target_bearing_deg * expected_sign > 90
        assert spin.left_rpm * expected_sign > 0
        assert spin.right_rpm * expected_sign < 0


def test_rear_seam_doa_jitter_cannot_reverse_selected_spin() -> None:
    rear = loud_observation(-175)  # raw185°。空き側の右旋回を選ぶ。
    trace = [(SoundFollowInput(link_ready=1, motion_allowed=1), 500)]
    trace.extend((SoundFollowInput(
        link_ready=1, new_observation=1, motion_allowed=1,
        observation=rear, rear_seam_turn_preference=1,
        imu_valid=1, gyro_z_dps_x10=-100, imu_update_count=index,
    ), 100) for index in range(1, 18))
    jitter = loud_observation(-174)  # raw186°。符号境界の反対側に留まる。
    trace.extend((SoundFollowInput(
        link_ready=1, new_observation=1, motion_allowed=1,
        observation=jitter, rear_seam_turn_preference=1,
        imu_valid=1, gyro_z_dps_x10=-100, imu_update_count=index,
    ), 100) for index in range(18, 23))
    outputs = sound_follow_trace(trace)
    assert all(output.state == STATE_SPIN_STEP for output in outputs[-5:])
    assert all(output.left_rpm > 0 > output.right_rpm for output in outputs[-5:])


def test_side_boundary_doa_jump_does_not_reverse_rear_spin() -> None:
    trace = stable_trace(135, 16, gyro_z_dps_x10=-100)
    # 右後方の観測に左前の生DoAが一瞬混ざると、較正角は+135→-89°へ跳ぶ。
    side_glitch = AcousticObservation(
        doa_deg=89, raw_doa_deg=89, level_dbfs_x100=-3000,
        peak_dbfs_x100=-2800, vad=1, doa_confidence=90,
        xvf_status=ACOUSTIC_XVF_STATUS_READY,
    )
    trace.extend((SoundFollowInput(
        link_ready=1, new_observation=1, motion_allowed=1,
        observation=side_glitch, imu_valid=1, gyro_z_dps_x10=-100,
        imu_update_count=index,
    ), 100) for index in range(17, 22))
    outputs = sound_follow_trace(trace)
    assert all(output.state == STATE_SPIN_STEP for output in outputs[-5:])
    assert all(output.left_rpm > 0 > output.right_rpm for output in outputs[-5:])


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


def test_rear_doa_appearing_during_motion_stops_for_relisten() -> None:
    """移動中にTV側の後方DoAへ飛んでも、その場旋回へ直行しない。"""
    front = loud_observation(10)
    rear = loud_observation(150)
    trace = [(SoundFollowInput(link_ready=1, motion_allowed=1), 500)]
    trace.extend((SoundFollowInput(
        link_ready=1, new_observation=1, motion_allowed=1,
        observation=front, match_required=1, target_sound_matched=1,
        target_sound_direction_valid=1,
    ), 100) for _ in range(16))
    trace.extend((SoundFollowInput(
        link_ready=1, new_observation=1, motion_allowed=1,
        observation=rear, match_required=1, target_sound_matched=1,
        target_sound_direction_valid=1,
    ), 100) for _ in range(8))
    outputs = sound_follow_trace(trace)
    assert any(output.state == STATE_MOVE_STEP for output in outputs)
    assert outputs[-1].state == STATE_SETTLE
    assert not any(output.state == STATE_SPIN_PREP for output in outputs)
    assert outputs[-1].left_rpm == outputs[-1].right_rpm == 0


def test_tv_doa_does_not_replace_target_bearing_after_near_miss() -> None:
    """照合保持中の非TARGET区間では、後方の人声のDoAを操舵に混ぜない。"""
    front = loud_observation(10)
    tv = loud_observation(160)
    trace = [(SoundFollowInput(link_ready=1, motion_allowed=1), 500)]
    trace.extend((SoundFollowInput(
        link_ready=1, new_observation=1, motion_allowed=1,
        observation=front, match_required=1, target_sound_matched=1,
        target_sound_direction_valid=1,
    ), 100) for _ in range(16))
    trace.extend((SoundFollowInput(
        link_ready=1, new_observation=1, motion_allowed=1,
        observation=tv, match_required=1, target_sound_matched=1,
        target_sound_direction_valid=0,
    ), 100) for _ in range(5))
    outputs = sound_follow_trace(trace)
    assert outputs[-1].state == STATE_MOVE_STEP
    assert outputs[-1].target_bearing_deg == 10
    assert not any(output.state == STATE_SPIN_PREP for output in outputs)


def test_near_miss_during_spin_keeps_imu_turn_without_using_other_doa() -> None:
    """一致保持中の一度の未一致では、別音のDoAを見ずにIMU旋回を続ける。"""
    rear = loud_observation(135)
    trace = [(SoundFollowInput(link_ready=1, motion_allowed=1), 500)]
    trace.extend((SoundFollowInput(
        link_ready=1, new_observation=1, motion_allowed=1,
        observation=rear, match_required=1, target_sound_matched=1,
        target_sound_direction_valid=1, imu_valid=1, gyro_z_dps_x10=-100,
        imu_update_count=index,
    ), 100) for index in range(1, 19))
    trace.append((SoundFollowInput(
        link_ready=1, new_observation=1, motion_allowed=1,
        observation=loud_observation(-135), match_required=1, target_sound_matched=1,
        target_sound_direction_valid=0, imu_valid=1, gyro_z_dps_x10=-100,
        imu_update_count=19,
    ), 100))
    outputs = sound_follow_trace(trace)
    assert outputs[-2].state == STATE_SPIN_STEP
    assert outputs[-1].state == STATE_SPIN_STEP
    assert outputs[-1].target_bearing_deg == 135
    assert outputs[-1].left_rpm > 0 > outputs[-1].right_rpm


def test_confirmed_target_loss_during_spin_stops_yaw() -> None:
    """保持期限切れ・別音確定では従来どおり停止する。"""
    rear = loud_observation(135)
    trace = [(SoundFollowInput(link_ready=1, motion_allowed=1), 500)]
    trace.extend((SoundFollowInput(
        link_ready=1, new_observation=1, motion_allowed=1,
        observation=rear, match_required=1, target_sound_matched=1,
        target_sound_direction_valid=1, imu_valid=1, gyro_z_dps_x10=-100,
        imu_update_count=index,
    ), 100) for index in range(1, 19))
    trace.append((SoundFollowInput(
        link_ready=1, new_observation=1, motion_allowed=1,
        observation=rear, match_required=1, target_sound_matched=0,
        target_sound_direction_valid=0,
    ), 100))
    outputs = sound_follow_trace(trace)
    assert outputs[-2].state == STATE_SPIN_STEP
    assert outputs[-1].state == STATE_SETTLE
    assert outputs[-1].left_rpm == outputs[-1].right_rpm == 0


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
    """音源が-90〜90度の外側（105度）にある場合、その場旋回で正面を音源に向ける。"""
    observation = loud_observation(105)
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
    """回避後、90度外側の横方向DoA（105度）ならスピンターンし、正面域へ入ったら前進する。"""
    observation = loud_observation(105)
    front_observation = loud_observation(10)
    trace = [
        (SoundFollowInput(link_ready=1, motion_allowed=1), 500),
        (SoundFollowInput(link_ready=1, motion_allowed=1, avoidance_relisten=1), 0),
    ]
    # 最初の旋回（105度音源に向かってスピン）
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
        for update_count in range(1, 20)
    )
    # 正面を向いた後の観測（10度）で前進（MOVE_STEP）へ遷移
    trace.extend(
        (
            SoundFollowInput(
                link_ready=1,
                new_observation=1,
                motion_allowed=1,
                observation=front_observation,
                imu_valid=1,
                gyro_z_dps_x10=0,
                imu_update_count=update_count,
            ),
            100,
        )
        for update_count in range(20, 55)
    )

    outputs = sound_follow_trace(trace)
    spin_prep_indices = [index for index, output in enumerate(outputs) if output.state == STATE_SPIN_PREP]
    first_move_index = next(index for index, output in enumerate(outputs) if output.state == STATE_MOVE_STEP)

    assert len(spin_prep_indices) >= 1
    assert spin_prep_indices[0] < first_move_index


def test_avoidance_relisten_moves_forward_directly_for_front_quadrant_sound() -> None:
    """音源が-90〜90度以内（75度）の場合、その場旋回を暴発させず前進操舵で追従する。"""
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
                gyro_z_dps_x10=0,
                imu_update_count=update_count,
            ),
            100,
        )
        for update_count in range(1, 28)
    )

    outputs = sound_follow_trace(trace)
    # その場旋回（SPIN_PREP / SPIN_STEP）に入らず、前進操舵（STEER_PREP / MOVE_STEP）へ進むこと
    assert not any(output.state == STATE_SPIN_PREP for output in outputs)
    assert any(output.state == STATE_MOVE_STEP for output in outputs)


def test_negative_angle_doa_spins_shortest_left() -> None:
    """左後方（theta=-135度）の音源に対し、右への大回転ではなく最短で左旋回（left_rpm < 0 < right_rpm）する。"""
    outputs = sound_follow_trace(stable_trace(-135, 17))
    spin_step = next(output for output in outputs if output.state == STATE_SPIN_STEP)
    assert spin_step.is_spin_turn
    assert spin_step.left_rpm < 0 < spin_step.right_rpm



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


def test_spin_does_not_reverse_when_live_doa_moves_away_from_the_target() -> None:
    """壁の反射・前後境界のDoA飛びだけで、確定した旋回方向を反転しない。"""
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
    assert all(output.left_rpm > 0 > output.right_rpm for output in spin_outputs)


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
