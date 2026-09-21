import ctypes
import math

from controlsim.bindings import (
    OdometryPose,
    SoundSourceLocalizerInput,
    SoundSourceLocalizerOutput,
    library,
)


def bearing_to(source_x: float, source_y: float, pose: OdometryPose) -> int:
    world = math.degrees(math.atan2(source_y - pose.y_mm, source_x - pose.x_mm))
    return round(pose.theta_deg_x10 / 10 - world)


def step(handle, *, x, y, now, sequence, source=(1000, 0), confidence=90, sound_valid=True):
    pose = OdometryPose(x_mm=x, y_mm=y, theta_mrad=0, theta_deg_x10=0, valid=1)
    value = SoundSourceLocalizerInput(
        pose=pose,
        relative_doa_deg=bearing_to(*source, pose),
        doa_confidence=confidence,
        new_observation=1,
        sound_valid=sound_valid,
        pose_valid=1,
        observation_sequence=sequence,
        now_ms=now,
    )
    output = SoundSourceLocalizerOutput()
    handle.sound_source_localizer_step(ctypes.byref(value), ctypes.byref(output))
    return output


def test_single_bearing_never_creates_a_range() -> None:
    handle = library()
    handle.sound_source_localizer_init()
    output = step(handle, x=0, y=0, now=0, sequence=1)
    assert output.observation_count == 1
    assert not output.source_position_valid
    assert not output.navigation_target_valid


def test_crossing_bearings_localize_source_with_geometry_diagnostics() -> None:
    handle = library()
    handle.sound_source_localizer_init()
    step(handle, x=0, y=0, now=0, sequence=1)
    # Rover moves to y=300 (left in world frame), so source at (1000, 0) is to the RIGHT (CW positive)
    output = step(handle, x=0, y=300, now=100, sequence=2)
    assert output.localization_geometry_valid
    assert output.source_position_valid
    assert output.navigation_target_valid
    assert abs(output.source_x_mm - 1000) < 40
    assert abs(output.source_y_mm) < 40
    assert output.baseline_mm >= 300
    assert output.bearing_crossing_angle_deg >= 12
    assert output.localization_residual_mm < 30
    # From y=300, source is to the right (+17 deg)
    assert 15 <= output.source_bearing_deg <= 19

    # From y=-300 (right in world frame), source at (1000, 0) is to the LEFT (CCW negative: -17 deg)
    output_left = step(handle, x=0, y=-300, now=200, sequence=3)
    assert -19 <= output_left.source_bearing_deg <= -15


def test_near_parallel_bearings_do_not_claim_a_range() -> None:
    handle = library()
    handle.sound_source_localizer_init()
    step(handle, x=0, y=0, now=0, sequence=1, source=(10000, 0))
    output = step(handle, x=0, y=200, now=100, sequence=2, source=(10000, 0))
    assert not output.localization_geometry_valid
    assert not output.source_position_valid


def test_arrival_requires_repeated_valid_observations_and_verify_time() -> None:
    handle = library()
    handle.sound_source_localizer_init()
    step(handle, x=0, y=0, now=0, sequence=1)
    step(handle, x=0, y=300, now=100, sequence=2)
    first = step(handle, x=700, y=0, now=200, sequence=3)
    assert first.arrival_state == 2
    assert first.arrival_confirm_count == 1
    second = step(handle, x=700, y=20, now=300, sequence=4)
    third = step(handle, x=700, y=-20, now=400, sequence=5)
    assert second.arrival_state == 2
    assert third.arrival_state == 2
    final = step(handle, x=700, y=0, now=500, sequence=6)
    assert final.arrival_state == 3
    assert final.source_range_mm <= 450


def test_invalid_sound_cannot_start_arrival_confirmation() -> None:
    handle = library()
    handle.sound_source_localizer_init()
    step(handle, x=0, y=0, now=0, sequence=1)
    step(handle, x=0, y=300, now=100, sequence=2)
    output = step(handle, x=700, y=0, now=200, sequence=3, sound_valid=False)
    assert not output.arrival_candidate
    assert output.arrival_state == 1
