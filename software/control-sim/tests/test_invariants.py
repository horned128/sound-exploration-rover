from controlsim.bindings import (
    ACOUSTIC_XVF_STATUS_READY,
    AcousticObservation,
    SoundFollowInput,
    sound_follow_trace,
)
from controlsim.invariants import assert_same_sound_outputs, assert_steering_within


def usable_loud_observation() -> AcousticObservation:
    return AcousticObservation(
        doa_deg=100,
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
