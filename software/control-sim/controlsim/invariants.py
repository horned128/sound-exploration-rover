"""Assertions shared by control-sim tests."""

from __future__ import annotations

from collections.abc import Iterable

from .bindings import SoundFollowOutput, sound_output_values


def assert_steering_within(outputs: Iterable[SoundFollowOutput], maximum_degrees: int) -> None:
    for output in outputs:
        assert abs(output.steering_deg) <= maximum_degrees


def assert_same_sound_outputs(
    first: Iterable[SoundFollowOutput], second: Iterable[SoundFollowOutput]
) -> None:
    assert [sound_output_values(output) for output in first] == [sound_output_values(output) for output in second]

