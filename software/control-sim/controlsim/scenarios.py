"""Synthetic inputs for portable control tests."""

from __future__ import annotations

from collections.abc import Iterator
from itertools import count

from .bindings import CPU0_SENSOR_VALID_ALL, SensorSnapshot

_sensor_generations = count(1)

def sensor_snapshot(*, left_mm: int, center_mm: int, right_mm: int) -> SensorSnapshot:
    snapshot = SensorSnapshot()
    snapshot.tof_distance_mm[:] = (left_mm, center_mm, right_mm)
    snapshot.valid_flags = CPU0_SENSOR_VALID_ALL
    snapshot.initialized = 1
    snapshot.accel_mg[:] = (0, 0, 1000)
    snapshot.update_count = next(_sensor_generations)
    return snapshot


def frontal_wall_approach() -> Iterator[SensorSnapshot]:
    """A forward wall approach ending inside the pivot-turn escape threshold."""

    for center_mm in (1200, 651, 251, 250):
        yield sensor_snapshot(left_mm=1000, center_mm=center_mm, right_mm=1000)


def rover_width_obstacle_approach() -> Iterator[SensorSnapshot]:
    """The measured approach that used to reverse its turn near the obstacle."""

    for left_mm, center_mm, right_mm in (
        (750, 901, 1282),
        (709, 857, 1229),
        (498, 621, 960),
        (382, 476, 386),
        (365, 452, 349),
        (312, 401, 294),
        (244, 347, 242),
    ):
        yield sensor_snapshot(left_mm=left_mm, center_mm=center_mm, right_mm=right_mm)
