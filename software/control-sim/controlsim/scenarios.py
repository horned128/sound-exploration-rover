"""Synthetic inputs for portable control tests."""

from __future__ import annotations

from collections.abc import Iterator

from .bindings import CPU0_SENSOR_VALID_ALL, SensorSnapshot


def sensor_snapshot(*, left_mm: int, center_mm: int, right_mm: int) -> SensorSnapshot:
    snapshot = SensorSnapshot()
    snapshot.tof_distance_mm[:] = (left_mm, center_mm, right_mm)
    snapshot.valid_flags = CPU0_SENSOR_VALID_ALL
    snapshot.initialized = 1
    return snapshot


def frontal_wall_approach() -> Iterator[SensorSnapshot]:
    """A forward wall approach ending at the existing 250 mm hard-stop threshold."""

    for center_mm in (1200, 651, 251, 250):
        yield sensor_snapshot(left_mm=1000, center_mm=center_mm, right_mm=1000)
