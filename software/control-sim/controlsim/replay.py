"""Replay rover-monitor JSON Lines through the portable C controllers.

The monitor writes a logging prefix before each JSON object.  This module keeps
that transport detail at the boundary and converts only the sensor fields
needed by the C API.  Controller behavior is never reimplemented in Python.
"""

from __future__ import annotations

import json
from collections.abc import Iterable, Iterator, Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from .bindings import SensorSnapshot, obstacle_avoidance_step


JSONValue = Any
TelemetrySource = str | Path | Iterable[str]


@dataclass(frozen=True)
class TelemetryRecord:
    """One decoded monitor record, retaining its source line number."""

    line_number: int
    timestamp: str | None
    payload: dict[str, JSONValue]


def _decode_line(line: str, line_number: int) -> TelemetryRecord | None:
    stripped = line.strip()
    if not stripped:
        return None

    json_start = stripped.find("{")
    if json_start < 0:
        raise ValueError(f"line {line_number}: JSON object not found")

    prefix = stripped[:json_start].strip()
    timestamp = prefix[:-2].strip() if prefix.endswith("-") else (prefix or None)
    try:
        payload = json.loads(stripped[json_start:])
    except json.JSONDecodeError as error:
        raise ValueError(f"line {line_number}: invalid telemetry JSON: {error.msg}") from error
    if not isinstance(payload, dict):
        raise ValueError(f"line {line_number}: telemetry root must be an object")
    return TelemetryRecord(line_number=line_number, timestamp=timestamp, payload=payload)


def iter_telemetry_records(source: TelemetrySource, *, strict: bool = True) -> Iterator[TelemetryRecord]:
    """Yield decoded records from a file path or an iterable of text lines.

    With ``strict=False``, blank and malformed lines are skipped.  Strict mode
    is the default so a damaged trace cannot silently become a shorter test.
    """

    if isinstance(source, (str, Path)):
        lines: Iterable[str] = Path(source).open(encoding="utf-8")
    else:
        lines = source

    try:
        for line_number, line in enumerate(lines, start=1):
            try:
                record = _decode_line(line, line_number)
            except ValueError:
                if strict:
                    raise
                continue
            if record is not None:
                yield record
    finally:
        if isinstance(source, (str, Path)):
            lines.close()  # type: ignore[attr-defined]


def _bounded_int(value: JSONValue, *, minimum: int, maximum: int, default: int) -> int:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return default
    integer = int(value)
    return min(max(integer, minimum), maximum)


def _vector(payload: Mapping[str, JSONValue], key: str, *, length: int, minimum: int, maximum: int) -> tuple[int, ...]:
    value = payload.get(key)
    if not isinstance(value, list):
        return (0,) * length
    return tuple(
        _bounded_int(value[index] if index < len(value) else 0, minimum=minimum, maximum=maximum, default=0)
        for index in range(length)
    )


def telemetry_to_sensor_snapshot(payload: Mapping[str, JSONValue]) -> SensorSnapshot:
    """Convert one monitor payload to the C ``sensor_snapshot_t`` layout."""

    sensors_value = payload.get("sensors")
    sensors = sensors_value if isinstance(sensors_value, Mapping) else {}
    valid_flags = _bounded_int(sensors.get("valid_flags", 0), minimum=0, maximum=0xFF, default=0)

    snapshot = SensorSnapshot()
    snapshot.tof_distance_mm[:] = _vector(
        sensors, "tof_mm", length=3, minimum=0, maximum=0xFFFF
    )
    snapshot.accel_mg[:] = _vector(sensors, "accel_mg", length=3, minimum=-0x8000, maximum=0x7FFF)
    snapshot.gyro_dps_x10[:] = _vector(
        sensors, "gyro_dps_x10", length=3, minimum=-0x8000, maximum=0x7FFF
    )
    snapshot.age_ms = _bounded_int(sensors.get("age_ms", 0xFFFFFFFF), minimum=0, maximum=0xFFFFFFFF, default=0xFFFFFFFF)
    snapshot.update_count = _bounded_int(
        sensors.get("update_count", 0), minimum=0, maximum=0xFFFFFFFF, default=0
    )
    snapshot.error_flags = _bounded_int(sensors.get("error_flags", 0), minimum=0, maximum=0xFFFFFFFF, default=0)
    snapshot.last_error = _bounded_int(sensors.get("last_error", 0), minimum=-(1 << 31), maximum=(1 << 31) - 1, default=0)
    snapshot.valid_flags = valid_flags
    # Older schema-2 logs do not expose ``initialized``.  A nonzero valid mask
    # is the conservative indication that the snapshot had been initialized.
    snapshot.initialized = int(bool(sensors.get("initialized", valid_flags != 0)))
    return snapshot


def replay_obstacle_avoidance(
    source: TelemetrySource, *, fault_active: bool = False, strict: bool = True
) -> list[Any]:
    """Run every decoded sensor snapshot through the actual C controller."""

    return [
        obstacle_avoidance_step(telemetry_to_sensor_snapshot(record.payload), fault_active=fault_active)
        for record in iter_telemetry_records(source, strict=strict)
    ]
