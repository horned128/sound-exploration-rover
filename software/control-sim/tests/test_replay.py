import json

from controlsim.replay import (
    iter_telemetry_records,
    replay_obstacle_avoidance,
    telemetry_to_sensor_snapshot,
)


def telemetry_line(**payload: object) -> str:
    return f"2026-09-12 12:00:00,000 - {json.dumps(payload)}\n"


def test_monitor_line_is_converted_without_reimplementing_controller() -> None:
    line = telemetry_line(
        schema=2,
        sensors={
            "valid_flags": 15,
            "tof_mm": [1000, 250, 1000],
            "accel_mg": [0, 0, 1000],
            "gyro_dps_x10": [0, 0, 0],
            "age_ms": 0,
            "update_count": 42,
        },
    )

    records = list(iter_telemetry_records([line]))
    snapshot = telemetry_to_sensor_snapshot(records[0].payload)
    outputs = replay_obstacle_avoidance([line])

    assert records[0].timestamp == "2026-09-12 12:00:00,000"
    assert snapshot.tof_distance_mm[:] == [1000, 250, 1000]
    assert snapshot.update_count == 42
    assert not outputs[0].actuator_enable
    assert outputs[0].left_rpm == outputs[0].right_rpm == 0
    assert not outputs[0].emergency_stop


def test_malformed_trace_is_rejected_or_explicitly_skipped() -> None:
    lines = ["not telemetry\n", telemetry_line(sensors={"valid_flags": 0})]

    try:
        list(iter_telemetry_records(lines))
    except ValueError as error:
        assert "line 1" in str(error)
    else:
        raise AssertionError("strict replay accepted a malformed line")

    records = list(iter_telemetry_records(lines, strict=False))
    assert len(records) == 1
    assert records[0].payload["sensors"]["valid_flags"] == 0
