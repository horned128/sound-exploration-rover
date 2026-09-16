import json
from pathlib import Path

from controlsim.replay import (
    iter_telemetry_records,
    replay_obstacle_avoidance,
    telemetry_to_sensor_snapshot,
)


def telemetry_line(**payload: object) -> str:
    return f"2026-09-12 12:00:00,000 - {json.dumps(payload)}\n"


def test_monitor_line_is_converted_and_triggers_pivot_escape() -> None:
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
    assert outputs[0].actuator_enable
    # 近接障害物検知直後は、壁につっかえないようまず直進後退（BACKUP）を開始
    assert outputs[0].steering_deg == 0
    assert outputs[0].left_rpm == 0
    assert outputs[0].right_rpm == 0
    assert not outputs[0].emergency_stop

    # 同一時刻のログを複製しても時間もクリアランスも進んでいない。
    extended_outputs = replay_obstacle_avoidance([line] * 17)
    assert all(output.rule == 9 and output.left_rpm == 0 for output in extended_outputs)


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


def test_recorded_wall_loop_does_not_claim_escape_without_heading_progress() -> None:
    source = Path(__file__).parent / "fixtures/wall_loop_20260915.jsonl"
    outputs = replay_obstacle_avoidance(source)
    assert len(outputs) == 72
    first_backup = next(i for i, output in enumerate(outputs) if output.rule == 9)
    assert all(output.steering_deg < 0 for output in outputs[:first_backup])
    # The historical world stops gaining clearance before the new 750 mm
    # target. Replaying it must time out, not invent a successful trajectory.
    assert all(output.rule in (9, 5) for output in outputs[first_backup:])
    assert outputs[-1].rule == 5 and not outputs[-1].actuator_enable


def test_replay_uses_cpu_time_and_does_not_advance_on_duplicate_packets() -> None:
    def line(time_ms):
        return telemetry_line(usb={"cpu_ms": time_ms, "cpu_seq": time_ms // 100}, sensors={
            "valid_flags": 15, "tof_mm": [360, 430, 320],
            "accel_mg": [0, 0, 1000], "gyro_dps_x10": [0, 0, 0], "age_ms": 0,
        })
    outputs = replay_obstacle_avoidance([line(t) for t in (0, 0, 0, 250, 500)])
    assert all(output.left_rpm == 0 for output in outputs[:4])
    assert outputs[-1].left_rpm == -100
