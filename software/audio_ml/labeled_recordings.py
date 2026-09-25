"""Extract reboot-aware TARGET-only and actual SW-debug motor-only PCM spans.

Motor audio is labeled only while CPU0 commands both wheels and encoder feedback
reports rotation. TARGET-only is a session label, not a per-frame event annotation.
"""
from __future__ import annotations

import argparse
import json
from datetime import datetime
from pathlib import Path

from pcm_dataset import RATE, convert, pcm_sessions


def load_status(path: Path) -> list[dict]:
    with path.open(encoding="utf-8") as source:
        return [r for line in source if (r := json.loads(line)).get("cpu_valid") and "command" in r]


def telemetry_motor_on(r: dict) -> bool:
    cmd = r["command"]
    actuator = r.get("actuator", {})
    return (r.get("think", {}).get("name") == "SENSOR_FORWARD" and cmd.get("enable") == 1 and
            cmd.get("left_rpm", 0) > 0 and cmd.get("right_rpm", 0) > 0 and
            actuator.get("valid") == 1 and
            (abs(actuator.get("left_encoder_rpm_x10", 0)) >= 100 or
             abs(actuator.get("right_encoder_rpm_x10", 0)) >= 100) and
            r.get("recognition", {}).get("status_name") != "TARGET")


def contiguous_intervals(times: list[float], max_gap: float = .6) -> list[tuple[float, float]]:
    if not times:
        return []
    grouped = []
    start = previous = times[0]
    for time in times[1:]:
        if time - previous > max_gap:
            grouped.append((start, previous))
            start = time
        previous = time
    grouped.append((start, previous))
    return grouped


def intersect_run(first: int, raw: bytes, start: int, end: int, minimum_s: float = .5) -> tuple[int, int] | None:
    left = max(first, start)
    right = min(first + len(raw) // 2, end)
    return (left, right) if right - left >= minimum_s * RATE else None


def build(motor_log: Path, target_log: Path, output: Path) -> dict:
    annotations: dict[str, list[dict]] = {}
    report: dict = {"sessions": {}, "constraints": "motor has encoder-confirmed motion; target is session-level, not frame truth"}
    for path, label in ((motor_log, "motor"), (target_log, "target_possible")):
        sessions = pcm_sessions(path)
        telemetry = load_status(path)
        spans = []
        session_report = []
        for s in sessions:
            if "audio_clock_anchor_epoch_s" not in s:
                raise ValueError(f"missing host timestamps: {path}")
            anchor = s["audio_clock_anchor_epoch_s"]
            bounds = (min(first for first, _ in s["runs"]) / RATE + anchor,
                      max(first + len(raw) // 2 for first, raw in s["runs"]) / RATE + anchor)
            status = [(datetime.fromisoformat(row["recorded_at"]).timestamp(), row) for row in telemetry
                      if bounds[0] - 1 <= datetime.fromisoformat(row["recorded_at"]).timestamp() <= bounds[1] + 1]
            motor_times = [t for t, row in status if telemetry_motor_on(row)]
            commanded_times = [t for t, row in status if row["command"].get("enable") and
                               (row["command"].get("left_rpm") or row["command"].get("right_rpm"))]
            if label == "motor":
                # Avoid edges at which the command or wheel feedback has just changed.
                intervals = [(a + .05, b + .05) for a, b in contiguous_intervals(motor_times) if b - a >= .55]
            else:
                intervals = [(bounds[0], bounds[1])]
                # Exclude the startup motor commands, including their transport lag.
                for a, b in contiguous_intervals(commanded_times):
                    prohibited = (a - .5, b + .5)
                    intervals = [(l, min(h, prohibited[0])) for l, h in intervals if l < prohibited[0]] + [
                        (max(l, prohibited[1]), h) for l, h in intervals if h > prohibited[1]]
            clips = []
            for start_wall, end_wall in intervals:
                first_sample = round((start_wall - anchor) * RATE)
                end_sample = round((end_wall - anchor) * RATE)
                for first, raw in s["runs"]:
                    intersected = intersect_run(first, raw, first_sample, end_sample)
                    if intersected is None:
                        continue
                    left, right = intersected
                    clips.append({"start_s": left / RATE, "end_s": right / RATE,
                                  "session_index": s["index"], "label": label,
                                  "session": f"{label}-{path.stem}-boot{s['index']}"})
            spans.extend(clips)
            session_report.append({"index": s["index"], "pcm_blocks": s["stats"]["blocks"],
                                   "missing_samples_between_runs": s["stats"]["missing_samples_between_runs"],
                                   "actual_motor_telemetry_rows": len(motor_times),
                                   "clips": len(clips), "seconds": round(sum(
                                       c["end_s"] - c["start_s"] for c in clips), 2)})
        annotations[path.name] = spans
        report["sessions"][path.name] = session_report
        if not spans:
            raise ValueError(f"no verifiable {label} PCM interval in {path}")
    output.mkdir(parents=True, exist_ok=True)
    (output / "annotations.json").write_text(json.dumps(annotations, indent=2) + "\n", encoding="utf-8")
    report["dataset"] = convert([motor_log, target_log], output / "wav", annotations)
    (output / "report.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"session_summary": report["sessions"], "clips": len(report["dataset"]["clips"])}))
    return report


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--motor-log", required=True, type=Path)
    parser.add_argument("--target-log", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    build(args.motor_log, args.target_log, args.output)
