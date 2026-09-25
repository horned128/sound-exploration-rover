"""Inventory PCM sessions and detect recorder restarts before labeling audio."""
from __future__ import annotations

import argparse
import base64
import json
import struct
from collections import Counter
from pathlib import Path


def inventory(path: Path) -> dict:
    segments: list[dict] = []
    current = None
    prior_ms = None
    prior_sample = None
    types = Counter()
    telemetry = Counter()
    with path.open(encoding="utf-8") as stream:
        for line_no, line in enumerate(stream, 1):
            row = json.loads(line)
            kind = row.get("record_type", "telemetry")
            types[kind] += 1
            if kind != "acoustic_pcm":
                if "command" in row:
                    cmd = row["command"]
                    if row.get("debug_motor", {}).get("active"):
                        telemetry["debug_active"] += 1
                    if cmd.get("enable") and cmd.get("left_rpm") and cmd.get("right_rpm"):
                        telemetry["command_moving"] += 1
                    if row.get("actuator", {}).get("valid") and (
                        abs(row["actuator"].get("left_encoder_rpm_x10", 0)) >= 100 or
                        abs(row["actuator"].get("right_encoder_rpm_x10", 0)) >= 100
                    ):
                        telemetry["motor_measured"] += 1
                    telemetry["think_" + row.get("think", {}).get("name", "unknown")] += 1
                continue
            ms = int(row["esp_ms"])
            sample = int(row["first_sample"])
            reset = current is None or (prior_ms is not None and ms + 1000 < prior_ms) or (
                prior_sample is not None and sample + 16000 < prior_sample)
            if reset:
                current = {"first_recorded_at": row["recorded_at"], "first_line": line_no,
                           "first_esp_ms": ms, "first_sample": sample, "blocks": 0,
                           "last_recorded_at": row["recorded_at"], "last_sample": sample,
                           "clipped_samples": 0, "peak": 0}
                segments.append(current)
            raw = base64.b64decode(row["pcm16le_b64"], validate=True)
            pcm = struct.unpack("<" + "h" * row["sample_count"], raw)
            current["blocks"] += 1
            current["clipped_samples"] += sum(abs(v) >= 32760 for v in pcm)
            current["peak"] = max(current["peak"], max(abs(v) for v in pcm))
            current["last_recorded_at"] = row["recorded_at"]
            current["last_sample"] = sample + row["sample_count"]
            prior_ms, prior_sample = ms, sample
    return {"file": path.name, "kinds": dict(types), "telemetry": dict(telemetry), "sessions": segments}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("logs", nargs="+", type=Path)
    args = parser.parse_args()
    for file in args.logs:
        print(json.dumps(inventory(file), indent=2, ensure_ascii=False))
