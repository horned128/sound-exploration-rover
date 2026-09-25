"""Align user-provided wall-clock intervals with ESP sample indices and audit this session.

Run: python3 software/audio_ml/field_session.py LOG --output DIR
The positive intervals mean 'TARGET may be sounding', NOT per-event ground truth.
"""
from __future__ import annotations

import argparse
import bisect
import json
import statistics
import math
import wave
from array import array
from collections import Counter
from datetime import datetime
from pathlib import Path

from pcm_dataset import RATE, convert, pcm_runs

# Observer's local times, September 25 2026. Intervals are deliberately separated
# by gaps; do not label unobserved time as either positive or negative.
INTERVALS = (
    ("quiet", "00:45:50", "00:46:30", "background"),
    ("tv_only", "00:46:40", "00:47:05", "tv"),
    ("speech_only", "00:47:30", "00:48:00", "speech"),
    ("target_motor", "00:48:00", "00:49:00", "target_possible"),
    ("target_tv_motor", "00:49:00", "00:51:00", "target_possible"),
)


def run(path: Path, output: Path) -> dict:
    with path.open(encoding="utf-8") as source:
        records = [json.loads(line) for line in source if line.strip()]
    pcm = [r for r in records if r.get("record_type") == "acoustic_pcm"]
    if not pcm:
        raise ValueError(f"no acoustic_pcm in {path}")
    # Compare the host arrival time to ESP uptime and then to its audio clock.
    # Minimum observed transport offset avoids converting startup queue backlog
    # into a fake time shift. Report residual spread, rather than claiming sync.
    host_offsets = [(datetime.fromisoformat(r["recorded_at"]).timestamp() - r["esp_ms"] / 1000)
                    for r in pcm]
    audio_offsets = [r["esp_ms"] / 1000 - r["first_sample"] / RATE for r in pcm]
    clock = statistics.median(host_offsets) + statistics.median(audio_offsets)
    offsets = [datetime.fromtimestamp(clock, datetime.fromisoformat(pcm[0]["recorded_at"]).tzinfo)]
    runs, transport = pcm_runs(path)
    telemetry = [r for r in records if "command" in r and "esp_ms" in r]
    telemetry_times = [r["esp_ms"] for r in telemetry]
    annotations = []
    metrics = {}
    for name, begin, end, label in INTERVALS:
        date = offsets[0].date()
        zone = offsets[0].tzinfo
        start_time = datetime.fromisoformat(f"{date}T{begin}").replace(tzinfo=zone).timestamp()
        end_time = datetime.fromisoformat(f"{date}T{end}").replace(tzinfo=zone).timestamp()
        start = max(0, round((start_time - clock) * RATE))
        stop = max(0, round((end_time - clock) * RATE))
        # Crop to contiguous run(s); a run fragment < 1 second is unusable.
        segments = []
        for first, raw in runs:
            left = max(first, start)
            right = min(first + len(raw) // 2, stop)
            if right - left >= RATE:
                segments.append({"start_s": left / RATE, "end_s": right / RATE,
                                 "label": label, "session": name})
        annotations.extend(segments)
        diagnostic = [r for r in records if r.get("record_type") == "acoustic_diagnostic"
                      and start_time <= datetime.fromisoformat(r["recorded_at"]).timestamp() < end_time]
        telemetry_interval = [r for r in telemetry if start_time <=
                              datetime.fromisoformat(r["recorded_at"]).timestamp() < end_time]
        status = Counter(r.get("identifier_status_name", "unknown") for r in diagnostic)
        moving = 0
        for r in diagnostic:
            ix = bisect.bisect_right(telemetry_times, r["esp_ms"]) - 1
            if ix >= 0 and r["esp_ms"] - telemetry_times[ix] < 500:
                command = telemetry[ix]["command"]
                moving += bool(command.get("enable") and (command.get("left_rpm") or command.get("right_rpm")))
        choices = {}
        for threshold in (0.055, 0.08, 0.10):
            for peak_gate in (True, False):
                choices[f"distance_{threshold:.3f}_peak_{peak_gate}"] = sum(
                    r.get("summary_valid") and 0 <= r.get("distance", -1) <= threshold
                    and (not peak_gate or (r.get("current_peak_bin", -1) >= 0 and
                         r.get("target_peak_bin", -1) >= 0 and
                         abs(r["current_peak_bin"] - r["target_peak_bin"]) <= 6))
                    for r in diagnostic)
        metrics[name] = {"span_s": round((stop - start) / RATE, 2),
                         "pcm_segments": len(segments),
                         "pcm_s": round(sum(s["end_s"] - s["start_s"] for s in segments), 2),
                         "diagnostic_status": dict(status),
                         "diagnostic_events": len(diagnostic), "diagnostics_while_moving": moving,
                         "think_states": dict(Counter(r.get("think", {}).get("name") for r in telemetry_interval)),
                         "link_ready_rows": sum(r.get("think", {}).get("link_ready") == 1 for r in telemetry_interval),
                         "clearance_rows_380_550_mm": sum(
                             len(r.get("sensors", {}).get("tof_mm", [])) == 3 and
                             r["sensors"]["tof_mm"][0] >= 380 and
                             r["sensors"]["tof_mm"][1] >= 550 and
                             r["sensors"]["tof_mm"][2] >= 380 for r in telemetry_interval),
                         "candidate_accepts_unverified": choices}
    output.mkdir(parents=True, exist_ok=True)
    annotated = {path.name: annotations}
    (output / "annotations.json").write_text(json.dumps(annotated, indent=2) + "\n", encoding="utf-8")
    manifest = convert([path], output / "wav", annotated)
    for clip in manifest["clips"]:
        name = clip["session"]
        with wave.open(str(output / "wav" / clip["wav"]), "rb") as audio:
            samples = array("h", audio.readframes(audio.getnframes()))
            if samples:
                rms = math.sqrt(sum(x * x for x in samples) / len(samples)) / 32768
                metrics[name].setdefault("clip_quality", []).append({
                    "rms_dbfs": round(20 * math.log10(max(rms, 1e-12)), 1),
                    "clipped_samples": sum(abs(x) >= 32760 for x in samples)})
    report = {"log": path.name, "clock_anchor": offsets[0].isoformat(),
              "clock_offset_spread_p95_ms": round(sorted(abs(x - statistics.median(host_offsets))
                                                   for x in host_offsets)[int(.95 * (len(host_offsets)-1))] * 1000),
              "audio_offset_spread_p95_ms": round(sorted(abs(x - statistics.median(audio_offsets))
                                                    for x in audio_offsets)[int(.95 * (len(audio_offsets)-1))] * 1000),
              "pcm_transport": transport, "intervals": metrics,
              "warning": "target_possible intervals contain unannotated silence; no frame-level recall/precision"}
    (output / "field_report.json").write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n",
                                               encoding="utf-8")
    print(json.dumps({"pcm_clips": len(manifest["clips"]), "intervals": metrics}, ensure_ascii=False))
    return report


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    run(args.log, args.output)
