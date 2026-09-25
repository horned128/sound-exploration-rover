"""Audit recorded 192D events without treating legacy predictions as ground truth.

Usage: python3 software/audio_ml/legacy_audit.py --output /tmp/audio-audit
Outputs a per-event CSV and a machine-readable summary. No NumPy dependency.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
from collections import Counter, defaultdict
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
LOGS = ROOT / "software/rover-monitor/logs"
FIELDNAMES = ["file", "esp_ms", "generation", "legacy_status", "distance", "threshold",
              "peak", "target_peak", "moving_proxy", "profile_generation", "recomputed_distance",
              "recomputed_peak", "legacy_reproduced", "no_peak_at_055", "no_peak_at_080"]


def peak(summary: list[int]) -> int:
    return max(range(32), key=lambda b: max(summary[64 + b], summary[160 + b]))


def weights(bin_index: int) -> list[float]:
    result = [0.5 if b < 3 else 1.0 for b in range(32)]
    for offset, value in ((-2, 1.3), (-1, 1.8), (0, 2.5), (1, 1.8), (2, 1.3)):
        if 0 <= bin_index + offset < 32:
            result[bin_index + offset] = value
    return result


def distance(left: list[int], right: list[int], bin_weights: list[float]) -> float:
    dot = ll = rr = 0.0
    for index, (l, r) in enumerate(zip(left, right)):
        w = bin_weights[index % 32]
        dot += w * l * r
        ll += w * l * l
        rr += w * r * r
    return 1.0 - dot / math.sqrt(ll * rr) if ll and rr else float("inf")


def consensus(samples: list[list[int]]) -> tuple[int, list[list[int]]]:
    """Mirror the firmware's 4-of-5 outlier and weighted peak-bin logic."""
    peaks = [peak(sample) for sample in samples]
    excluded = None
    if len(samples) == 5:
        for candidate in range(5):
            keep = [i for i in range(5) if i != candidate]
            w = weights(peaks[keep[0]])
            if all(abs(peaks[i] - peaks[j]) <= 6 and
                   distance(samples[i], samples[j], w) <= 0.25
                   for n, i in enumerate(keep) for j in keep[n + 1:]) and all(
                       distance(samples[candidate], samples[i], w) > 0.25 for i in keep):
                excluded = candidate
                break
    kept = [sample for i, sample in enumerate(samples) if i != excluded]
    bin_index = max(range(32), key=lambda b: sum(max(s[64 + b], s[160 + b]) for s in kept))
    return bin_index, kept


def load_log(path: Path) -> list[dict]:
    with path.open(encoding="utf-8") as stream:
        return [json.loads(line) for line in stream if line.strip()]


def audit(paths: list[Path]) -> tuple[list[dict], dict]:
    events = []
    files = {}
    for path in paths:
        records = load_log(path)
        profiles: dict[int, dict[int, list[int]]] = defaultdict(dict)
        for r in records:
            if r.get("record_type") == "acoustic_sample" and len(r.get("summary", [])) == 192:
                profiles[r["profile_generation"]][r["sample_index"]] = r["summary"]
        complete = {g: [members[i] for i in range(5)] for g, members in profiles.items()
                    if set(members) == set(range(5))}
        profile_generation = next(iter(complete)) if len(complete) == 1 else None
        matching = complete.get(profile_generation, []) if profile_generation is not None else []
        target_peak, kept = consensus(matching) if matching else (None, [])
        w = weights(target_peak) if target_peak is not None else []
        telemetry = [r for r in records if "command" in r and "esp_ms" in r]
        t_index = 0
        counters = Counter()
        for r in records:
            kind = r.get("record_type")
            if kind in ("acoustic_diagnostic", "acoustic_sample", "acoustic_pcm"):
                counters[kind] += 1
            if kind != "acoustic_diagnostic" or len(r.get("summary", [])) != 192:
                continue
            while t_index + 1 < len(telemetry) and telemetry[t_index + 1]["esp_ms"] <= r["esp_ms"]:
                t_index += 1
            nearby = telemetry[t_index] if telemetry and abs(telemetry[t_index]["esp_ms"] - r["esp_ms"]) < 500 else None
            moving = bool(nearby and nearby["command"]["enable"] and
                          (nearby["command"]["left_rpm"] or nearby["command"]["right_rpm"]))
            valid = bool(r.get("summary_valid"))
            actual_peak = peak(r["summary"]) if valid else None
            d = min((distance(r["summary"], s, w) for s in kept), default=float("inf")) if valid else float("inf")
            reproduced = (d <= min(r.get("threshold", 0), 0.055) and
                          abs(actual_peak - target_peak) <= 6) if matching and valid else None
            legacy = r.get("identifier_status")
            if legacy in (3, 4):
                counters[f"legacy_{legacy}"] += 1
                if moving:
                    counters[f"moving_{legacy}"] += 1
                if reproduced is not None:
                    counters["replay_count"] += 1
                    counters["replay_agree"] += (reproduced == (legacy == 4))
                if legacy == 3 and r.get("distance", 1) > 0.055 and r.get("current_peak_bin") == r.get("target_peak_bin"):
                    counters["same_peak_reject"] += 1
            events.append(dict(zip(FIELDNAMES, [path.name, r["esp_ms"], r["feature_generation"], legacy,
                          r.get("distance"), r.get("threshold"), r.get("current_peak_bin"),
                          r.get("target_peak_bin"), moving, profile_generation,
                          round(d, 5) if math.isfinite(d) else None, actual_peak, reproduced,
                          d <= 0.055 if matching else None, d <= 0.08 if matching else None])))
            if matching and valid and legacy in (3, 4):
                if d <= 0.055 and legacy == 3:
                    counters["no_peak_new_hits_055_unverified"] += 1
                if d <= 0.08 and legacy == 3:
                    counters["no_peak_new_hits_080_unverified"] += 1
        files[path.name] = {"records": len(records), "profile_generations": sorted(profiles),
                            "complete_profiles": sorted(complete), "counters": dict(counters),
                            "duration_s": round((telemetry[-1]["esp_ms"] - telemetry[0]["esp_ms"]) / 1000, 2)
                            if len(telemetry) >= 2 else None}
    return events, {"files": files, "event_rows": len(events),
                    "ground_truth": "unavailable: legacy status and motor commands are not sound-presence labels",
                    "measured_recall_precision_fp_per_hour": None}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("logs", nargs="*", type=Path)
    args = parser.parse_args()
    paths = args.logs or sorted(p for p in LOGS.glob("*.jsonl") if not p.is_symlink())
    events, report = audit(paths)
    args.output.mkdir(parents=True, exist_ok=True)
    with (args.output / "events.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=FIELDNAMES)
        writer.writeheader()
        writer.writerows(events)
    (args.output / "audit.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"event_rows": report["event_rows"], "files": len(report["files"])}))


if __name__ == "__main__":
    main()
