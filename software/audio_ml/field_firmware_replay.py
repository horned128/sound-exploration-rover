"""Replay the *compiled CPU0 C classifier* on a labeled field JSONL session.

The labels identify sound conditions, not per-frame presence of intermittent TARGET.
"""
from __future__ import annotations

import argparse
import ctypes
import csv
import json
import sys
from collections import Counter
from datetime import datetime
from pathlib import Path

from field_session import INTERVALS
from legacy_audit import ROOT

sys.path.insert(0, str(ROOT / "software/control-sim"))
from controlsim.bindings import AcousticIdentifierSummaryOutput, library  # noqa: E402


def replay(log: Path, output: Path) -> dict:
    with log.open(encoding="utf-8") as f:
        data = [json.loads(line) for line in f if line.strip()]
    profiles: dict[int, dict[int, list[int]]] = {}
    for r in data:
        if r.get("record_type") == "acoustic_sample" and len(r.get("summary", [])) == 192:
            profiles.setdefault(r["profile_generation"], {})[r["sample_index"]] = r["summary"]
    if len(profiles) != 1 or set(next(iter(profiles.values()))) != set(range(5)):
        raise ValueError("single complete field profile required")
    samples = next(iter(profiles.values()))
    saved = (ctypes.c_int8 * 960)(*(v for i in range(5) for v in samples[i]))
    handle = library()
    mask = int(handle.acoustic_identifier_consensus_mask(saved, 5))
    groups: dict[str, Counter] = {name: Counter() for name, _, _, _ in INTERVALS}
    groups["unlabeled"] = Counter()
    rows = []
    for r in data:
        if r.get("record_type") != "acoustic_diagnostic" or len(r.get("summary", [])) != 192:
            continue
        time = datetime.fromisoformat(r["recorded_at"]).strftime("%H:%M:%S")
        group = next((name for name, start, end, _ in INTERVALS if start <= time < end), "unlabeled")
        if not r.get("summary_valid"):
            groups[group]["indeterminate"] += 1
            continue
        candidate = (ctypes.c_int8 * 192)(*r["summary"])
        result = AcousticIdentifierSummaryOutput()
        # The firmware clamps the saved threshold to 0.055 for acceptance. The
        # field snapshot records the effective value; no re-tuning on test data.
        handle.acoustic_identifier_summary_classify(
            candidate, True, 80, saved, 5, None, ctypes.c_float(r.get("threshold", .055)), ctypes.byref(result)
        )
        old = r["identifier_status"] == 4
        new = result.status == 4
        groups[group]["original_TARGET" if old else "original_OTHER"] += 1
        groups[group]["replayed_TARGET" if new else "replayed_OTHER"] += 1
        if old != new:
            groups[group]["changed"] += 1
        rows.append({"time": r["recorded_at"], "group": group, "generation": r["feature_generation"],
                     "old_target": old, "new_target": new, "new_nearest_sample": result.sample_index,
                     "old_distance": r["distance"],
                     "new_distance": round(float(result.minimum_cosine_distance), 5)})
    report = {"log": log.name, "retained_sample_mask": mask,
              "event_groups": {name: dict(counts) for name, counts in groups.items()},
              "truth_warning": "TARGET is intermittent in mixed intervals; other detections there are not verified false negatives"}
    output.mkdir(parents=True, exist_ok=True)
    with (output / "events.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    (output / "replay.json").write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    return report


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(replay(args.log, args.output), indent=2))
