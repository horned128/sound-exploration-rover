"""Fetch bounded ESC-50 general events for unseen-class few-shot evaluation.

Audio and metadata stay outside git. ESC-50 is CC BY-NC 3.0; see upstream
LICENSE and recording-specific attribution before redistributing any WAV.
"""
from __future__ import annotations

import argparse
import csv
import io
import json
import sys
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "software/audio_ml"))
from fetch_esc10 import BASE, retrieve  # noqa: E402

TRAIN = ("chainsaw", "clock_tick", "crackling_fire", "crying_baby", "dog", "helicopter",
         "rain", "rooster", "sea_waves", "sneezing", "engine", "washing_machine",
         "footsteps", "fireworks", "wind", "breathing", "keyboard_typing", "siren")
VALID = ("mouse_click", "car_horn", "water_drops")
TEST = ("clapping", "church_bells", "glass_breaking")


def fetch(output: Path, per_class: int = 12, all_remaining_train: bool = False) -> dict:
    metadata = list(csv.DictReader(io.StringIO(retrieve(BASE + "/meta/esc50.csv").decode("utf-8"))))
    additional = sorted({r["category"] for r in metadata} - set((*TRAIN, *VALID, *TEST))) \
        if all_remaining_train else []
    train = (*TRAIN, *additional)
    selected = []
    for category in (*train, *VALID, *TEST):
        rows = sorted((row for row in metadata if row["category"] == category),
                      key=lambda r: (int(r["fold"]), r["filename"]))
        support = [r for r in rows if int(r["fold"]) <= 2][:5]
        heldout = [r for r in rows if int(r["fold"]) >= 3][:per_class-5]
        if len(support) != 5 or len(heldout) != per_class - 5:
            raise ValueError(f"not enough clips for {category}")
        selected.extend({"file": row["filename"], "class": category, "fold": int(row["fold"]),
                         "split": split} for split, examples in (("support", support), ("test", heldout))
                        for row in examples)
    output.mkdir(parents=True, exist_ok=True)
    def save(row: dict) -> None:
        file = output / row["file"]
        if not file.exists():
            previous = output.parent / "esc50-selected" / row["file"]
            if previous.is_file():
                try:
                    file.hardlink_to(previous)
                    return
                except OSError:
                    pass
            content = retrieve(BASE + "/audio/" + row["file"])
            if not content.startswith(b"RIFF"):
                raise ValueError(row["file"])
            file.write_bytes(content)
    with ThreadPoolExecutor(max_workers=8) as pool:
        list(pool.map(save, selected))
    result = {"source": BASE, "license": "CC BY-NC 3.0 (ESC-50 upstream LICENSE)",
              "train_classes": list(train), "validation_classes": list(VALID),
              "test_classes": list(TEST), "classes": list((*train, *VALID, *TEST)), "clips": selected}
    (output / "manifest.json").write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--all-remaining-train", action="store_true",
                        help="use every ESC-50 class except the fixed held-out 3+3")
    args = parser.parse_args()
    result = fetch(args.output, all_remaining_train=args.all_remaining_train)
    print(json.dumps({"train_classes": len(result["train_classes"]),
                      "classes": len(result["classes"]), "clips": len(result["clips"])}))
