"""Download a bounded, reproducible ESC-10 class-disjoint few-shot evaluation set.

ESC-10 files are CC BY 3.0; audio is kept in ignored dataset/ (not committed).
Select 5 enrollment clips (folds 1/2) and 7 held-out clips (folds 3/4/5)
per class. Labels are independent of the rover's field TARGET identity.
"""
from __future__ import annotations

import argparse
import csv
import io
import json
import urllib.request
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

COMMIT = "33c8ce9eb2cf0b1c2f8bcf322eb349b6be34dbb6"
BASE = f"https://raw.githubusercontent.com/karolpiczak/ESC-50/{COMMIT}"


def retrieve(url: str) -> bytes:
    request = urllib.request.Request(url, headers={"User-Agent": "SoundExplorationRover-audio-evaluation"})
    with urllib.request.urlopen(request, timeout=30) as response:
        return response.read()


def fetch(output: Path, support: int = 5, test: int = 7) -> dict:
    output.mkdir(parents=True, exist_ok=True)
    metadata = retrieve(BASE + "/meta/esc50.csv").decode("utf-8")
    rows = list(csv.DictReader(io.StringIO(metadata)))
    categories: dict[str, list[dict]] = {}
    for row in rows:
        if row["esc10"] == "True":
            categories.setdefault(row["category"], []).append(row)
    clips = []
    for category in sorted(categories):
        examples = sorted(categories[category], key=lambda r: (int(r["fold"]), r["filename"]))
        enrollers = [r for r in examples if int(r["fold"]) <= 2][:support]
        heldout = [r for r in examples if int(r["fold"]) >= 3][:test]
        if len(enrollers) != support or len(heldout) != test:
            raise ValueError(f"insufficient ESC-10 examples for {category}")
        clips.extend({"file": r["filename"], "class": category,
                      "fold": int(r["fold"]), "split": split}
                     for split, selected in (("support", enrollers), ("test", heldout)) for r in selected)

    def save(row: dict) -> None:
        path = output / row["file"]
        if not path.exists():
            payload = retrieve(BASE + "/audio/" + row["file"])
            if not payload.startswith(b"RIFF"):
                raise ValueError(f"invalid WAV from upstream: {row['file']}")
            path.write_bytes(payload)

    with ThreadPoolExecutor(max_workers=8) as workers:
        list(workers.map(save, clips))
    report = {"source": BASE, "license": "ESC-10 subset CC BY 3.0; see https://github.com/karoldvl/ESC-50/blob/master/LICENSE",
              "classes": sorted(categories), "clips": clips}
    (output / "manifest.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    return report


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    result = fetch(args.output)
    print(json.dumps({"classes": len(result["classes"]), "clips": len(result["clips"])}))
