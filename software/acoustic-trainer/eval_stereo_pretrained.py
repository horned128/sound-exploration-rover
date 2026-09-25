"""Replay a frozen TARGET-independent query model separately on each I2S slot.

Threshold=0.95 and 2-of-3 vote were selected on *other* ESC-50 classes;
no fitting on these field logs. A detected session is not event-level recall.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import tensorflow as tf

from eval_query_stream import query_five, scores, predict_events
from eval_stereo_query import sounds


def evaluate(folders: dict[str, Path], model_path: Path, output: Path, threshold: float = .95) -> dict:
    model = tf.keras.models.load_model(model_path, compile=False)
    result = {"model": model_path.name, "threshold": threshold,
              "source_of_threshold": "ESC-50 validation classes, never tuned to field stereo",
              "components": {}}
    for component in ("left", "right", "mid", "side"):
        targets = sounds(folders["target"], component)
        longest = max(targets, key=len)
        query = query_five(longest[:len(longest)//2], "absolute")
        if query is None:
            result["components"][component] = {"error": "five valid enrollment windows unavailable"}
            continue
        groups = {}
        for group in ("target", "target_noise", "motor", "silence"):
            records = sounds(folders[group], component)
            decisions = [predict_events(scores(model, query, pcm, "absolute")[0], threshold, 2)
                         for pcm in records]
            groups[group] = {"recordings": len(records), "recordings_with_match": sum(bool(x.any()) for x in decisions),
                             "positive_updates": sum(int(x.sum()) for x in decisions),
                             "updates": sum(len(x) for x in decisions)}
        result["components"][component] = groups
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2))
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    for group in ("silence", "target", "target_noise", "motor"):
        parser.add_argument("--" + group.replace("_", "-"), required=True, type=Path)
    parser.add_argument("--model", required=True, type=Path)
    parser.add_argument("--threshold", type=float, default=.95,
                        help="threshold selected exclusively on general-audio validation")
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    evaluate({group: getattr(args, group) for group in
              ("silence", "target", "target_noise", "motor")}, args.model, args.output, args.threshold)
