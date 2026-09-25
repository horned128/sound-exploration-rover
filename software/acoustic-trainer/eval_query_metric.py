"""Compare mixture-trained CNN embeddings as few-shot source-presence detectors.

Unknown ESC-50 source excerpts (val/test classes excluded from NN training) are
registered five times; test positives inject that exact source into independent
motor, TV and general audio. Negatives include same-class OTHER source clips.
"""
from __future__ import annotations

import argparse
import json
import wave
from collections import defaultdict
from pathlib import Path

import numpy as np
import tensorflow as tf

from esc10_fewshot_eval import read_resampled
from eval_fewshot import embed, mel_inputs, vote, episodes
from eval_query_mixture import load_field
from train_fewshot import mix


def eval_models(manifest: Path, motor_manifest: Path, tv_manifest: Path, model_dir: Path, output: Path) -> dict:
    data = json.loads(manifest.read_text(encoding="utf-8"))
    cases = [r for r in data["clips"] if r["class"] in
             (*data["validation_classes"], *data["test_classes"]) and r["split"] == "test"]
    audio = {r["file"]: read_resampled(manifest.parent / r["file"])
             for r in data["clips"] if r["split"] == "test"}
    motor = load_field(motor_manifest, "motor")
    tv = load_field(tv_manifest, "tv")
    rng = np.random.default_rng(20260925)
    result = {"upstream": data["source"], "target_training": False, "models": []}
    for model_path in sorted(model_dir.glob("metric_*.keras")):
        model = tf.keras.models.load_model(model_path, compile=False)
        negatives = {name: (embed(model, inputs), valid) for name, pcm in audio.items()
                     for inputs, valid in [mel_inputs(pcm)]}
        queries = []
        for cohort_name, cohort in (("validation", data["validation_classes"]),
                                    ("test", data["test_classes"])):
            for row in cases:
                if row["class"] not in cohort:
                    continue
                source = audio[row["file"]]
                frames = mel_inputs(source)[0]
                if len(frames) < 6:
                    continue
                # A snippet with ample active frontend frames is the unknown query.
                original_valid = mel_inputs(source)[1]
                if not original_valid.any():
                    continue
                source_position = int(np.flatnonzero(original_valid)[len(np.flatnonzero(original_valid)) // 2]) * 1280
                snippet = source[source_position:source_position + 13040]
                if len(snippet) != 13040:
                    continue
                support = []
                for shift in (0, 80, -80, 40, -40):
                    repeat = np.roll(snippet, shift).copy()
                    if shift > 0:
                        repeat[:shift] = 0
                    if shift < 0:
                        repeat[shift:] = 0
                    patches, usable = mel_inputs(repeat)
                    if not usable.any():
                        break
                    support.append(patches[int(np.flatnonzero(usable)[len(np.flatnonzero(usable)) // 2])])
                if len(support) != 5:
                    continue
                query = embed(model, np.stack(support))
                same = [r["file"] for r in cases if r["class"] == row["class"] and r["file"] != row["file"]]
                different = sorted(r["file"] for r in data["clips"] if r["split"] == "test" and r["class"] != row["class"])
                exposed = same + list(rng.choice(different, size=12, replace=False))
                backgrounds = [motor, tv, audio[different[int(rng.integers(len(different)))]]]
                mixtures: dict[int, list[tuple[np.ndarray, np.ndarray]]] = defaultdict(list)
                for background in backgrounds:
                    for snr in (-6, 0, 6):
                        start = int(rng.integers(0, len(background)-len(snippet)))
                        bg = background[start:start+len(snippet)]
                        mixed = mix(snippet, bg, snr)
                        pcm = np.r_[bg[:3200], mixed, bg[-3200:]].astype(np.int16)
                        x, valid = mel_inputs(pcm)
                        mixtures[snr].append((embed(model, x), valid))
                clean_x, clean_valid = mel_inputs(np.r_[np.zeros(3200, dtype=np.int16), snippet,
                                                      np.zeros(3200, dtype=np.int16)])
                clean_emb = embed(model, clean_x)
                clean_distance = float(np.min(np.where(clean_valid,
                    np.min(1 - clean_emb @ query.T, axis=1), 2.0)))
                queries.append({"cohort": cohort_name, "source": row["file"], "class": row["class"],
                                "query": query, "negatives": exposed, "positives": mixtures,
                                "clean_min_distance": round(clean_distance, 4)})
        model_summary = {"model": model_path.name, "parameters": model.count_params(),
                         "queries": len(queries),
                         "clean_min_distance_median": float(np.median([q["clean_min_distance"] for q in queries])),
                         "grid": []}
        for cohort_name in ("validation", "test"):
            for mode in ("nearest", "centroid"):
                for threshold in (.1, .2, .3, .4, .5, .6):
                    for required in (2, 3):
                        neg_seconds = fp = 0
                        positive = {str(snr): {"hits": 0, "total": 0} for snr in (-6, 0, 6)}
                        for item in queries:
                            if item["cohort"] != cohort_name:
                                continue
                            query = item["query"]
                            if mode == "centroid":
                                query = query.mean(axis=0, keepdims=True)
                                query /= np.maximum(np.linalg.norm(query, axis=1, keepdims=True), 1e-8)
                            def detect(pair: tuple[np.ndarray, np.ndarray]) -> np.ndarray:
                                emb, valid = pair
                                score = np.where(valid, np.min(1 - emb @ query.T, axis=1), 2.0)
                                return vote(score, threshold, required)
                            for neg in item["negatives"]:
                                fp += episodes(detect(negatives[neg]))
                                neg_seconds += len(audio[neg]) / 16000
                            for snr, pairs in item["positives"].items():
                                for pair in pairs:
                                    positive[str(snr)]["hits"] += bool(detect(pair).any())
                                    positive[str(snr)]["total"] += 1
                        model_summary["grid"].append({"cohort": cohort_name, "mode": mode,
                            "threshold": threshold, "vote": required, "false_episodes": fp,
                            "fp_per_hour": round(fp * 3600 / neg_seconds, 2) if neg_seconds else None,
                            "positive_by_snr": positive})
        result["models"].append(model_summary)
        print(json.dumps({"model": model_path.name, "queries": len(queries),
                          "best_validation": max((r for r in model_summary["grid"] if r["cohort"] == "validation"
                              and r["fp_per_hour"] is not None and r["fp_per_hour"] <= 10),
                              key=lambda r: r["positive_by_snr"]["0"]["hits"], default=None)}))
    output.mkdir(parents=True, exist_ok=True)
    (output / "query_metric.json").write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--esc", required=True, type=Path)
    parser.add_argument("--motor", required=True, type=Path)
    parser.add_argument("--tv", required=True, type=Path)
    parser.add_argument("--models", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    eval_models(args.esc, args.motor, args.tv, args.models, args.output)
