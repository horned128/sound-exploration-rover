"""Evaluate frozen mixture-trained embeddings on unseen ESC classes and rover PCM.

Threshold and metric search uses only ESC-10 validation classes (6,7). Held-out
classes (8,9) and the rover field recordings are reported separately. TARGET
identity is never an offline training class. FP/hour counts distinct alarm rises.
"""
from __future__ import annotations

import argparse
import json
import sys
import wave
from collections import defaultdict
from pathlib import Path

import numpy as np
import tensorflow as tf

from esc10_fewshot_eval import read_resampled
from train_fewshot import fast_frontend

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "software/audio_ml"))
from short_window_probe import frontend_mel  # noqa: E402


def mel_inputs(pcm: np.ndarray, *, step: int = 8, limit: int = 40,
               frontend: str = "centered") -> tuple[np.ndarray, np.ndarray]:
    if frontend == "absolute":
        from train_fewshot import fast_frontend
        mel = fast_frontend(pcm, "absolute")
        raw = pcm.astype(np.float32)
        frames = np.lib.stride_tricks.sliding_window_view(raw, 400)[::160]
        active = np.sqrt(np.mean(frames * frames, axis=1)) > 75
    else:
        mel = frontend_mel(pcm)
        active = np.std(mel, axis=1) > 12
    frames, valid = [], []
    for start in range(0, len(mel) - limit + 1, step):
        patch = mel[start:start+limit]
        frames.append(patch[..., None] / 128.0)
        valid.append(active[start:start+limit].sum() >= max(5, limit // 4))
    return np.array(frames, dtype=np.float32), np.array(valid, dtype=bool)


def embed(model: tf.keras.Model, x: np.ndarray) -> np.ndarray:
    if not len(x):
        return np.empty((0, int(model.output_shape[-1])), dtype=np.float32)
    values = np.concatenate([model(batch, training=False).numpy() for batch in np.array_split(x, max(1, len(x) // 256))])
    return values / np.maximum(np.linalg.norm(values, axis=1, keepdims=True), 1e-8)


def vote(score: np.ndarray, threshold: float, rule: int) -> np.ndarray:
    hit = score < threshold
    width = 3 if rule == 2 else 5
    accepted = np.zeros(len(score), dtype=bool)
    for i in range(width-1, len(score)):
        accepted[i] = hit[i-width+1:i+1].sum() >= rule
    return accepted


def episodes(accepted: np.ndarray) -> int:
    return int(np.count_nonzero(accepted & ~np.r_[False, accepted[:-1]]))


def load_field(manifest: Path) -> list[tuple[dict, np.ndarray]]:
    data = json.loads(manifest.read_text(encoding="utf-8"))
    result = []
    for r in data["clips"]:
        with wave.open(str(manifest.parent / r["wav"]), "rb") as src:
            pcm = np.frombuffer(src.readframes(src.getnframes()), dtype="<i2").copy()
        result.append((r, pcm))
    return result


def register(model: tf.keras.Model, pcm_list: list[np.ndarray]) -> np.ndarray | None:
    queries = []
    for pcm in pcm_list:
        features, valid = mel_inputs(pcm)
        if not valid.any():
            return None
        # Registration exemplars are gathered without knowing the class name.
        i = int(np.flatnonzero(valid)[len(np.flatnonzero(valid)) // 2])
        queries.append(features[i])
    return embed(model, np.stack(queries))


def score(emb: np.ndarray, valid: np.ndarray, query: np.ndarray, mode: str) -> np.ndarray:
    if mode == "centroid":
        prototype = query.mean(axis=0)
        prototype /= max(np.linalg.norm(prototype), 1e-8)
        values = 1 - emb @ prototype
    else:
        values = np.min(1 - emb @ query.T, axis=1)
    return np.where(valid, values, 2.0)


def summary(rows: list[tuple[float, np.ndarray, bool]], threshold: float, required: int) -> dict:
    positive = negative = found = false_episodes = 0
    neg_seconds = 0.0
    for duration, scores, present in rows:
        accepted = vote(scores, threshold, required)
        if present:
            positive += 1
            found += bool(accepted.any())
        else:
            negative += 1
            neg_seconds += duration
            false_episodes += episodes(accepted)
    return {"positive_clips": positive, "detected_clips": found,
            "clip_recall": round(found / positive, 4) if positive else None,
            "negative_clips": negative, "negative_seconds": round(neg_seconds, 2),
            "false_alarm_episodes": false_episodes,
            "fp_per_hour": round(false_episodes * 3600 / neg_seconds, 3) if neg_seconds else None}


def evaluate(esc: Path, field: Path, old: Path, models: Path, output: Path) -> dict:
    esc_data = json.loads(esc.read_text(encoding="utf-8"))
    groups: dict[str, dict[str, list[np.ndarray]]] = defaultdict(lambda: {"support": [], "test": []})
    for row in esc_data["clips"]:
        groups[row["class"]][row["split"]].append(read_resampled(esc.parent / row["file"]))
    field_clips = load_field(field)
    old_clips = load_field(old)
    classes = esc_data["classes"]
    validation = esc_data.get("validation_classes", classes[6:8])
    test = esc_data.get("test_classes", classes[8:10])
    report = {"source": esc_data["source"], "validation": validation, "test": test,
              "field_target_training": False, "models": []}
    for path in sorted(models.glob("metric_*.keras")):
        model = tf.keras.models.load_model(path, compile=False)
        cache = {}
        def prep(pcm: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
            key = id(pcm)
            if key not in cache:
                x, valid = mel_inputs(pcm)
                cache[key] = (embed(model, x), valid)
            return cache[key]

        queries = {category: register(model, groups[category]["support"]) for category in (*validation, *test)}
        # Five short, spaced exemplars from the first half of the new target-only log.
        target_records = [pcm for row, pcm in field_clips if row["label"] == "target_possible"]
        target_long = max(target_records, key=len)
        if len(target_long) < 5 * 6640:
            raise ValueError("five field enrollment windows not available")
        query_pcm = [target_long[int(i):int(i)+6640] for i in np.linspace(0, len(target_long)//2-6640, 5)]
        field_query = register(model, query_pcm)
        if field_query is None:
            raise ValueError("field TARGET registration failed")
        for mode in ("nearest", "centroid"):
            esc_rows = {"validation": [], "test": []}
            for split, cohort in (("validation", validation), ("test", test)):
                for category in cohort:
                    if queries[category] is None:
                        continue
                    for actual in cohort:
                        for pcm in groups[actual]["test"]:
                            emb, valid = prep(pcm)
                            esc_rows[split].append((len(pcm) / 16000,
                                score(emb, valid, queries[category], mode), category == actual))
            field_rows = []
            for row, pcm in field_clips + old_clips:
                emb, valid = prep(pcm)
                if row["label"] in ("background", "tv", "speech", "motor"):
                    field_rows.append((len(pcm) / 16000, score(emb, valid, field_query, mode), False))
                elif row["label"] == "target_possible" and row["log"].endswith("080028.jsonl"):
                    # Query and test from same physical session: only use second half.
                    if len(pcm) >= 2 * 16000:
                        emb = emb[len(emb)//2:]
                        valid = valid[len(valid)//2:]
                        field_rows.append((len(pcm) / 32000, score(emb, valid, field_query, mode), True))
                elif row["label"] == "target_possible":
                    # Mixed real sound: TARGET onset/presence is only weakly known.
                    field_rows.append((len(pcm) / 16000, score(emb, valid, field_query, mode), True))
            candidates = []
            for threshold in (.06, .10, .15, .20, .30, .45, .60):
                for required in (2, 3):
                    candidates.append({"threshold": threshold, "voting": f"{required}-of-{3 if required==2 else 5}",
                        "validation": summary(esc_rows["validation"], threshold, required),
                        "test": summary(esc_rows["test"], threshold, required),
                        "field": summary(field_rows, threshold, required)})
            # Threshold choice uses validation only. No field/held-out test tuning.
            eligible = [c for c in candidates if c["validation"]["fp_per_hour"] <= 10]
            selected = max(eligible, key=lambda c: c["validation"]["clip_recall"]) if eligible else None
            report["models"].append({"model": path.name, "metric": mode,
                                     "selected_by_validation": selected,
                                     "grid": candidates})
            if selected:
                print(json.dumps({"model": path.name, "metric": mode,
                                  "selected_threshold": selected["threshold"], "voting": selected["voting"],
                                  "validation": selected["validation"], "test": selected["test"],
                                  "field": selected["field"]}))
    output.mkdir(parents=True, exist_ok=True)
    (output / "comparison.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    return report


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--esc", type=Path, required=True)
    parser.add_argument("--field", type=Path, required=True)
    parser.add_argument("--old", type=Path, required=True)
    parser.add_argument("--models", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    evaluate(args.esc, args.field, args.old, args.models, args.output)
