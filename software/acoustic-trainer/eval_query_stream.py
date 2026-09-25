"""Streaming query-conditioned CNN replay on unseen ESC sources and rover PCM.

Five on-site query spectrograms are paired with each 400ms audio window. The
threshold and voting policy are selected using only ESC validation sources;
ESC test sources and rover recordings are held out. Real field mixtures are
weakly labeled because TARGET onset times were not supplied.
"""
from __future__ import annotations

import argparse
import json
import wave
from pathlib import Path

import numpy as np
import tensorflow as tf

from esc10_fewshot_eval import read_resampled
from eval_fewshot import mel_inputs, vote, episodes
from eval_query_mixture import load_field
from train_fewshot import mix


def scores(model: tf.keras.Model, queries: np.ndarray, pcm: np.ndarray,
           frontend: str = "centered") -> tuple[np.ndarray, np.ndarray]:
    candidate, valid = mel_inputs(pcm, frontend=frontend)
    if not len(candidate):
        return np.empty((0,)), valid
    predicted = []
    for query in queries:
        pair = np.concatenate((np.broadcast_to(query, candidate.shape), candidate), axis=-1)
        response = np.concatenate([model(batch, training=False).numpy().ravel() for batch in
                                   np.array_split(pair, max(1, len(pair)//256))])
        predicted.append(response)
    stacked = np.sort(np.stack(predicted), axis=0)
    # Require support from at least two independent enrollment examples.
    score = (stacked[-1] + stacked[-2]) * .5
    return np.where(valid, score, 0.0), valid


def query_five(pcm: np.ndarray, frontend: str = "centered") -> np.ndarray | None:
    windows, valid = mel_inputs(pcm, frontend=frontend)
    if valid.sum() < 5:
        return None
    indices = np.flatnonzero(valid)
    positions = np.linspace(0, len(indices)-1, 5).round().astype(int)
    return windows[indices[positions]]


def predict_events(values: np.ndarray, threshold: float, voting: int) -> np.ndarray:
    return vote(threshold - values, 0.0, voting)


def replay(esc: Path, motor_manifest: Path, tv_manifest: Path,
           target_manifest: Path, model_path: Path, output: Path,
           frontend: str = "centered") -> dict:
    meta = json.loads(esc.read_text(encoding="utf-8"))
    cases = [r for r in meta["clips"] if r["class"] in
             (*meta["validation_classes"], *meta["test_classes"]) and r["split"] == "test"]
    audio = {r["file"]: read_resampled(esc.parent / r["file"])
             for r in meta["clips"] if r["split"] == "test"}
    model = tf.keras.models.load_model(model_path, compile=False)
    rng = np.random.default_rng(20260925)
    cohorts = {"validation": {"negative": [], "positive": []},
               "test": {"negative": [], "positive": []}}
    for r in cases:
        cohort = "validation" if r["class"] in meta["validation_classes"] else "test"
        pcm = audio[r["file"]]
        # Clean query source and its physical repetition under gain/time shifts.
        mel, valid = mel_inputs(pcm, frontend=frontend)
        usable = np.flatnonzero(valid)
        if len(usable) < 5:
            continue
        origin = int(usable[len(usable)//2]) * 1280
        snippet = pcm[origin:origin+13040]
        if len(snippet) != 13040:
            continue
        repetitions = []
        for delay in (0, 80, -80, 40, -40):
            repeated = np.roll(snippet, delay).copy()
            if delay > 0:
                repeated[:delay] = 0
            elif delay < 0:
                repeated[delay:] = 0
            x, ok = mel_inputs(repeated, frontend=frontend)
            if not ok.any():
                break
            repetitions.append(x[int(np.flatnonzero(ok)[len(np.flatnonzero(ok))//2])])
        if len(repetitions) != 5:
            continue
        query = np.stack(repetitions)
        same = [x["file"] for x in cases if x["class"] == r["class"] and x["file"] != r["file"]]
        other = sorted(x["file"] for x in meta["clips"] if x["split"] == "test" and x["class"] != r["class"])
        negative_files = same + list(rng.choice(other, 12, replace=False))
        for filename in negative_files:
            cohorts[cohort]["negative"].append((len(audio[filename])/16000,
                                                    scores(model, query, audio[filename], frontend)[0]))
        background = [load_field(motor_manifest, "motor"), load_field(tv_manifest, "tv"),
                      audio[other[int(rng.integers(len(other)))]]]
        for bg in background:
            for snr in (-6, 0, 6):
                start = int(rng.integers(0, len(bg)-len(snippet)))
                src = bg[start:start+len(snippet)]
                mixed = mix(snippet, src, snr)
                wave_pcm = np.r_[src[:3200], mixed, src[-3200:]].astype(np.int16)
                cohorts[cohort]["positive"].append((snr, scores(model, query, wave_pcm, frontend)[0]))
    field_data = json.loads(target_manifest.read_text(encoding="utf-8"))
    target_rows = [x for x in field_data["clips"] if x["label"] == "target_possible"]
    longest = max(target_rows, key=lambda x: x["end_sample"] - x["start_sample"])
    with wave.open(str(target_manifest.parent / longest["wav"]), "rb") as src:
        pcm = np.frombuffer(src.readframes(src.getnframes()), dtype="<i2").copy()
    field_query = query_five(pcm[:len(pcm)//2], frontend)
    if field_query is None:
        raise ValueError("five TARGET enrollment patches unavailable")
    field_neg = []
    for manifest in (motor_manifest, tv_manifest):
        for r in json.loads(manifest.read_text(encoding="utf-8"))["clips"]:
            if r["label"] not in ("motor", "tv", "speech", "background"):
                continue
            with wave.open(str(manifest.parent / r["wav"]), "rb") as src:
                candidate = np.frombuffer(src.readframes(src.getnframes()), dtype="<i2").copy()
            field_neg.append((len(candidate)/16000, scores(model, field_query, candidate, frontend)[0]))
    field_pos = [("target_only_heldout", scores(model, field_query, pcm[len(pcm)//2:], frontend)[0])]
    for r in json.loads(tv_manifest.read_text(encoding="utf-8"))["clips"]:
        if r["label"] == "target_possible":
            with wave.open(str(tv_manifest.parent / r["wav"]), "rb") as src:
                field_pos.append((r["session"], scores(model, field_query,
                    np.frombuffer(src.readframes(src.getnframes()), dtype="<i2").copy(), frontend)[0]))
    choices = []
    for threshold in (.5, .7, .85, .9, .95, .98, .995):
        for voting in (2, 3):
            row = {"threshold": threshold, "voting": f"{voting}-of-{3 if voting==2 else 5}"}
            for cohort in ("validation", "test"):
                negatives = cohorts[cohort]["negative"]
                seconds = sum(time for time, _ in negatives)
                false = sum(episodes(predict_events(scores_, threshold, voting)) for _, scores_ in negatives)
                row[cohort] = {"negative_seconds": round(seconds, 1), "false_episodes": false,
                               "fp_per_hour": round(false * 3600 / seconds, 2) if seconds else None,
                               "positive_by_snr": {str(snr): {"hit": sum(bool(predict_events(scores_, threshold, voting).any())
                                                   for at_snr, scores_ in cohorts[cohort]["positive"] if at_snr == snr),
                                                   "total": sum(at_snr == snr for at_snr, _ in cohorts[cohort]["positive"])}
                                                   for snr in (-6, 0, 6)}}
            seconds = sum(t for t, _ in field_neg)
            false = sum(episodes(predict_events(x, threshold, voting)) for _, x in field_neg)
            row["field"] = {"negative_seconds": round(seconds, 1), "false_episodes": false,
                            "fp_per_hour": round(false * 3600 / seconds, 2),
                            "positive_sessions": {name: bool(predict_events(x, threshold, voting).any())
                                                  for name, x in field_pos}}
            choices.append(row)
    eligible = [r for r in choices if r["validation"]["fp_per_hour"] is not None
                and r["validation"]["fp_per_hour"] <= 10]
    selected = (max(eligible, key=lambda r: r["validation"]["positive_by_snr"]["0"]["hit"])
                if eligible else None)
    result = {"model": model_path.name, "frontend": frontend, "parameters": model.count_params(),
              "queries": {k: len(v["positive"]) // 9 for k, v in cohorts.items()},
              "chosen_on_validation_only": selected, "grid": choices,
              "truth_warning": "Actual rover mixed TARGET intervals have no frame-level onset labels."}
    output.mkdir(parents=True, exist_ok=True)
    (output / "query_stream.json").write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"model": model_path.name, "queries": result["queries"], "selected": selected}, indent=2))
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--esc", required=True, type=Path)
    parser.add_argument("--motor", required=True, type=Path)
    parser.add_argument("--tv", required=True, type=Path)
    parser.add_argument("--target", required=True, type=Path)
    parser.add_argument("--model", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--frontend", choices=("centered", "absolute"), default="centered")
    args = parser.parse_args()
    replay(args.esc, args.motor, args.tv, args.target, args.model, args.output, args.frontend)
