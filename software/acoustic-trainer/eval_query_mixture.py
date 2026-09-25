"""Source-disjoint query-by-example eval: five shots, waveform mixtures and hard negatives.

Each ESC-50 test recording is an *unknown target source*. Five lightly varied
repetitions of an audible 0.8s excerpt are enrolled (no offline retraining).
Positive tests contain that exact source in independent TV/motor/general audio
at -6/0/+6 dB; negatives include different recordings of the SAME sound class.
This measures inclusion of a registered event, not semantic class recognition.
"""
from __future__ import annotations

import argparse
import json
import sys
import wave
from collections import defaultdict
from pathlib import Path

import numpy as np

from esc10_fewshot_eval import read_resampled, identifier, enroll_summary
from train_fewshot import mix

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "software/audio_ml"))
from short_window_probe import frontend_mel  # noqa: E402


def load_field(manifest: Path, label: str) -> np.ndarray:
    for entry in json.loads(manifest.read_text(encoding="utf-8"))["clips"]:
        if entry["label"] == label and (entry["end_sample"] - entry["start_sample"]) >= 2 * 16000:
            with wave.open(str(manifest.parent / entry["wav"]), "rb") as src:
                return np.frombuffer(src.readframes(src.getnframes()), dtype="<i2").copy()
    raise ValueError(f"no {label} WAV in {manifest}")


def summary_windows(mel: np.ndarray, size: int) -> np.ndarray:
    active = mel.std(axis=1) > 12
    result = []
    for start in range(0, len(mel)-size+1, 8):
        patch = mel[start:start+size][active[start:start+size]]
        if len(patch) < max(4, size//4):
            result.append(np.zeros(96))
        else:
            result.append(np.r_[patch.mean(axis=0), patch.std(axis=0), patch.max(axis=0)])
    return np.array(result, dtype=np.float32)


def decisions(features: np.ndarray, slots: np.ndarray, threshold: float, dimension: int) -> np.ndarray:
    q = features[:, :dimension]
    p = slots[:, :dimension]
    qnorm = np.linalg.norm(q, axis=1)
    pnorm = np.linalg.norm(p, axis=1)
    dist = 1 - q @ p.T / np.maximum(qnorm[:, None] * pnorm[None, :], 1e-9)
    hits = (np.min(dist, axis=1) < threshold) & (qnorm > 0)
    result = np.zeros(len(hits), dtype=bool)
    for index in range(2, len(hits)):
        result[index] = hits[index-2:index+1].sum() >= 2
    return result


def episode_count(x: np.ndarray) -> int:
    return int(np.count_nonzero(x & ~np.r_[False, x[:-1]]))


def evaluate(manifest: Path, motor_manifest: Path, tv_manifest: Path, output: Path) -> dict:
    output.mkdir(parents=True, exist_ok=True)
    meta = json.loads(manifest.read_text(encoding="utf-8"))
    validation = meta["validation_classes"]
    test = meta["test_classes"]
    cases = [r for r in meta["clips"] if r["class"] in (*validation, *test) and r["split"] == "test"]
    motor = load_field(motor_manifest, "motor")
    tv = load_field(tv_manifest, "tv")
    audio = {row["file"]: read_resampled(manifest.parent / row["file"]) for row in meta["clips"]
             if row["split"] == "test"}
    # Every test recording of all 24 classes is a potential hard negative.
    negatives = {name: frontend_mel(pcm) for name, pcm in audio.items()}
    context_sizes = (20, 40)
    precomputed = {name: {size: summary_windows(mel, size) for size in context_sizes}
                   for name, mel in negatives.items()}
    lib = identifier(output)
    rng = np.random.default_rng(20260925)
    reports = []
    for cohort_name, cohort in (("validation", validation), ("test", test)):
        for row in cases:
            if row["class"] not in cohort:
                continue
            source = audio[row["file"]]
            mel = negatives[row["file"]]
            active = mel.std(axis=1) > 12
            if len(mel) < 80:
                continue
            start_frame = max(range(0, len(mel)-80+1, 8), key=lambda k: int(active[k:k+80].sum()))
            start = start_frame * 160
            snippet = source[start:start + (79 * 160 + 400)]
            if len(snippet) != 13040:
                continue
            # Independent repeated presentations: timing/volume and mild room echo.
            recordings = []
            for offset, echo in ((0, 0.0), (80, 0.0), (-80, 0.0), (40, .08), (-40, .12)):
                shifted = np.roll(snippet, offset).astype(np.float32)
                shifted[:max(0, offset)] = 0
                if offset < 0:
                    shifted[offset:] = 0
                if echo:
                    shifted[400:] += echo * shifted[:-400]
                recordings.append(np.clip(shifted, -32768, 32767).astype(np.int16))
            exemplars = [enroll_summary(frontend_mel(w), lib) for w in recordings]
            if any(sample is None for sample in exemplars):
                continue
            slots = np.array([s[k:k+96] for s in exemplars for k in (0, 96)], dtype=np.float32)
            # Exclude the query clip from negative exposure; same class other clip
            # is explicitly a hard negative, not automatically the registered source.
            same_class = [other["file"] for other in cases if other["class"] == row["class"]
                          and other["file"] != row["file"]]
            different = sorted(other["file"] for other in meta["clips"] if other["split"] == "test"
                               and other["class"] != row["class"])
            exposed = same_class + list(rng.choice(different, size=12, replace=False))
            chosen = [audio[other["file"]] for other in cases if other["class"] != row["class"]]
            backgrounds = (motor, tv, chosen[int(rng.integers(len(chosen)))])
            positives = defaultdict(list)
            for background in backgrounds:
                for snr in (-6, 0, 6):
                    begin = int(rng.integers(0, max(1, len(background)-len(snippet))))
                    back = background[begin:begin + len(snippet)]
                    if len(back) != len(snippet):
                        continue
                    # Pad with background on both sides; the inserted TARGET is
                    # known to be present in every positive example.
                    mixture = mix(snippet, back, snr)
                    signal = np.r_[back[:3200], mixture, back[-3200:]].astype(np.int16)
                    m = frontend_mel(signal)
                    for size in context_sizes:
                        positives[(size, snr)].append(summary_windows(m, size))
            per_source = {"class": row["class"], "query": row["file"], "cohort": cohort_name,
                          "negative_audio_s": round(sum(len(audio[name])/16000 for name in exposed), 1),
                          "candidates": []}
            for size in context_sizes:
                for dim in (32, 96):
                    for threshold in (.04, .08, .12):
                        negative_episodes = sum(episode_count(decisions(precomputed[name][size], slots,
                                                                          threshold, dim)) for name in exposed)
                        positive = {str(snr): {"hits": sum(bool(decisions(feat, slots, threshold, dim).any())
                                                 for feat in positives[(size, snr)]),
                                                "total": len(positives[(size, snr)])}
                                    for snr in (-6, 0, 6)}
                        per_source["candidates"].append({"context_ms": size*10, "dimension": dim,
                            "threshold": threshold, "negative_episodes": negative_episodes,
                            "positives": positive})
            reports.append(per_source)
    aggregate = []
    for cohort_name in ("validation", "test"):
        selected = [r for r in reports if r["cohort"] == cohort_name]
        for size in context_sizes:
            for dim in (32, 96):
                for threshold in (.04, .08, .12):
                    rows = [next(c for c in r["candidates"] if c["context_ms"] == size*10 and
                                 c["dimension"] == dim and c["threshold"] == threshold) for r in selected]
                    seconds = sum(r["negative_audio_s"] for r in selected)
                    aggregate.append({"cohort": cohort_name, "context_ms": size*10,
                        "dimension": dim, "threshold": threshold, "query_count": len(rows),
                        "false_alarm_episodes": sum(r["negative_episodes"] for r in rows),
                        "fp_per_hour": round(sum(r["negative_episodes"] for r in rows) * 3600 / seconds, 2)
                                       if seconds else None,
                        "positive_by_snr": {str(snr): {"hits": sum(r["positives"][str(snr)]["hits"] for r in rows),
                                                  "total": sum(r["positives"][str(snr)]["total"] for r in rows)}
                                             for snr in (-6, 0, 6)}})
    result = {"source": meta["source"], "positive_label": "same exact sound excerpt mixed at PCM level",
              "negative_label": "different recording, including same sound category",
              "sources": reports, "aggregate": aggregate}
    (output / "source_fewshot.json").write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(aggregate, indent=2))
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--esc", type=Path, required=True)
    parser.add_argument("--motor", type=Path, required=True)
    parser.add_argument("--tv", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    evaluate(args.esc, args.motor, args.tv, args.output)
