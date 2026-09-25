"""Compare I2S left/right/mid/side for the same field five-shot query.

Training queries come only from the TARGET-only recording. Motor and quiet
audio are genuine negatives. TARGET+TV is session-level positive and may
contain gaps between strikes; window acceptance is NOT event-level recall.
"""
from __future__ import annotations

import argparse
import ctypes
import json
import sys
import wave
from pathlib import Path

import numpy as np

from esc10_fewshot_eval import identifier, enroll_summary

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "software/audio_ml"))
from short_window_probe import frontend_mel, select_three_if_two_outliers  # noqa: E402


def sounds(directory: Path, component: str) -> list[np.ndarray]:
    tracks = []
    for path in sorted(directory.glob("*.wav")):
        with wave.open(str(path), "rb") as source:
            if (source.getnchannels(), source.getsampwidth(), source.getframerate()) != (2, 2, 16000):
                raise ValueError(path)
            values = np.frombuffer(source.readframes(source.getnframes()), dtype="<i2").reshape(-1, 2)
        left, right = values.astype(np.float32).T
        pcm = {"left": left, "right": right, "mid": (left + right) * .5,
               "side": (left - right) * .5}[component]
        tracks.append(np.clip(np.rint(pcm), -32768, 32767).astype(np.int16))
    return tracks


def summarize(mel: np.ndarray, frames: int = 20) -> np.ndarray:
    active = mel.std(axis=1) > 12
    vectors = []
    for start in range(0, len(mel)-frames+1, 8):
        patch = mel[start:start+frames][active[start:start+frames]]
        vectors.append(np.r_[patch.mean(axis=0), patch.std(axis=0), patch.max(axis=0)]
                       if len(patch) >= 5 else np.zeros(96))
    return np.array(vectors, dtype=np.float32)


def matches(vectors: np.ndarray, support: np.ndarray, threshold: float) -> np.ndarray:
    if not len(vectors):
        return np.array([], dtype=bool)
    lhs = np.linalg.norm(vectors, axis=1)
    rhs = np.linalg.norm(support, axis=1)
    distance = 1 - vectors @ support.T / np.maximum(lhs[:, None] * rhs[None, :], 1e-9)
    passed = (distance.min(axis=1) < threshold) & (lhs > 0)
    voted = np.zeros(len(vectors), dtype=bool)
    for i in range(2, len(vectors)):
        voted[i] = passed[i-2:i+1].sum() >= 2
    return voted


def episodes(x: np.ndarray) -> int:
    return int(np.count_nonzero(x & ~np.r_[False, x[:-1]]))


def evaluate(dirs: dict[str, Path], out: Path) -> dict:
    out.mkdir(parents=True, exist_ok=True)
    lib = identifier(out)
    results = []
    for component in ("left", "right", "mid", "side"):
        recordings = {label: [frontend_mel(raw) for raw in sounds(path, component)]
                      for label, path in dirs.items()}
        enrolled = [enroll_summary(mel, lib) for mel in recordings["target"]]
        if len(enrolled) < 5 or any(x is None for x in enrolled[:5]):
            results.append({"component": component, "enrollment": "failed", "valid_profiles": sum(x is not None for x in enrolled)})
            continue
        kept = select_three_if_two_outliers(enrolled[:5])
        slots = np.array([s[k:k+96] for s in kept for k in (0, 96)], dtype=np.float32)
        for threshold in (.04, .06, .08, .10, .12):
            group = {}
            for label, vectors in recordings.items():
                decisions = [matches(summarize(mel), slots, threshold) for mel in vectors]
                group[label] = {"windows": sum(len(x) for x in decisions),
                                "accepted": sum(int(x.sum()) for x in decisions),
                                "episodes": sum(episodes(x) for x in decisions)}
            negatives_s = sum(len(x) / 16000 for label in ("silence", "motor")
                              for x in sounds(dirs[label], component))
            results.append({"component": component, "retained_profiles": len(kept),
                            "threshold": threshold, "groups": group,
                            "negative_fp_per_hour": round(3600 * (group["silence"]["episodes"] +
                                group["motor"]["episodes"]) / negatives_s, 2)})
    report = {"components": results,
              "interpretation": "No TV-only stereo negative; mixture windows can lack TARGET. Compare with field confidence, not certified FP/hour or recall."}
    (out / "stereo_matching.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(results, indent=2))
    return report


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    for condition in ("silence", "target", "target_noise", "motor"):
        parser.add_argument("--" + condition.replace("_", "-"), type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    evaluate({label: getattr(args, label) for label in
              ("silence", "target", "target_noise", "motor")}, args.output)
