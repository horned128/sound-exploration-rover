"""Class-disjoint five-shot evaluation using ESP32-S3 C log-mel and CPU0 C enrollment.

ESC-10 audio is kept out of the repository. No sound type is fixed in the model:
each of ten classes is registered from five fold-1/2 clips and tested on seven
fold-3/4/5 clips. Labels are clip-level, not exact event onsets.
"""
from __future__ import annotations

import argparse
import ctypes
import json
import subprocess
import sys
import wave
from collections import defaultdict
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "software/audio_ml"))
from short_window_probe import frontend_mel, select_three_if_two_outliers  # noqa: E402


def identifier(output: Path) -> ctypes.CDLL:
    """Build only the real CPU0 identifier into the writable evaluation directory."""
    root = ROOT / "firmware/ra8p1/SoundExplorationRover_CPU0/src"
    target = output / ("libacoustic.dylib" if sys.platform == "darwin" else "libacoustic.so")
    mode = "-dynamiclib" if sys.platform == "darwin" else "-shared"
    subprocess.run(["clang", "-std=c99", "-O2", "-fPIC", mode,
                    f"-I{ROOT / 'software/control-sim/shim'}", f"-I{root}",
                    str(root / "services/acoustic_identifier.c"), "-lm", "-o", str(target)], check=True)
    lib = ctypes.CDLL(str(target))
    lib.acoustic_identifier_summary_create.argtypes = [ctypes.POINTER(ctypes.c_int8), ctypes.c_uint32,
                                                         ctypes.POINTER(ctypes.c_int8), ctypes.POINTER(ctypes.c_uint8)]
    lib.acoustic_identifier_summary_create.restype = ctypes.c_int32
    return lib


def read_resampled(path: Path) -> np.ndarray:
    with wave.open(str(path), "rb") as source:
        rate, width, channels = source.getframerate(), source.getsampwidth(), source.getnchannels()
        if width != 2 or channels != 1:
            raise ValueError(f"expected mono 16-bit WAV: {path}")
        raw = np.frombuffer(source.readframes(source.getnframes()), dtype="<i2").astype(np.float64)
    if rate != 16000:
        taps = np.arange(-100, 101, dtype=np.float64)
        filt = 2 * 7400 / rate * np.sinc(2 * 7400 / rate * taps) * np.hamming(len(taps))
        filt /= filt.sum()
        n = 1 << (len(raw) + len(filt) - 1).bit_length()
        lowpass = np.fft.irfft(np.fft.rfft(raw, n) * np.fft.rfft(filt, n), n)
        lowpass = lowpass[100:100 + len(raw)]
        raw = np.interp(np.arange(round(len(raw) * 16000 / rate)) * rate / 16000,
                        np.arange(len(raw)), lowpass)
    return np.clip(np.rint(raw), -32768, 32767).astype(np.int16)


def enroll_summary(mel: np.ndarray, lib: ctypes.CDLL) -> list[int] | None:
    if len(mel) < 80:
        return None
    active = mel.std(axis=1) > 12
    start = max(range(0, len(mel) - 80 + 1, 8),
                key=lambda k: (active[k:k + 80].sum(), -abs(k - 100)))
    patch = np.ascontiguousarray(mel[start:start + 80], dtype=np.int8)
    out = (ctypes.c_int8 * 192)()
    count = ctypes.c_uint8()
    valid = lib.acoustic_identifier_summary_create(
        patch.ctypes.data_as(ctypes.POINTER(ctypes.c_int8)), 80, out, ctypes.byref(count))
    return list(out) if valid else None


def windows(mel: np.ndarray, length: int) -> tuple[np.ndarray, np.ndarray]:
    active = mel.std(axis=1) > 12
    q = []
    for start in range(0, len(mel) - length + 1, 8):
        window = mel[start:start + length][active[start:start + length]]
        if len(window) < max(4, length // 4):
            q.append(np.zeros(96, dtype=np.float32))
        else:
            q.append(np.concatenate((window.mean(axis=0), window.std(axis=0), window.max(axis=0))))
    return np.array(q, dtype=np.float32), np.arange(len(q)) * 80 + length * 10 + 15


def predict(summary: np.ndarray, prototype: np.ndarray, dimension: int, threshold: float) -> np.ndarray:
    q = summary[:, :dimension]
    p = prototype[:, :dimension]
    qnorm = np.linalg.norm(q, axis=1)
    pnorm = np.linalg.norm(p, axis=1)
    score = 1 - (q @ p.T) / np.maximum(qnorm[:, None] * pnorm[None, :], 1e-9)
    passed = (np.min(score, axis=1) < threshold) & (qnorm > 0)
    result = np.zeros(len(passed), dtype=bool)
    for i in range(2, len(passed)):
        result[i] = passed[i-2:i+1].sum() >= 2
    return result


def alarm_episodes(decisions: np.ndarray) -> int:
    return int(np.count_nonzero(decisions & ~np.r_[False, decisions[:-1]]))


def evaluate(esc_manifest: Path, output: Path) -> dict:
    output.mkdir(parents=True, exist_ok=True)
    lib = identifier(output)
    manifest = json.loads(esc_manifest.read_text(encoding="utf-8"))
    classes = manifest["classes"]
    clips = defaultdict(lambda: {"support": [], "test": []})
    samples = {}
    features = {}
    for row in manifest["clips"]:
        pcm = read_resampled(esc_manifest.parent / row["file"])
        samples[row["file"]] = pcm
        features[row["file"]] = frontend_mel(pcm)
        clips[row["class"]][row["split"]].append(row["file"])
    prototypes = {}
    for category in classes:
        enrollment = [enroll_summary(features[file], lib) for file in clips[category]["support"]]
        if any(example is None for example in enrollment):
            prototypes[category] = None
        else:
            retained = select_three_if_two_outliers(enrollment)
            prototypes[category] = np.array([s[k:k+96] for s in retained for k in (0, 96)], dtype=np.float32)
    report = {"upstream": manifest["source"], "classes": classes, "support_per_class": 5,
              "test_per_class": 7, "retained_profile_samples": {
                  category: None if prototypes[category] is None else len(prototypes[category]) // 2
                  for category in classes}, "candidates": []}
    test_files = {file for category in classes for file in clips[category]["test"]}
    precomputed = {file: {length: windows(features[file], length) for length in (20, 32, 40)}
                   for file in test_files}
    for length in (20, 32, 40):
        for dimension in (32, 96):
            true_clips = negative_clips = hit_clips = false_clips = false_episodes = 0
            negative_seconds = 0.0
            first_updates = []
            per_class = {}
            for category in classes:
                proto = prototypes[category]
                if proto is None:
                    continue
                per_class[category] = {"hit": 0, "total": 0, "false_episodes": 0}
                for actual in classes:
                    for file in clips[actual]["test"]:
                        q, time_ms = precomputed[file][length]
                        detection = predict(q, proto, dimension, .08)
                        if category == actual:
                            true_clips += 1
                            hit_clips += bool(detection.any())
                            per_class[category]["total"] += 1
                            per_class[category]["hit"] += bool(detection.any())
                            if detection.any():
                                first_updates.append(int(time_ms[np.argmax(detection)]))
                        else:
                            negative_clips += 1
                            negative_seconds += len(samples[file]) / 16000
                            false_clips += bool(detection.any())
                            count = alarm_episodes(detection)
                            false_episodes += count
                            per_class[category]["false_episodes"] += count
            report["candidates"].append({"context_ms": length * 10, "dimension": dimension,
                "threshold": .08, "vote": "2-of-3 at 80 ms",
                "target_clip_recall": hit_clips / true_clips if true_clips else None,
                "target_hits": hit_clips, "target_clips": true_clips,
                "negative_episodes": false_episodes, "negative_clips_with_fp": false_clips,
                "negative_clips": negative_clips, "negative_hours": round(negative_seconds / 3600, 3),
                "fp_per_hour": false_episodes * 3600 / negative_seconds if negative_seconds else None,
                "first_positive_update_ms_median": float(np.median(first_updates)) if first_updates else None,
                "first_update_note": "clip start, not sound onset", "per_class": per_class})
    output.mkdir(parents=True, exist_ok=True)
    (output / "esc10_results.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"candidates": [{k: v for k, v in r.items() if k != "per_class"}
                                     for r in report["candidates"]], "model_parameters": 0}, indent=2))
    return report


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    evaluate(args.manifest, args.output)
