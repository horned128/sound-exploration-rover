"""Assess query matching after causal per-band background-power subtraction.

Background power is estimated from the first 200ms of each recording and
frozen; it never uses a TARGET label. The query is five mild repetitions of
an unseen ESC source. No off-target model training is performed.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from esc10_fewshot_eval import read_resampled
from eval_query_mixture import load_field
from features import mel_filterbank, periodic_hann
from train_fewshot import mix

MEL = mel_filterbank().T
HANN = periodic_hann(np.dtype(np.float32))


def power(pcm: np.ndarray) -> np.ndarray:
    frames = np.lib.stride_tricks.sliding_window_view(pcm.astype(np.float32) / 32768, 400)[::160]
    fft = np.fft.rfft(frames * HANN[None, :], n=512, axis=1)
    return np.maximum((fft.real * fft.real + fft.imag * fft.imag) @ MEL, 1e-12)


def mel_from_power(values: np.ndarray, reference: np.ndarray | None = None) -> np.ndarray:
    if reference is not None:
        values = np.maximum(values - reference[None, :], 1e-12)
    log = np.log(values)
    normalized = (log - log.mean(axis=1, keepdims=True)) / .125
    return np.clip(np.rint(normalized), -128, 127).astype(np.float32)


def window_summaries(mel: np.ndarray, size: int = 20) -> np.ndarray:
    active = mel.std(axis=1) > 12
    values = []
    for first in range(0, len(mel) - size + 1, 8):
        frames = mel[first:first+size][active[first:first+size]]
        values.append(np.r_[frames.mean(axis=0), frames.std(axis=0), frames.max(axis=0)]
                      if len(frames) >= 5 else np.zeros(96))
    return np.array(values, dtype=np.float32)


def detection(summary: np.ndarray, refs: np.ndarray, threshold: float) -> np.ndarray:
    l = np.linalg.norm(summary, axis=1)
    r = np.linalg.norm(refs, axis=1)
    distances = 1 - summary @ refs.T / np.maximum(l[:, None] * r[None, :], 1e-9)
    hit = (distances.min(axis=1) < threshold) & (l > 0)
    result = np.zeros(len(hit), dtype=bool)
    for i in range(2, len(hit)):
        result[i] = hit[i-2:i+1].sum() >= 2
    return result


def alarm_count(x: np.ndarray) -> int:
    return int(np.count_nonzero(x & ~np.r_[False, x[:-1]]))


def evaluate(manifest: Path, motor_manifest: Path, tv_manifest: Path, output: Path) -> dict:
    meta = json.loads(manifest.read_text(encoding="utf-8"))
    cohorts = {"validation": meta["validation_classes"], "test": meta["test_classes"]}
    cases = [r for r in meta["clips"] if r["split"] == "test"]
    audio = {r["file"]: read_resampled(manifest.parent / r["file"]) for r in cases}
    source_cases = [r for r in cases if r["class"] in (*cohorts["validation"], *cohorts["test"])]
    base = {}
    for name, pcm in audio.items():
        spectral = power(pcm)
        # No audio before first 200ms: do not count those incomplete windows.
        background = np.median(spectral[:20], axis=0)
        base[name] = window_summaries(mel_from_power(spectral, background))
    motor = load_field(motor_manifest, "motor")
    tv = load_field(tv_manifest, "tv")
    rng = np.random.default_rng(20260925)
    samples = []
    for source in source_cases:
        pcm = audio[source["file"]]
        mel = mel_from_power(power(pcm))
        active = mel.std(axis=1) > 12
        if len(mel) < 80:
            continue
        first_frame = max(range(0, len(mel)-80+1, 8), key=lambda i: int(active[i:i+80].sum()))
        wave = pcm[first_frame*160:first_frame*160+13040]
        if len(wave) < 13040:
            continue
        references = []
        for shift in (0, 80, -80, 40, -40):
            repeat = np.roll(wave, shift).copy()
            if shift > 0:
                repeat[:shift] = 0
            elif shift < 0:
                repeat[shift:] = 0
            query = mel_from_power(power(repeat))
            summary = window_summaries(query, 40)
            if not len(summary):
                break
            references.append(summary[len(summary)//2])
        if len(references) != 5:
            continue
        refs = np.array(references, dtype=np.float32)
        same = [c["file"] for c in cases if c["class"] == source["class"] and c["file"] != source["file"]]
        other = sorted(c["file"] for c in cases if c["class"] != source["class"])
        negative_files = same + list(rng.choice(other, 12, replace=False))
        backgrounds = (motor, tv, audio[other[int(rng.integers(len(other)))]] )
        positives = []
        for noise in backgrounds:
            for snr in (-6, 0, 6):
                i = int(rng.integers(0, len(noise)-len(wave)))
                bg = noise[i:i+len(wave)]
                mixture = np.r_[bg[:3200], mix(wave, bg, snr), bg[-3200:]].astype(np.int16)
                spect = power(mixture)
                estimate = np.median(spect[:20], axis=0)
                positives.append((snr, window_summaries(mel_from_power(spect, estimate))))
        samples.append({"cohort": "validation" if source["class"] in cohorts["validation"] else "test",
                        "query": source["file"], "refs": refs, "negatives": negative_files,
                        "positives": positives})
    summary = []
    for cohort in cohorts:
        for threshold in (.02, .04, .08, .12, .20, .32):
            sources = [s for s in samples if s["cohort"] == cohort]
            neg_secs = fp = 0
            positive = {str(snr): {"hit": 0, "total": 0} for snr in (-6, 0, 6)}
            for s in sources:
                for name in s["negatives"]:
                    neg_secs += len(audio[name]) / 16000
                    fp += alarm_count(detection(base[name], s["refs"], threshold))
                for snr, values in s["positives"]:
                    positive[str(snr)]["total"] += 1
                    positive[str(snr)]["hit"] += bool(detection(values, s["refs"], threshold).any())
            summary.append({"cohort": cohort, "threshold": threshold, "queries": len(sources),
                            "false_episodes": fp, "fp_per_hour": round(fp * 3600 / neg_secs, 2),
                            "positive_by_snr": positive})
    output.mkdir(parents=True, exist_ok=True)
    report = {"source": meta["source"], "method": "frozen 200ms background power subtraction + 200ms matching /80ms hop/2-of-3 vote",
              "note": "Background is estimated from 200ms immediately preceding test audio; the source is never estimated from the mixture.",
              "summary": summary}
    (output / "subtraction.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(summary, indent=2))
    return report


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--esc", required=True, type=Path)
    parser.add_argument("--motor", required=True, type=Path)
    parser.add_argument("--tv", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    evaluate(args.esc, args.motor, args.tv, args.output)
