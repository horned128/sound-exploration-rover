"""Compare processed I2S left/right/mid/side across four field conditions.

This is an information audit, not proof of raw four-microphone access or of
target detection performance. Use only time-aligned stereo WAVs exported by
stereo_pcm_audit.py, which splits UDP gaps and ESP boots.
"""
from __future__ import annotations

import argparse
import json
import wave
from pathlib import Path

import numpy as np


def read_audio(folder: Path) -> list[np.ndarray]:
    result = []
    for filename in sorted(folder.glob("*.wav")):
        with wave.open(str(filename), "rb") as source:
            if (source.getframerate(), source.getnchannels(), source.getsampwidth()) != (16000, 2, 2):
                raise ValueError(f"stereo PCM16 16k required: {filename}")
            values = np.frombuffer(source.readframes(source.getnframes()), dtype="<i2")
            result.append(values.astype(np.float64).reshape(-1, 2))
    if not result:
        raise ValueError(f"no paired stereo PCM in {folder}")
    return result


def summarize(clips: list[np.ndarray]) -> dict:
    data = np.concatenate(clips)
    left, right = data.T
    components = {"left": left, "right": right, "mid": (left + right) / 2,
                  "side": (left - right) / 2}
    energy = {key: round(20 * np.log10(max(1e-9, np.sqrt(np.mean(x*x)) / 32768)), 2)
              for key, x in components.items()}
    lags = []
    for clip in clips:
        if np.std(clip[:, 0]) < 20 or np.std(clip[:, 1]) < 20:
            continue
        a, b = clip.T
        values = []
        for lag in range(-8, 9):
            x, y = ((a[-lag:], b[:len(b)+lag]) if lag < 0 else
                    (a[:len(a)-lag], b[lag:])) if lag else (a, b)
            values.append((lag, float(np.corrcoef(x, y)[0, 1])))
        lags.append(max(values, key=lambda pair: abs(pair[1])))
    return {"paired_seconds": round(len(data) / 16000, 2), "rms_dbfs": energy,
            "clip_count": len(clips), "best_lag_samples": [row[0] for row in lags],
            "best_lag_abs_corr": [round(abs(row[1]), 4) for row in lags],
            "sample_identity_fraction": round(float(np.mean(left == right)), 5)}


def analyze(folders: dict[str, Path]) -> dict:
    conditions = {name: summarize(read_audio(folder)) for name, folder in folders.items()}
    target = conditions["target"]["rms_dbfs"]
    motor = conditions["motor"]["rms_dbfs"]
    # Level varies across recordings. This contrast is diagnostic only;
    # background and TARGET are not independently isolated in a mixed recording.
    contrast = {key: round(target[key] - motor[key], 2) for key in target}
    return {"conditions": conditions,
            "target_vs_motor_contrast_db": contrast,
            "side_improvement_vs_mid_db": round(contrast["side"] - contrast["mid"], 2),
            "conclusion_rule": "If channels remain highly correlated at best lag and side gives no consistent target/background gain, 2-slot processing is not an independent spatial source separation input.",
            "caution": "Distinct I2S slots do not imply access to raw four-microphone PCM."}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    for label in ("silence", "target", "target_noise", "motor"):
        parser.add_argument("--" + label.replace("_", "-"), type=Path, required=True)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    result = analyze({label: getattr(args, label) for label in
                      ("silence", "target", "target_noise", "motor")})
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2))
