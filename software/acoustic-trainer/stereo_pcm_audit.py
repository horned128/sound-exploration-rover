"""Pair both XVF3800 I2S slots by ESP boot ID inferred from clock resets.

Reports per-slot level, byte identity and correlation. A stereo I2S setting
does NOT prove independent microphone signals; only measurements can decide
whether the second output adds spatial information.
"""
from __future__ import annotations

import argparse
import base64
import json
import wave
from collections import defaultdict
from pathlib import Path

import numpy as np


def analyze(path: Path, wav_dir: Path | None = None, min_clip_seconds: float = .5) -> dict:
    by_boot: list[dict[int, dict[int, np.ndarray]]] = [defaultdict(dict)]
    last_ms = last_sample = None
    packets = [0, 0]
    with path.open(encoding="utf-8") as stream:
        for line in stream:
            row = json.loads(line)
            if row.get("record_type") not in ("acoustic_pcm", "acoustic_pcm_ch1"):
                continue
            channel = 0 if row["record_type"] == "acoustic_pcm" else 1
            first = row["first_sample"]
            ms = row["esp_ms"]
            if channel == 0:
                if ((last_ms is not None and ms + 1000 < last_ms) or
                    (last_sample is not None and first + 16000 < last_sample)):
                    by_boot.append(defaultdict(dict))
                last_ms, last_sample = ms, first
            raw = base64.b64decode(row["pcm16le_b64"], validate=True)
            if len(raw) != row["sample_count"] * 2:
                raise ValueError(f"PCM payload mismatch at sample {first}")
            samples = np.frombuffer(raw, dtype="<i2").astype(np.float64)
            stored = by_boot[-1][first]
            if channel in stored and not np.array_equal(stored[channel], samples):
                raise ValueError(f"conflicting duplicate in boot {len(by_boot)-1}: {first}")
            if channel not in stored:
                packets[channel] += 1
            stored[channel] = samples
    per_boot = []
    if wav_dir is not None:
        wav_dir.mkdir(parents=True, exist_ok=True)
    for index, items in enumerate(by_boot):
        aligned = [(first, a[0], a[1]) for first, a in sorted(items.items()) if set(a) == {0, 1}]
        if any(len(a) != len(b) for _, a, b in aligned):
            raise ValueError(f"stereo channel lengths disagree in boot {index}")
        pairs = [(a, b) for _, a, b in aligned]
        if not pairs:
            per_boot.append({"boot": index, "paired_blocks": 0})
            continue
        left = np.concatenate([p[0] for p in pairs])
        right = np.concatenate([p[1] for p in pairs])
        rms = [float(np.sqrt(np.mean(channel * channel))) for channel in (left, right)]
        correlations = []
        for a, b in pairs:
            if np.std(a) > 20 and np.std(b) > 20:
                correlations.append(float(np.corrcoef(a, b)[0, 1]))
        avg_abs_corr = float(np.median(np.abs(correlations))) if correlations else None
        clips = []
        if wav_dir is not None:
            runs = []
            chunk = []
            first_in_run = expected = None
            for first, a, b in aligned:
                if expected is not None and first != expected:
                    runs.append((first_in_run, chunk))
                    chunk = []
                if not chunk:
                    first_in_run = first
                chunk.append(np.stack((a, b), axis=1).astype("<i2"))
                expected = first + len(a)
            if chunk:
                runs.append((first_in_run, chunk))
            for run, (first, blocks) in enumerate(runs):
                frames = np.concatenate(blocks)
                if len(frames) < min_clip_seconds * 16000:
                    continue
                filename = f"{path.stem}-boot{index}-run{run}.wav"
                with wave.open(str(wav_dir / filename), "wb") as output:
                    output.setnchannels(2)
                    output.setsampwidth(2)
                    output.setframerate(16000)
                    output.writeframes(frames.tobytes())
                clips.append({"file": filename, "first_sample": first, "samples": len(frames)})
        per_boot.append({"boot": index, "paired_blocks": len(pairs),
            "unpaired_channel_0": sum(0 in x and 1 not in x for x in items.values()),
            "unpaired_channel_1": sum(1 in x and 0 not in x for x in items.values()),
            "seconds_paired": len(left) / 16000,
            "rms_dbfs": [round(20 * np.log10(max(1e-9, level / 32768)), 2) for level in rms],
            "identical_sample_fraction": round(float(np.mean(left == right)), 5),
            "active_block_median_absolute_correlation": round(avg_abs_corr, 4) if avg_abs_corr is not None else None,
            "second_slot_information": "unproven" if avg_abs_corr is None else (
                "potentially_distinct" if avg_abs_corr < .95 and rms[1] > 32 else "likely_duplicate_or_silent"),
            "stereo_wav_clips": clips})
    return {"log": path.name, "channel_packets": packets, "boots": per_boot,
            "caution": "Different processed I2S slots are not necessarily raw independent microphones."}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--wav-dir", type=Path)
    args = parser.parse_args()
    report = analyze(args.log, args.wav_dir)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2, ensure_ascii=False))
