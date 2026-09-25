"""Reproducible, non-deployed A-variant scan on real PCM using firmware C log-mel.

Compares shorter 200/320/400 ms windows against the stored 192D five-shot
profile (no TARGET-specific offline training). Negative-only threshold search is
an *in-sample* diagnostic, not a certified false positive rate or recall.
"""
from __future__ import annotations

import argparse
import csv
import ctypes
import itertools
import json
import wave
from pathlib import Path

import numpy as np

from legacy_audit import consensus, distance as profile_distance, peak, weights

ROOT = Path(__file__).resolve().parents[2]
MEL_LIBRARY = ROOT / "software/acoustic-trainer/build/log_mel/liblog_mel.dylib"
MEL_LIBRARY_LINUX = ROOT / "software/acoustic-trainer/build/log_mel/liblog_mel.so"


def frontend_mel(samples: np.ndarray) -> np.ndarray:
    libpath = MEL_LIBRARY if MEL_LIBRARY.exists() else MEL_LIBRARY_LINUX
    if not libpath.exists():
        raise FileNotFoundError("run: uv run python tests/test_log_mel.py (in software/acoustic-trainer)")
    lib = ctypes.CDLL(str(libpath))
    lib.log_mel_extractor_context_size.restype = ctypes.c_size_t
    lib.log_mel_extractor_init.argtypes = [ctypes.c_void_p]
    lib.log_mel_extractor_self_test.argtypes = [ctypes.c_void_p]
    lib.log_mel_extractor_self_test.restype = ctypes.c_bool
    callback_t = ctypes.CFUNCTYPE(None, ctypes.POINTER(ctypes.c_int8), ctypes.c_void_p)
    lib.log_mel_extractor_feed.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_int32), ctypes.c_size_t,
                                           callback_t, ctypes.c_void_p]
    lib.log_mel_extractor_feed.restype = ctypes.c_size_t
    context = ctypes.create_string_buffer(lib.log_mel_extractor_context_size())
    lib.log_mel_extractor_init(context)
    if not lib.log_mel_extractor_self_test(context):
        raise RuntimeError("ESP32-S3 log-mel self-test failed on host")
    frames = []

    @callback_t
    def collect(frame: ctypes.POINTER(ctypes.c_int8), _: ctypes.c_void_p) -> None:
        frames.append(np.ctypeslib.as_array(frame, shape=(32,)).copy())

    samples32 = (np.asarray(samples, dtype=np.int32) << 16).copy()
    for start in range(0, len(samples32), 4096):
        chunk = samples32[start:start + 4096]
        lib.log_mel_extractor_feed(context, chunk.ctypes.data_as(ctypes.POINTER(ctypes.c_int32)),
                                   len(chunk), collect, None)
    return np.array(frames, dtype=np.float32)


def read_manifest(manifest: Path) -> list[tuple[Path, dict]]:
    data = json.loads(manifest.read_text(encoding="utf-8"))
    return [(manifest.parent, row) for row in data["clips"]]


def vote_hits(rows: list[dict], threshold: float, required: int = 2, recent: int = 3) -> int:
    """Count positive updates after N-of-M; a time gap invalidates stale votes."""
    history: list[bool] = []
    last = None
    result = 0
    for row in sorted(rows, key=lambda r: r["first_sample"]):
        if last is None or row["first_sample"] - last != 8 * 160:
            history = []
        history = (history + [row["distance"] < threshold])[-recent:]
        result += len(history) == recent and sum(history) >= required
        last = row["first_sample"]
    return result


def select_three_if_two_outliers(samples: list[list[int]]) -> list[list[int]]:
    """Diagnostic-only symmetric 3-of-5 consensus, independent of sound name."""
    if len(samples) != 5:
        return samples
    peaks = [peak(sample) for sample in samples]
    candidates = []
    for group in itertools.combinations(range(5), 3):
        w = weights(peaks[group[0]])
        if all(abs(peaks[i] - peaks[j]) <= 6 and
               profile_distance(samples[i], samples[j], w) <= 0.25
               for i, j in itertools.combinations(group, 2)):
            outsiders = [i for i in range(5) if i not in group]
            if all(all(abs(peaks[o] - peaks[i]) > 6 or
                       profile_distance(samples[o], samples[i], w) > 0.25 for i in group)
                   for o in outsiders):
                candidates.append(group)
    if len(candidates) != 1:
        return samples
    return [samples[i] for i in candidates[0]]


def scan(log: Path, manifest: Path, out: Path, majority_probe: bool = False,
         extra_manifests: tuple[Path, ...] = ()) -> dict:
    with log.open(encoding="utf-8") as src:
        samples = [r for line in src if (r := json.loads(line)).get("record_type") == "acoustic_sample"]
    generations = {r["profile_generation"] for r in samples}
    if len(generations) != 1:
        raise ValueError("one unambiguous registration generation is required")
    profile = {r["sample_index"]: r["summary"] for r in samples}
    if set(profile) != set(range(5)):
        raise ValueError("complete single five-shot profile required")
    target_bin, usable = consensus([profile[i] for i in range(5)])
    if majority_probe:
        usable = select_three_if_two_outliers(usable)
    profile_slots = np.array([s[k:k + 96] for s in usable for k in (0, 96)], dtype=np.float32)
    support_loo = {}
    for mode, count in (("mean", 32), ("mean_std_max", 96)):
        distances = []
        for i in range(len(usable)):
            refs = np.array([s[k:k + 96][:count] for j, s in enumerate(usable) if j != i
                             for k in (0, 96)], dtype=np.float32)
            examples = np.array([usable[i][k:k + 96][:count] for k in (0, 96)], dtype=np.float32)
            norm = np.maximum(np.linalg.norm(refs, axis=1)[:, None] *
                              np.linalg.norm(examples, axis=1)[None, :], 1e-9)
            distances.append(round(float(np.min(1 - (refs @ examples.T) / norm)), 5))
        support_loo[mode] = distances
    # The legacy peak weights were tuned for one field capture. Use uniform
    # distances so that a peak shared with TV doesn't dominate a short window.
    clips = [item for file in (manifest, *extra_manifests) for item in read_manifest(file)]
    by_session = sorted({clip["session"] for _, clip in clips})
    rows = []
    best = []
    for parent, clip in clips:
        name = clip["session"]
        with wave.open(str(parent / clip["wav"]), "rb") as src:
            if (src.getframerate(), src.getsampwidth(), src.getnchannels()) != (16000, 2, 1):
                raise ValueError("expected 16 kHz mono PCM16 WAV")
            pcm = np.frombuffer(src.readframes(src.getnframes()), dtype="<i2")
        # Legacy quantized frontend; clipping/lost packets are reflected in WAV.
        mel = frontend_mel(pcm)
        active = np.std(mel, axis=1) > 12
        for length in (20, 32, 40):
            for start in range(0, len(mel) - length + 1, 8):
                window = mel[start:start + length][active[start:start + length]]
                if len(window) < max(4, length // 4):
                    continue
                feature = np.concatenate((window.mean(axis=0), window.std(axis=0), window.max(axis=0)))
                for mode, count in (("mean", 32), ("mean_std_max", 96)):
                    q = feature[:count]
                    refs = profile_slots[:, :count]
                    qnorm = np.linalg.norm(q)
                    norms = np.linalg.norm(refs, axis=1) * qnorm
                    dists = 1.0 - (refs @ q) / np.maximum(norms, 1e-9)
                    d = float(dists.min())
                    rows.append({"session": name, "log": clip["log"], "label": clip["label"], "window_frames": length,
                                 "metric": mode, "first_sample": clip["start_sample"] + (start * 160),
                                 "active_frames": len(window), "distance": round(d, 5)})
    for length in (20, 32, 40):
        for mode in ("mean", "mean_std_max"):
            selected = [r for r in rows if r["window_frames"] == length and r["metric"] == mode]
            negative = [r["distance"] for r in selected if r["label"] in ("background", "tv", "speech", "motor")]
            if not negative:
                continue
            # Float-safe strict below minimum observed negative. Not a validated threshold.
            threshold = min(negative) - .00001
            stats = {}
            for session in by_session:
                member = [r for r in selected if r["session"] == session]
                stats[session] = {"windows": len(member), "below_threshold": sum(r["distance"] < threshold for r in member),
                                  "voted_2_of_3_updates": vote_hits(member, threshold),
                                  "median_distance": round(float(np.median([r["distance"] for r in member])), 4)
                                  if member else None}
            negative_leave_one_out = {}
            negative_names = [name for name in by_session if any(r["session"] == name and
                              r["label"] in ("background", "tv", "speech", "motor") for r in selected)]
            for held in negative_names:
                fitted = [r["distance"] for r in selected if r["session"] in
                          negative_names and r["session"] != held]
                if not fitted:
                    continue
                observed = [r["distance"] for r in selected if r["session"] == held]
                negative_leave_one_out[held] = {
                    "threshold_from_other_sessions": round(min(fitted), 5),
                    "false_windows_on_held_out": sum(d < min(fitted) for d in observed),
                    "voted_false_updates_on_held_out": vote_hits(
                        [r for r in selected if r["session"] == held], min(fitted))}
            best.append({"window_frames": length, "metric": mode, "in_sample_negative_min": min(negative),
                         "threshold_exclusive": threshold, "negative_leave_one_out": negative_leave_one_out,
                         "fixed_threshold_0_08": {session: {"windows": len(member := [r for r in selected if r["session"] == session]),
                                                   "accepted": sum(r["distance"] < .08 for r in member),
                                                   "voted_2_of_3": vote_hits(member, .08)}
                                                  for session in by_session},
                         "sessions": stats})
    out.mkdir(parents=True, exist_ok=True)
    with (out / "windows.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    report = {"target_type": "unspecified; profile is registered at runtime", "profile_peak_bin": target_bin,
              "majority_probe": majority_probe, "retained_samples": len(usable),
              "registration_leave_one_out_distance": support_loo,
              "threshold_warning": "Thresholds are chosen on the same negative sessions. Target-possible windows include silence; impact/similar sounds and unseen TARGET classes remain untested. Do NOT deploy these thresholds.",
              "candidates": best}
    (out / "comparison.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    return report


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", type=Path)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--extra-manifest", action="append", type=Path, default=[], help="independent PCM sessions")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--majority-probe", action="store_true", help="offline 3-of-5 profile comparison")
    args = parser.parse_args()
    report = scan(args.log, args.manifest, args.output, args.majority_probe, tuple(args.extra_manifest))
    print(json.dumps({"comparison": str(args.output / "comparison.json"),
                      "windows": str(args.output / "windows.csv"),
                      "candidates": len(report["candidates"]),
                      "sessions": sorted(report["candidates"][0]["sessions"])}))
