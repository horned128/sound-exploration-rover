"""Replay the actual CPU0 five-shot 80-frame classifier on each I2S slot.

Uses identical C frontend, C summary and C nearest-sample/peak-gated decision
for both channels. TARGET is registered from five TARGET-only WAV segments.
Results are window-level, not annotated sound-event recall.
"""
from __future__ import annotations

import argparse
import ctypes
import json
import sys
from pathlib import Path

import numpy as np

from esc10_fewshot_eval import identifier, enroll_summary
from eval_stereo_query import sounds

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "software/audio_ml"))
from short_window_probe import frontend_mel  # noqa: E402


class Output(ctypes.Structure):
    _fields_ = [("active_frame_count", ctypes.c_uint8), ("sample_index", ctypes.c_uint32),
                ("minimum_cosine_distance", ctypes.c_float), ("threshold", ctypes.c_float),
                ("status", ctypes.c_int32)]


def setup(lib: ctypes.CDLL) -> None:
    i8 = ctypes.POINTER(ctypes.c_int8)
    lib.acoustic_identifier_consensus_mask.argtypes = [i8, ctypes.c_uint32]
    lib.acoustic_identifier_consensus_mask.restype = ctypes.c_uint8
    lib.acoustic_identifier_consensus_peak_bin.argtypes = [i8, ctypes.c_uint32]
    lib.acoustic_identifier_consensus_peak_bin.restype = ctypes.c_uint8
    lib.acoustic_identifier_leave_one_out_threshold.argtypes = [i8, ctypes.c_uint32,
        ctypes.POINTER(ctypes.c_float), ctypes.POINTER(ctypes.c_float)]
    lib.acoustic_identifier_leave_one_out_threshold.restype = ctypes.c_int32
    lib.acoustic_identifier_summary_classify.argtypes = [i8, ctypes.c_int32, ctypes.c_uint8,
        i8, ctypes.c_uint32, ctypes.POINTER(ctypes.c_float), ctypes.c_float, ctypes.POINTER(Output)]


def evaluate(folders: dict[str, Path], output: Path) -> dict:
    output.mkdir(parents=True, exist_ok=True)
    lib = identifier(output)
    setup(lib)
    result = {"method": "firmware CPU0 80x32->192D, five exemplars, cosine + peak gate",
              "profile_source": "new TARGET-only stereo session, five independent available WAV runs",
              "channels": {}}
    for channel in ("left", "right"):
        clips = {name: [frontend_mel(pcm) for pcm in sounds(folder, channel)]
                 for name, folder in folders.items()}
        support = [enroll_summary(mel, lib) for mel in clips["target"]]
        if len(support) < 5 or any(s is None for s in support[:5]):
            result["channels"][channel] = {"registration": "incomplete", "valid_samples": sum(s is not None for s in support)}
            continue
        samples = (ctypes.c_int8 * (5 * 192))(*[n for s in support[:5] for n in s])
        threshold = ctypes.c_float()
        if not lib.acoustic_identifier_leave_one_out_threshold(samples, 5, None, ctypes.byref(threshold)):
            raise ValueError("failed to calibrate C five-shot threshold")
        tested = {"profile_mask": int(lib.acoustic_identifier_consensus_mask(samples, 5)),
                  "profile_peak_bin": int(lib.acoustic_identifier_consensus_peak_bin(samples, 5)),
                  "stored_threshold": round(threshold.value, 5), "effective_threshold_max": .055,
                  "hops": {}}
        for hop in (40, 80):
            conditions = {}
            for name, recordings in clips.items():
                count = accepted = indeterminate = 0
                distances = []
                for recording in recordings:
                    for start in range(0, len(recording) - 80 + 1, hop):
                        patch = np.ascontiguousarray(recording[start:start + 80], dtype=np.int8)
                        summary = (ctypes.c_int8 * 192)()
                        active = ctypes.c_uint8()
                        valid = lib.acoustic_identifier_summary_create(
                            patch.ctypes.data_as(ctypes.POINTER(ctypes.c_int8)),
                            80, summary, ctypes.byref(active))
                        decision = Output()
                        lib.acoustic_identifier_summary_classify(summary, valid, active, samples, 5,
                            None, threshold, ctypes.byref(decision))
                        count += 1
                        accepted += decision.status == 4
                        indeterminate += decision.status == 1
                        if decision.minimum_cosine_distance >= 0:
                            distances.append(float(decision.minimum_cosine_distance))
                conditions[name] = {"windows": count, "TARGET": accepted,
                                    "INDETERMINATE": indeterminate,
                                    "median_distance": round(float(np.median(distances)), 4) if distances else None}
            tested["hops"][str(hop)] = conditions
        result["channels"][channel] = tested
    (output / "legacy_stereo.json").write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2))
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("silence", "target", "target_noise", "motor"):
        parser.add_argument("--" + name.replace("_", "-"), required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    evaluate({name: getattr(args, name) for name in ("silence", "target", "target_noise", "motor")}, args.output)
