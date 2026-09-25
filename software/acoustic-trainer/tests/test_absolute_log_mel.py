"""Embedded C and training reference agree for the optional energy-preserving frontend."""
from __future__ import annotations

import ctypes
import subprocess
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "software/acoustic-trainer"))
from train_fewshot import fast_frontend  # noqa: E402


def test_absolute_feature_matches_real_c_without_changing_centered(tmp_path: Path) -> None:
    source = ROOT / "firmware/esp32s3/src/log_mel_extractor.c"
    library_path = tmp_path / ("logmel.dylib" if sys.platform == "darwin" else "logmel.so")
    subprocess.run(["clang", "-std=c11", "-O2", "-Werror", "-ffp-contract=off",
                    "-fPIC", "-dynamiclib" if sys.platform == "darwin" else "-shared",
                    str(source), "-I", str(source.parent), "-lm", "-o", str(library_path)], check=True)
    lib = ctypes.CDLL(str(library_path))
    lib.log_mel_extractor_context_size.restype = ctypes.c_size_t
    lib.log_mel_extractor_init.argtypes = [ctypes.c_void_p]
    lib.log_mel_extractor_self_test.argtypes = [ctypes.c_void_p]
    lib.log_mel_extractor_self_test.restype = ctypes.c_bool
    lib.log_mel_extractor_process_window_pair.argtypes = [
        ctypes.c_void_p, ctypes.POINTER(ctypes.c_int32),
        ctypes.POINTER(ctypes.c_int8), ctypes.POINTER(ctypes.c_int8),
    ]
    context = ctypes.create_string_buffer(lib.log_mel_extractor_context_size())
    lib.log_mel_extractor_init(context)
    assert lib.log_mel_extractor_self_test(context)

    def extract(pcm: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        raw = np.ascontiguousarray(pcm.astype(np.int32) << 16)
        centered = (ctypes.c_int8 * 32)()
        absolute = (ctypes.c_int8 * 32)()
        lib.log_mel_extractor_process_window_pair(context,
            raw.ctypes.data_as(ctypes.POINTER(ctypes.c_int32)), centered, absolute)
        return np.array(centered[:], dtype=np.int8), np.array(absolute[:], dtype=np.int8)

    random = np.random.default_rng(72).integers(-5000, 5000, size=400, dtype=np.int16)
    centered, absolute = extract(random)
    np.testing.assert_array_equal(centered, fast_frontend(random, "centered")[0])
    np.testing.assert_allclose(absolute, fast_frontend(random, "absolute")[0], atol=1)

    phase = np.arange(400) * (2 * np.pi * 440 / 16000)
    quiet = np.rint(np.sin(phase) * 1024).astype(np.int16)
    loud = np.rint(np.sin(phase) * 8192).astype(np.int16)
    _, abs_quiet = extract(quiet)
    _, abs_loud = extract(loud)
    assert np.any(abs_loud.astype(np.int16) > abs_quiet.astype(np.int16) + 12)
