#!/usr/bin/env python3
"""ESP32S3 log-mel C実装とNumPy参照実装の固定ベクトル試験。"""

from __future__ import annotations

import ctypes
import pathlib
import subprocess
import sys
import unittest

import numpy as np

REPOSITORY_ROOT = pathlib.Path(__file__).resolve().parents[3]
TRAINER_ROOT = REPOSITORY_ROOT / "software" / "acoustic-trainer"
BUILD_DIR = TRAINER_ROOT / "build" / "log_mel"
SOURCE = REPOSITORY_ROOT / "firmware" / "esp32s3" / "src" / "log_mel_extractor.c"
INCLUDE_DIR = SOURCE.parent
LIBRARY = BUILD_DIR / ("liblog_mel.dylib" if sys.platform == "darwin" else "liblog_mel.so")
sys.path.insert(0, str(TRAINER_ROOT))

GOLDEN_FRAMES = {
    "silence": [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    "impulse": [-9, -8, -8, -7, -7, -6, -5, -5, -4, -4, -3, -3, -2, -1, -1, 0, 0, 1, 1, 2, 3, 3, 4, 4, 5, 5, 6, 7, 7, 8, 8, 9],
    "440_hz": [69, 79, 126, 127, 127, 127, 79, 55, 44, 32, 22, 13, 4, -3, -10, -16, -22, -29, -34, -40, -45, -50, -55, -60, -65, -70, -74, -78, -82, -85, -87, -89],
    "mixed": [-7, -5, 8, 18, 31, 65, 127, 127, 102, 41, 18, 2, -11, -21, -29, -35, -36, -25, -1, 99, 122, 93, -8, -33, -50, -62, -73, -81, -89, -95, -100, -103],
    "noise": [-20, -8, -15, -6, -8, -3, 3, -8, -11, -8, -8, -11, 1, -1, 0, 0, 3, 1, 8, 3, 3, 4, 8, 7, 3, 2, 5, 14, 12, 10, 10, 11],
}

from features import (  # noqa: E402
    MEL_BIN_COUNT,
    SAMPLE_RATE_HZ,
    WINDOW_SAMPLES,
    extract_log_mel_frame,
    extract_log_mel_stream,
    round_half_away_from_zero,
)


def _build_library() -> None:
    BUILD_DIR.mkdir(parents=True, exist_ok=True)
    link_mode = "-dynamiclib" if sys.platform == "darwin" else "-shared"
    subprocess.run(
        [
            "clang",
            "-std=c11",
            "-O2",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-ffp-contract=off",
            "-fPIC",
            link_mode,
            str(SOURCE),
            "-I",
            str(INCLUDE_DIR),
            "-lm",
            "-o",
            str(LIBRARY),
        ],
        check=True,
    )


def _fixed_noise(sample_count: int) -> np.ndarray:
    state = 0x13579BDF
    output = np.empty(sample_count, dtype=np.int32)
    for index in range(sample_count):
        state = ((1664525 * state) + 1013904223) & 0xFFFFFFFF
        output[index] = np.int32((state ^ 0x80000000) - 0x80000000) // 4
    return output


class LogMelCLibrary:
    def __init__(self) -> None:
        _build_library()
        self.library = ctypes.CDLL(str(LIBRARY))
        self.library.log_mel_extractor_context_size.restype = ctypes.c_size_t
        self.library.log_mel_extractor_init.argtypes = [ctypes.c_void_p]
        self.library.log_mel_extractor_self_test.argtypes = [ctypes.c_void_p]
        self.library.log_mel_extractor_self_test.restype = ctypes.c_bool
        self.library.log_mel_extractor_process_window.argtypes = [
            ctypes.c_void_p,
            ctypes.POINTER(ctypes.c_int32),
            ctypes.POINTER(ctypes.c_int8),
        ]
        self.callback_type = ctypes.CFUNCTYPE(None, ctypes.POINTER(ctypes.c_int8), ctypes.c_void_p)
        self.library.log_mel_extractor_feed.argtypes = [
            ctypes.c_void_p,
            ctypes.POINTER(ctypes.c_int32),
            ctypes.c_size_t,
            self.callback_type,
            ctypes.c_void_p,
        ]
        self.library.log_mel_extractor_feed.restype = ctypes.c_size_t

    def new_context(self) -> ctypes.Array[ctypes.c_char]:
        context = ctypes.create_string_buffer(self.library.log_mel_extractor_context_size())
        self.library.log_mel_extractor_init(context)
        return context

    def process_window(self, samples: np.ndarray) -> np.ndarray:
        pcm = np.ascontiguousarray(samples, dtype=np.int32)
        output = np.empty(MEL_BIN_COUNT, dtype=np.int8)
        context = self.new_context()
        self.library.log_mel_extractor_process_window(
            context,
            pcm.ctypes.data_as(ctypes.POINTER(ctypes.c_int32)),
            output.ctypes.data_as(ctypes.POINTER(ctypes.c_int8)),
        )
        return output

    def process_chunks(self, samples: np.ndarray, chunk_sizes: list[int]) -> np.ndarray:
        pcm = np.ascontiguousarray(samples, dtype=np.int32)
        context = self.new_context()
        frames: list[np.ndarray] = []

        @self.callback_type
        def collect(frame: ctypes.POINTER(ctypes.c_int8), _: ctypes.c_void_p) -> None:
            frames.append(np.ctypeslib.as_array(frame, shape=(MEL_BIN_COUNT,)).copy())

        offset = 0
        chunk_index = 0
        while offset < pcm.size:
            count = min(chunk_sizes[chunk_index % len(chunk_sizes)], pcm.size - offset)
            chunk = pcm[offset : offset + count]
            self.library.log_mel_extractor_feed(
                context,
                chunk.ctypes.data_as(ctypes.POINTER(ctypes.c_int32)),
                count,
                collect,
                None,
            )
            offset += count
            chunk_index += 1
        return np.stack(frames) if frames else np.empty((0, MEL_BIN_COUNT), dtype=np.int8)


class LogMelParityTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.c_library = LogMelCLibrary()

    def test_round_half_away_from_zero(self) -> None:
        values = np.array([-2.5, -1.5, -0.5, 0.5, 1.5, 2.5])
        np.testing.assert_array_equal(round_half_away_from_zero(values), [-3, -2, -1, 1, 2, 3])

    def test_embedded_self_test_passes(self) -> None:
        context = self.c_library.new_context()
        self.assertTrue(self.c_library.library.log_mel_extractor_self_test(context))

    def test_fixed_windows_are_bit_exact(self) -> None:
        sample = np.arange(WINDOW_SAMPLES, dtype=np.float64)
        amplitude = float(1 << 28)
        windows = {
            "silence": np.zeros(WINDOW_SAMPLES, dtype=np.int32),
            "impulse": np.pad(np.array([1 << 30], dtype=np.int32), (137, WINDOW_SAMPLES - 138)),
            "440_hz": np.rint(amplitude * np.sin(2.0 * np.pi * 440.0 * sample / SAMPLE_RATE_HZ)).astype(np.int32),
            "mixed": np.rint(
                amplitude
                * (
                    np.sin(2.0 * np.pi * 731.0 * sample / SAMPLE_RATE_HZ)
                    + (0.37 * np.sin(2.0 * np.pi * 2987.0 * sample / SAMPLE_RATE_HZ))
                )
            ).astype(np.int32),
            "noise": _fixed_noise(WINDOW_SAMPLES),
        }
        for name, window in windows.items():
            with self.subTest(name=name):
                firmware_reference = extract_log_mel_frame(window)
                golden = np.asarray(GOLDEN_FRAMES[name], dtype=np.int8)
                np.testing.assert_array_equal(firmware_reference, golden)
                np.testing.assert_array_equal(self.c_library.process_window(window), golden)

    def test_streaming_crosses_i2s_boundaries_without_drift(self) -> None:
        pcm = _fixed_noise(4_337)
        expected = extract_log_mel_stream(pcm)
        self.assertEqual(expected.shape[0], 25)
        for chunk_sizes in ([256], [1, 159, 160, 399, 17, 256], [511, 3, 97]):
            with self.subTest(chunk_sizes=chunk_sizes):
                actual = self.c_library.process_chunks(pcm, list(chunk_sizes))
                np.testing.assert_array_equal(actual, expected)

    def test_short_stream_does_not_emit(self) -> None:
        actual = self.c_library.process_chunks(_fixed_noise(WINDOW_SAMPLES - 1), [256])
        self.assertEqual(actual.shape, (0, MEL_BIN_COUNT))


if __name__ == "__main__":
    unittest.main(verbosity=2)
