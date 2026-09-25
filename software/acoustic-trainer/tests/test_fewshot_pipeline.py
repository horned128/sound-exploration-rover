"""Check the deployed frontend contract used by PCM-level mixture experiments."""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "software/audio_ml"))
sys.path.insert(0, str(ROOT / "software/acoustic-trainer"))

from short_window_probe import frontend_mel  # noqa: E402
from train_fewshot import fast_frontend, mix  # noqa: E402


def test_training_frontend_is_bit_exact_for_real_c_quantizer() -> None:
    rng = np.random.default_rng(732)
    signal = rng.integers(-4000, 4000, 16000, dtype=np.int16)
    np.testing.assert_array_equal(fast_frontend(signal), frontend_mel(signal).astype(np.int8))


def test_waveform_mixture_has_target_energy_at_requested_snr() -> None:
    rng = np.random.default_rng(951)
    target = rng.integers(-3000, 3000, 6640, dtype=np.int16)
    background = rng.integers(-1000, 1000, 6640, dtype=np.int16)
    mixed = mix(target, background, 0)
    assert len(mixed) == len(target)
    assert not np.array_equal(frontend_mel(mixed), frontend_mel(background))
