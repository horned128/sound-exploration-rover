#!/usr/bin/env python3
"""背景AEと能動フレーム照合のホスト参照回帰。"""

from __future__ import annotations

import pathlib
import sys
import unittest

import numpy as np

TRAINER_ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TRAINER_ROOT))

from acoustic_learning import (  # noqa: E402
    ACTIVE_FRAME_STDDEV_LSB,
    FEATURE_BINS,
    HIDDEN_UNITS,
    MIN_ACTIVE_FRAMES,
    RLS_FORGETTING_FACTOR,
    SUMMARY_DIMENSION,
    BackgroundModel,
    Identification,
    active_summary,
    cosine_distance,
    fixed_encoder,
    identify,
    is_active_frame,
    leave_one_out_threshold,
    normalize_feature,
)


def _active_frame(offset: int = 0) -> np.ndarray:
    """標準偏差が12 LSBを越える、決定論的な32-binフレームを作る。"""

    return np.clip(np.arange(-32, 32, 2, dtype=np.int16) + offset, -128, 127).astype(np.int8)


def _patch(active_count: int, offset: int = 0) -> np.ndarray:
    active = np.stack([_active_frame(offset + (index % 3)) for index in range(active_count)])
    inactive = np.zeros((80 - active_count, FEATURE_BINS), dtype=np.int8)
    return np.concatenate((active, inactive))


class BackgroundLearningReferenceTest(unittest.TestCase):
    def test_fixed_encoder_is_seed_reproducible_and_has_true_bottleneck(self) -> None:
        first_weights, first_bias = fixed_encoder()
        second_weights, second_bias = fixed_encoder()
        self.assertEqual(first_weights.shape, (HIDDEN_UNITS, FEATURE_BINS))
        self.assertEqual(first_bias.shape, (HIDDEN_UNITS,))
        self.assertLess(HIDDEN_UNITS, FEATURE_BINS)
        np.testing.assert_array_equal(first_weights, second_weights)
        np.testing.assert_array_equal(first_bias, second_bias)

    def test_normalization_clips_to_hard_sigmoid_input_domain(self) -> None:
        frame = np.full(FEATURE_BINS, -128, dtype=np.int8)
        frame[:5] = [-128, -64, 0, 64, 127]
        normalized = normalize_feature(frame)
        np.testing.assert_array_equal(normalized[:5], [0.0, 0.0, 0.5, 1.0, 1.0])
        self.assertTrue(np.all((0.0 <= normalized) & (normalized <= 1.0)))

    def test_rls_learns_background_but_active_frame_cannot_update_it(self) -> None:
        model = BackgroundModel.create()
        background = np.zeros(FEATURE_BINS, dtype=np.int8)
        initial_mse = model.mse(background)
        for _ in range(80):
            self.assertTrue(model.update_if_background(background))
        self.assertLess(model.mse(background), initial_mse * 0.02)

        decoder_before = model.decoder.copy()
        correlation_before = model.inverse_correlation.copy()
        self.assertFalse(model.update_if_background(_active_frame()))
        np.testing.assert_array_equal(model.decoder, decoder_before)
        np.testing.assert_array_equal(model.inverse_correlation, correlation_before)

    def test_rls_update_is_deterministic_with_the_fixed_forgetting_factor(self) -> None:
        self.assertEqual(RLS_FORGETTING_FACTOR, 0.95)
        left = BackgroundModel.create()
        right = BackgroundModel.create()
        sequence = [_active_frame(offset) for offset in (-10, 0, 10)]
        for frame in sequence:
            left.update(frame)
            right.update(frame)
        np.testing.assert_array_equal(left.decoder, right.decoder)
        np.testing.assert_array_equal(left.inverse_correlation, right.inverse_correlation)


class ActiveFrameIdentifierReferenceTest(unittest.TestCase):
    def test_active_frame_uses_population_stddev_and_strict_threshold(self) -> None:
        equal_threshold = np.zeros(FEATURE_BINS, dtype=np.int8)
        equal_threshold[:2] = [-48, 48]
        scaled = equal_threshold.astype(np.float64)
        scaled *= ACTIVE_FRAME_STDDEV_LSB / np.std(scaled, ddof=0)
        equal_threshold = np.rint(scaled).astype(np.int8)
        self.assertLessEqual(np.std(equal_threshold.astype(np.float64), ddof=0), ACTIVE_FRAME_STDDEV_LSB)
        self.assertFalse(is_active_frame(equal_threshold))
        self.assertTrue(is_active_frame(_active_frame()))

    def test_summary_is_96_int8_values_and_ignores_inactive_frames(self) -> None:
        source = _patch(MIN_ACTIVE_FRAMES)
        summary = active_summary(source)
        self.assertIsNotNone(summary)
        assert summary is not None
        self.assertEqual(summary.shape, (SUMMARY_DIMENSION,))
        self.assertEqual(summary.dtype, np.int8)
        expected_active = source[:MIN_ACTIVE_FRAMES]
        expected_mean = np.rint(np.mean(expected_active, axis=0)).astype(np.int8)
        np.testing.assert_array_equal(summary[:FEATURE_BINS], expected_mean)
        np.testing.assert_array_equal(summary[FEATURE_BINS : 2 * FEATURE_BINS], np.array([1] * FEATURE_BINS, dtype=np.int8))
        np.testing.assert_array_equal(summary[2 * FEATURE_BINS :], np.max(expected_active, axis=0))

    def test_insufficient_active_frames_is_indeterminate_not_not_target(self) -> None:
        samples = np.stack([active_summary(_patch(MIN_ACTIVE_FRAMES))] * 5)
        result = identify(_patch(MIN_ACTIVE_FRAMES - 1), samples)
        self.assertEqual(result.status, Identification.INDETERMINATE)
        self.assertEqual(result.active_frame_count, MIN_ACTIVE_FRAMES - 1)
        self.assertIsNone(result.minimum_distance)

    def test_leave_one_out_threshold_and_individual_sample_matching(self) -> None:
        reference = active_summary(_patch(MIN_ACTIVE_FRAMES, offset=0))
        assert reference is not None
        samples = np.stack([reference] * 5)
        self.assertEqual(leave_one_out_threshold(samples), 0.0)
        target = identify(_patch(MIN_ACTIVE_FRAMES, offset=0), samples)
        self.assertEqual(target.status, Identification.TARGET)
        self.assertEqual(target.minimum_distance, 0.0)

        non_target = identify(_patch(MIN_ACTIVE_FRAMES, offset=50), samples)
        self.assertEqual(non_target.status, Identification.NOT_TARGET)
        self.assertGreater(non_target.minimum_distance or 0.0, 0.0)

    def test_not_ready_and_zero_vector_are_never_silently_accepted(self) -> None:
        reference = active_summary(_patch(MIN_ACTIVE_FRAMES))
        assert reference is not None
        result = identify(_patch(MIN_ACTIVE_FRAMES), np.expand_dims(reference, axis=0))
        self.assertEqual(result.status, Identification.NOT_READY)
        self.assertIsNone(cosine_distance(np.zeros(SUMMARY_DIMENSION, np.int8), reference))


if __name__ == "__main__":
    unittest.main(verbosity=2)
