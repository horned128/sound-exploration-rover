from __future__ import annotations

import ctypes

import pytest

from controlsim.bindings import (
    BOOL,
    CPU0_BACKGROUND_MODEL_HIDDEN_DIMENSION,
    CPU0_BACKGROUND_MODEL_INPUT_DIMENSION,
    BackgroundModelState,
    library,
)


def frame(values: list[int]) -> ctypes.Array[ctypes.c_int8]:
    assert len(values) == CPU0_BACKGROUND_MODEL_INPUT_DIMENSION
    return (ctypes.c_int8 * CPU0_BACKGROUND_MODEL_INPUT_DIMENSION)(*values)


def initialized_model() -> BackgroundModelState:
    state = BackgroundModelState()
    library().background_model_init(ctypes.byref(state), 0)
    return state


def test_initialization_has_fixed_seed_zero_decoder_and_i_over_point_zero_one() -> None:
    state = initialized_model()
    assert state.encoder_seed == 0x53455231
    assert list(state.decoder) == [0.0] * (CPU0_BACKGROUND_MODEL_INPUT_DIMENSION * CPU0_BACKGROUND_MODEL_HIDDEN_DIMENSION)
    for row in range(CPU0_BACKGROUND_MODEL_HIDDEN_DIMENSION):
        for column in range(CPU0_BACKGROUND_MODEL_HIDDEN_DIMENSION):
            assert state.inverse_correlation[(row * CPU0_BACKGROUND_MODEL_HIDDEN_DIMENSION) + column] == (100.0 if row == column else 0.0)


def test_background_rls_reduces_error_and_builds_mean_plus_three_sigma_threshold() -> None:
    handle = library()
    state = initialized_model()
    background = frame([0] * CPU0_BACKGROUND_MODEL_INPUT_DIMENSION)
    initial_mse = ctypes.c_float()
    assert handle.background_model_mse(ctypes.byref(state), background, ctypes.byref(initial_mse)) == 1
    observed = ctypes.c_float()
    for _ in range(80):
        assert handle.background_model_observe(ctypes.byref(state), background, BOOL(0), ctypes.byref(observed)) == 1
    learned_mse = ctypes.c_float()
    assert handle.background_model_mse(ctypes.byref(state), background, ctypes.byref(learned_mse)) == 1
    assert learned_mse.value < initial_mse.value * 0.02
    threshold = ctypes.c_float()
    assert handle.background_model_mse_threshold(ctypes.byref(state), ctypes.byref(threshold)) == 1
    assert threshold.value >= 0.0
    assert state.mse_count == 80


def test_active_frame_is_scored_but_never_updates_decoder_or_inverse_correlation() -> None:
    handle = library()
    state = initialized_model()
    background = frame([0] * CPU0_BACKGROUND_MODEL_INPUT_DIMENSION)
    mse = ctypes.c_float()
    for _ in range(10):
        assert handle.background_model_observe(ctypes.byref(state), background, BOOL(0), ctypes.byref(mse)) == 1
    decoder_before = list(state.decoder)
    correlation_before = list(state.inverse_correlation)
    active = frame(list(range(-32, 32, 2)))
    assert handle.background_model_observe(ctypes.byref(state), active, BOOL(1), ctypes.byref(mse)) == 1
    assert list(state.decoder) == decoder_before
    assert list(state.inverse_correlation) == correlation_before
    assert state.mse_count == 10


def test_invalid_arguments_and_restored_decoder_reset_path_fail_closed() -> None:
    handle = library()
    state = initialized_model()
    output = ctypes.c_float()
    background = frame([0] * CPU0_BACKGROUND_MODEL_INPUT_DIMENSION)
    assert handle.background_model_mse(None, background, ctypes.byref(output)) == 0
    assert handle.background_model_mse(ctypes.byref(state), None, ctypes.byref(output)) == 0
    assert handle.background_model_mse_threshold(ctypes.byref(state), ctypes.byref(output)) == 0
    state.decoder[0] = 3.0
    state.inverse_correlation[1] = 2.0
    handle.background_model_reset_inverse_correlation(ctypes.byref(state))
    assert state.decoder[0] == 3.0
    assert state.inverse_correlation[1] == 0.0
    assert state.inverse_correlation[0] == 100.0
    assert state.inverse_correlation[CPU0_BACKGROUND_MODEL_HIDDEN_DIMENSION + 1] == 100.0


@pytest.mark.parametrize("seed", [1, 0x53455231, 0xFFFFFFFF])
def test_same_seed_and_observations_produce_identical_state(seed: int) -> None:
    handle = library()
    left = BackgroundModelState()
    right = BackgroundModelState()
    handle.background_model_init(ctypes.byref(left), seed)
    handle.background_model_init(ctypes.byref(right), seed)
    output = ctypes.c_float()
    for offset in (-10, 0, 10):
        values = [max(-64, min(64, value + offset)) for value in range(-20, 44, 2)]
        assert handle.background_model_observe(ctypes.byref(left), frame(values), BOOL(0), ctypes.byref(output)) == 1
        assert handle.background_model_observe(ctypes.byref(right), frame(values), BOOL(0), ctypes.byref(output)) == 1
    assert bytes(left) == bytes(right)
