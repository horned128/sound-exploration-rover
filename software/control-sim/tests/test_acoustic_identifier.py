from __future__ import annotations

import ctypes
from controlsim.bindings import (
    CPU0_ACOUSTIC_FEATURE_BIN_COUNT,
    CPU0_ACOUSTIC_SUMMARY_DIMENSION,
    AcousticIdentifierSummaryOutput,
    library,
)


SUMMARY_INDETERMINATE = 1
SUMMARY_NOT_READY = 2
SUMMARY_NOT_TARGET = 3
SUMMARY_TARGET = 4


def feature_frame(offset: int = 0) -> ctypes.Array[ctypes.c_int8]:
    values = [max(-128, min(127, value + offset)) for value in range(-32, 32, 2)]
    assert len(values) == CPU0_ACOUSTIC_FEATURE_BIN_COUNT
    return (ctypes.c_int8 * CPU0_ACOUSTIC_FEATURE_BIN_COUNT)(*values)


def feature_patch(active_count: int, offset: int = 0) -> ctypes.Array[ctypes.c_int8]:
    values: list[int] = []
    for index in range(80):
        if index < active_count:
            values.extend(feature_frame(offset + (index % 3)))
        else:
            values.extend([0] * CPU0_ACOUSTIC_FEATURE_BIN_COUNT)
    return (ctypes.c_int8 * len(values))(*values)


def summary_from_patch(active_count: int, offset: int = 0) -> tuple[ctypes.Array[ctypes.c_int8], int, int]:
    patch = feature_patch(active_count, offset)
    summary = (ctypes.c_int8 * CPU0_ACOUSTIC_SUMMARY_DIMENSION)()
    active = ctypes.c_uint8()
    valid = library().acoustic_identifier_summary_create(patch, 80, summary, ctypes.byref(active))
    return summary, int(active.value), int(valid)


def test_active_frame_uses_strict_population_stddev_threshold() -> None:
    handle = library()
    boundary = (ctypes.c_int8 * CPU0_ACOUSTIC_FEATURE_BIN_COUNT)(-48, 48, *([0] * 30))
    assert handle.acoustic_identifier_frame_is_active(boundary) == 0
    assert handle.acoustic_identifier_frame_is_active(feature_frame()) == 1


def test_summary_has_mean_std_max_layout_and_rejects_insufficient_active_frames() -> None:
    summary, active_count, valid = summary_from_patch(10)
    assert (active_count, valid) == (10, 1)
    # Slot 1: 有効フレーム
    assert list(summary[:CPU0_ACOUSTIC_FEATURE_BIN_COUNT]) == [value + 1 for value in range(-32, 32, 2)]
    assert list(summary[CPU0_ACOUSTIC_FEATURE_BIN_COUNT : 2 * CPU0_ACOUSTIC_FEATURE_BIN_COUNT]) == [1] * 32
    assert list(summary[2 * CPU0_ACOUSTIC_FEATURE_BIN_COUNT : 3 * CPU0_ACOUSTIC_FEATURE_BIN_COUNT]) == [value + 2 for value in range(-32, 32, 2)]
    # Slot 2: 能動フレームなし（全ゼロ）
    assert list(summary[3 * CPU0_ACOUSTIC_FEATURE_BIN_COUNT :]) == [0] * (3 * CPU0_ACOUSTIC_FEATURE_BIN_COUNT)

    rejected, active_count, valid = summary_from_patch(9)
    assert (active_count, valid) == (9, 0)
    assert list(rejected) == [0] * CPU0_ACOUSTIC_SUMMARY_DIMENSION


def test_cosine_and_leave_one_out_are_individual_sample_metrics() -> None:
    handle = library()
    summary, _, valid = summary_from_patch(10)
    assert valid == 1
    distance = ctypes.c_float()
    assert handle.acoustic_identifier_cosine_distance(summary, summary, ctypes.byref(distance)) == 1
    assert distance.value == 0.0
    zero = (ctypes.c_int8 * CPU0_ACOUSTIC_SUMMARY_DIMENSION)()
    assert handle.acoustic_identifier_cosine_distance(zero, summary, ctypes.byref(distance)) == 0

    samples = (ctypes.c_int8 * (5 * CPU0_ACOUSTIC_SUMMARY_DIMENSION))(*list(summary) * 5)
    peak_bin = handle.acoustic_identifier_find_peak_bin(samples, 5)
    assert peak_bin == 31  # range(-32, 32, 2)の最大はbin 31

    weights = (ctypes.c_float * CPU0_ACOUSTIC_FEATURE_BIN_COUNT)()
    handle.acoustic_identifier_build_weights(peak_bin, weights)
    assert abs(weights[0] - 0.5) < 1e-5  # 低周波暗騒音抑制
    assert abs(weights[31] - 2.5) < 1e-5  # ピーク強調
    assert abs(weights[30] - 1.8) < 1e-5  # 隣接ビン

    threshold = ctypes.c_float()
    assert handle.acoustic_identifier_leave_one_out_threshold(samples, 5, None, ctypes.byref(threshold)) == 1
    # 下限クランプ (CPU0_ACOUSTIC_IDENTIFIER_THRESHOLD_MIN = 0.08)
    assert abs(threshold.value - 0.08) < 1e-4
    assert handle.acoustic_identifier_leave_one_out_threshold(samples, 1, None, ctypes.byref(threshold)) == 0


def test_slot_division_distinguishes_pulse_and_continuous_sound() -> None:
    handle = library()
    # 短音: 前半のみ15フレーム
    pulse_patch = feature_patch(15)
    s_pulse = (ctypes.c_int8 * CPU0_ACOUSTIC_SUMMARY_DIMENSION)()
    active = ctypes.c_uint8()
    assert handle.acoustic_identifier_summary_create(pulse_patch, 80, s_pulse, ctypes.byref(active)) == 1

    # 連続音: 前半15フレーム + 後半15フレーム (40〜54)
    values: list[int] = []
    for index in range(80):
        if (index < 15) or (40 <= index < 55):
            values.extend(feature_frame(index % 3))
        else:
            values.extend([0] * CPU0_ACOUSTIC_FEATURE_BIN_COUNT)
    cont_patch = (ctypes.c_int8 * len(values))(*values)
    s_cont = (ctypes.c_int8 * CPU0_ACOUSTIC_SUMMARY_DIMENSION)()
    assert handle.acoustic_identifier_summary_create(cont_patch, 80, s_cont, ctypes.byref(active)) == 1

    distance = ctypes.c_float()
    assert handle.acoustic_identifier_cosine_distance(s_pulse, s_cont, ctypes.byref(distance)) == 1
    # 短音と連続音は時間構造の違いにより有意に離れる（> 0.25）
    assert distance.value > 0.25


def test_summary_classification_keeps_indeterminate_and_not_ready_distinct_from_non_target() -> None:
    handle = library()
    reference, active_count, valid = summary_from_patch(10)
    samples = (ctypes.c_int8 * (5 * CPU0_ACOUSTIC_SUMMARY_DIMENSION))(*list(reference) * 5)
    threshold = ctypes.c_float()
    assert handle.acoustic_identifier_leave_one_out_threshold(samples, 5, None, ctypes.byref(threshold)) == 1

    output = AcousticIdentifierSummaryOutput()
    handle.acoustic_identifier_summary_classify(reference, valid, active_count, samples, 5, None, threshold.value, ctypes.byref(output))
    assert output.status == SUMMARY_TARGET
    assert output.minimum_cosine_distance == 0.0

    insufficient, short_active, short_valid = summary_from_patch(9)
    handle.acoustic_identifier_summary_classify(insufficient, short_valid, short_active, samples, 5, None, threshold.value, ctypes.byref(output))
    assert output.status == SUMMARY_INDETERMINATE

    handle.acoustic_identifier_summary_classify(reference, valid, active_count, samples, 1, None, threshold.value, ctypes.byref(output))
    assert output.status == SUMMARY_NOT_READY

    # 異なる周波数パターンの非対象音（低周波binにピーク）
    other_values: list[int] = []
    for index in range(80):
        if index < 10:
            frame = [0] * CPU0_ACOUSTIC_FEATURE_BIN_COUNT
            frame[3] = 80
            frame[4] = -80
            other_values.extend(frame)
        else:
            other_values.extend([0] * CPU0_ACOUSTIC_FEATURE_BIN_COUNT)
    other_patch = (ctypes.c_int8 * len(other_values))(*other_values)
    other = (ctypes.c_int8 * CPU0_ACOUSTIC_SUMMARY_DIMENSION)()
    other_active = ctypes.c_uint8()
    other_valid = handle.acoustic_identifier_summary_create(other_patch, 80, other, ctypes.byref(other_active))
    assert other_valid == 1

    handle.acoustic_identifier_summary_classify(other, other_valid, int(other_active.value), samples, 5, None, threshold.value, ctypes.byref(output))
    assert output.status == SUMMARY_NOT_TARGET
    assert output.minimum_cosine_distance > 0.2


def test_classification_enforces_peak_band_and_runtime_threshold_cap() -> None:
    handle = library()
    reference_values = [10] * CPU0_ACOUSTIC_SUMMARY_DIMENSION
    candidate_values = list(reference_values)
    for summary_index in (2 * CPU0_ACOUSTIC_FEATURE_BIN_COUNT, 5 * CPU0_ACOUSTIC_FEATURE_BIN_COUNT):
        reference_values[summary_index + 14] = 30
        candidate_values[summary_index + 1] = 30

    reference = (ctypes.c_int8 * CPU0_ACOUSTIC_SUMMARY_DIMENSION)(*reference_values)
    candidate = (ctypes.c_int8 * CPU0_ACOUSTIC_SUMMARY_DIMENSION)(*candidate_values)
    samples = (ctypes.c_int8 * (5 * CPU0_ACOUSTIC_SUMMARY_DIMENSION))(*list(reference) * 5)
    output = AcousticIdentifierSummaryOutput()

    # 旧MRAMに保存された0.220のしきい値でも、実機識別の受理上限0.055で照合する。
    handle.acoustic_identifier_summary_classify(candidate, 1, 80, samples, 5, None, 0.22, ctypes.byref(output))
    assert abs(output.threshold - 0.055) < 1e-4
    assert output.minimum_cosine_distance < 0.20
    assert output.status == SUMMARY_NOT_TARGET


def test_classification_accepts_the_measured_six_bin_peak_drift() -> None:
    """設置位置で生じる±6binのピーク移動は、同一音の照合を切らない。"""
    handle = library()
    reference_values = [10] * CPU0_ACOUSTIC_SUMMARY_DIMENSION
    candidate_values = list(reference_values)
    for summary_index in (2 * CPU0_ACOUSTIC_FEATURE_BIN_COUNT, 5 * CPU0_ACOUSTIC_FEATURE_BIN_COUNT):
        reference_values[summary_index + 14] = 30
        # スペクトル形状は近いまま、最大binだけが6つ移動した実測相当のケース。
        candidate_values[summary_index + 14] = 30
        candidate_values[summary_index + 8] = 31

    reference = (ctypes.c_int8 * CPU0_ACOUSTIC_SUMMARY_DIMENSION)(*reference_values)
    candidate = (ctypes.c_int8 * CPU0_ACOUSTIC_SUMMARY_DIMENSION)(*candidate_values)
    samples = (ctypes.c_int8 * (5 * CPU0_ACOUSTIC_SUMMARY_DIMENSION))(*list(reference) * 5)
    output = AcousticIdentifierSummaryOutput()

    handle.acoustic_identifier_summary_classify(candidate, 1, 80, samples, 5, None, 0.20, ctypes.byref(output))
    assert output.minimum_cosine_distance <= 0.055
    assert output.status == SUMMARY_TARGET


def test_isolated_training_sample_is_detected_and_ignored_for_matching() -> None:
    """Four matching prototypes identify one contaminating capture, even at the same peak bin."""
    handle = library()
    base, _, valid = summary_from_patch(80)
    assert valid == 1
    good = list(base)
    outlier = good.copy()
    for slot in range(2):
        for bin_index in range(CPU0_ACOUSTIC_FEATURE_BIN_COUNT):
            index = slot * 3 * CPU0_ACOUSTIC_FEATURE_BIN_COUNT + bin_index
            outlier[index] = -outlier[index]

    samples = (ctypes.c_int8 * (5 * CPU0_ACOUSTIC_SUMMARY_DIMENSION))(
        *(outlier + good * 4)
    )
    isolated = ctypes.c_uint32(99)
    assert handle.acoustic_identifier_isolated_sample_find(samples, 5, ctypes.byref(isolated)) == 1
    assert isolated.value == 0

    output = AcousticIdentifierSummaryOutput()
    query = (ctypes.c_int8 * CPU0_ACOUSTIC_SUMMARY_DIMENSION)(*outlier)
    handle.acoustic_identifier_summary_classify(
        query, 1, 80, samples, 5, None, 0.20, ctypes.byref(output)
    )
    assert output.status == SUMMARY_NOT_TARGET
    assert output.sample_index != 0

    coherent = (ctypes.c_int8 * (5 * CPU0_ACOUSTIC_SUMMARY_DIMENSION))(*(good * 5))
    assert handle.acoustic_identifier_isolated_sample_find(coherent, 5, ctypes.byref(isolated)) == 0
