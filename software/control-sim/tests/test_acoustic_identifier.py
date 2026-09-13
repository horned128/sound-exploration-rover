from __future__ import annotations

import ctypes
import random

from controlsim.bindings import (
    CPU0_ACOUSTIC_EMBEDDING_DIMENSION,
    AcousticIdentifierOutput,
    AcousticIdentifierPrototype,
    library,
)


UINT32_MAX = 0xFFFFFFFF


def embedding(values: list[int]) -> ctypes.Array[ctypes.c_int8]:
    assert len(values) == CPU0_ACOUSTIC_EMBEDDING_DIMENSION
    return (ctypes.c_int8 * CPU0_ACOUSTIC_EMBEDDING_DIMENSION)(*values)


def prototype(
    values: list[int], *, class_id: int, threshold: int, valid: bool = True
) -> AcousticIdentifierPrototype:
    result = AcousticIdentifierPrototype()
    result.embedding[:] = values
    result.max_squared_distance = threshold
    result.class_id = class_id
    result.valid = valid
    return result


def classify(
    query: ctypes.Array[ctypes.c_int8], prototypes: list[AcousticIdentifierPrototype]
) -> AcousticIdentifierOutput:
    prototype_array = (AcousticIdentifierPrototype * len(prototypes))(*prototypes)
    output = AcousticIdentifierOutput()
    library().acoustic_identifier_classify(query, prototype_array, len(prototypes), ctypes.byref(output))
    return output


def test_squared_l2_matches_integer_reference_for_full_int8_range() -> None:
    rng = random.Random(20260913)
    handle = library()

    cases = [([-128] * 64, [127] * 64), ([127] * 64, [-128] * 64)]
    cases.extend(
        (
            [rng.randint(-128, 127) for _ in range(64)],
            [rng.randint(-128, 127) for _ in range(64)],
        )
        for _ in range(250)
    )

    for left_values, right_values in cases:
        expected = sum((left - right) ** 2 for left, right in zip(left_values, right_values, strict=True))
        actual = handle.acoustic_identifier_squared_l2(embedding(left_values), embedding(right_values))
        assert actual == expected


def test_classify_ignores_invalid_entries_and_applies_nearest_threshold() -> None:
    query = embedding([0] * 64)
    prototypes = [
        prototype([0] * 64, class_id=1, threshold=0, valid=False),
        prototype([1] * 64, class_id=7, threshold=64),
        prototype([2] * 64, class_id=9, threshold=1024),
    ]

    output = classify(query, prototypes)

    assert output.prototype_index == 1
    assert output.class_id == 7
    assert output.squared_distance == 64
    assert output.matched == 1


def test_classify_rejects_distance_above_threshold_but_keeps_nearest_diagnostics() -> None:
    query = embedding([0] * 64)
    output = classify(query, [prototype([1] * 64, class_id=3, threshold=63)])

    assert output.prototype_index == 0
    assert output.class_id == 3
    assert output.squared_distance == 64
    assert output.matched == 0


def test_classify_uses_first_prototype_for_equal_distance() -> None:
    query = embedding([0] * 64)
    prototypes = [
        prototype([1] * 64, class_id=4, threshold=64),
        prototype([-1] * 64, class_id=5, threshold=64),
    ]

    output = classify(query, prototypes)

    assert output.prototype_index == 0
    assert output.class_id == 4
    assert output.matched == 1


def test_classify_returns_explicit_no_match_when_no_prototype_is_valid() -> None:
    query = embedding([0] * 64)
    output = classify(query, [prototype([0] * 64, class_id=1, threshold=0, valid=False)])

    assert output.prototype_index == UINT32_MAX
    assert output.class_id == 0xFF
    assert output.squared_distance == UINT32_MAX
    assert output.matched == 0


def test_null_distance_inputs_fail_closed() -> None:
    handle = library()
    vector = embedding([0] * 64)

    assert handle.acoustic_identifier_squared_l2(None, vector) == UINT32_MAX
    assert handle.acoustic_identifier_squared_l2(vector, None) == UINT32_MAX
