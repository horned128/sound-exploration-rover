"""Routing helpers for UDP-only diagnostic JSON records."""

PRIVATE_RECORD_TYPES = frozenset({"acoustic_diagnostic", "acoustic_sample"})


def is_private_diagnostic_record(parsed: object) -> bool:
    return isinstance(parsed, dict) and parsed.get("record_type") in PRIVATE_RECORD_TYPES
