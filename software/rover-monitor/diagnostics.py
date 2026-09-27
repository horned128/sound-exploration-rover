"""Routing helpers for UDP-only diagnostic JSON records."""

import math

PRIVATE_RECORD_TYPES = frozenset({"acoustic_diagnostic", "acoustic_sample", "acoustic_pcm", "acoustic_pcm_ch1"})
IDENTIFIER_STATUS_NAMES = ("INVALID", "INDETERMINATE", "NOT_READY", "NOT_TARGET", "TARGET")
BACKGROUND_METRIC_U16_MAX = 65.535


def is_private_diagnostic_record(parsed: object) -> bool:
    return isinstance(parsed, dict) and parsed.get("record_type") in PRIVATE_RECORD_TYPES


def background_autoencoder_update(parsed: object) -> dict[str, object] | None:
    """Return only the background-autoencoder summary for the live dashboard."""
    if not isinstance(parsed, dict) or parsed.get("record_type") != "acoustic_diagnostic":
        return None

    context: dict[str, object] = {}
    generation = parsed.get("feature_generation")
    if isinstance(generation, int) and not isinstance(generation, bool):
        context["feature_generation"] = generation

    status_name = parsed.get("identifier_status_name")
    status_name = status_name.strip().upper() if isinstance(status_name, str) else None
    if status_name not in IDENTIFIER_STATUS_NAMES:
        status = parsed.get("identifier_status")
        if isinstance(status, int) and not isinstance(status, bool) and 0 <= status < len(IDENTIFIER_STATUS_NAMES):
            status_name = IDENTIFIER_STATUS_NAMES[status]
        else:
            status_name = None
    if status_name is not None:
        context["match_status"] = status_name

    background_status = parsed.get("background_status")
    if isinstance(background_status, str) and background_status in {
        "no_mse_data",
        "no_baseline",
        "limit_out_of_range",
        "mse_out_of_range",
        "mse_and_limit_out_of_range",
    }:
        update: dict[str, object] = {
            "_type": "background_ae",
            "valid": False,
            "state": background_status,
            "anomaly": parsed.get("background_anomaly") is True,
            **context,
        }
        mse = parsed.get("background_mse")
        if isinstance(mse, (int, float)) and not isinstance(mse, bool) and math.isfinite(mse) and mse >= 0:
            update["mse"] = float(mse)
        threshold = parsed.get("background_threshold")
        if (isinstance(threshold, (int, float)) and not isinstance(threshold, bool) and
                math.isfinite(threshold) and threshold > 0):
            update["threshold"] = float(threshold)
        return update

    mse = parsed.get("background_mse")
    threshold = parsed.get("background_threshold")
    if not isinstance(mse, (int, float)) or isinstance(mse, bool):
        return {"_type": "background_ae", "valid": False, "state": "not_reported", **context}
    if not isinstance(threshold, (int, float)) or isinstance(threshold, bool):
        return {"_type": "background_ae", "valid": False, "state": "not_reported", **context}

    # Legacy ESP32 firmware cannot distinguish this saturated x1000 value from a
    # valid metric because it does not send background_status.
    mse_overflow = math.isfinite(mse) and mse == BACKGROUND_METRIC_U16_MAX
    threshold_overflow = math.isfinite(threshold) and threshold == BACKGROUND_METRIC_U16_MAX
    if mse_overflow or threshold_overflow:
        state = (
            "mse_and_limit_out_of_range" if mse_overflow and threshold_overflow else
            "mse_out_of_range" if mse_overflow else "limit_out_of_range"
        )
        update = {
            "_type": "background_ae",
            "valid": False,
            "state": state,
            "anomaly": parsed.get("background_anomaly") is True,
            **context,
        }
        if not mse_overflow and math.isfinite(mse) and mse >= 0:
            update["mse"] = float(mse)
        if not threshold_overflow and math.isfinite(threshold) and threshold > 0:
            update["threshold"] = float(threshold)
        return update

    valid = math.isfinite(mse) and math.isfinite(threshold) and mse >= 0 and threshold > 0
    if not valid:
        return {"_type": "background_ae", "valid": False, "state": "no_baseline", **context}

    return {
        "_type": "background_ae",
        "valid": True,
        "mse": float(mse),
        "threshold": float(threshold),
        "anomaly": parsed.get("background_anomaly") is True,
        **context,
    }
