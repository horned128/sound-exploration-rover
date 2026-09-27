import unittest

from diagnostics import background_autoencoder_update, is_private_diagnostic_record


class DiagnosticRoutingTests(unittest.TestCase):
    def test_feature_diagnostics_are_private_records(self) -> None:
        self.assertTrue(is_private_diagnostic_record({"record_type": "acoustic_diagnostic"}))
        self.assertTrue(is_private_diagnostic_record({"record_type": "acoustic_sample"}))
        self.assertTrue(is_private_diagnostic_record({"record_type": "acoustic_pcm"}))
        self.assertTrue(is_private_diagnostic_record({"record_type": "acoustic_pcm_ch1"}))

    def test_status_telemetry_stays_on_the_normal_web_route(self) -> None:
        self.assertFalse(is_private_diagnostic_record({"schema": 2, "think": {"state": 3}}))
        self.assertFalse(is_private_diagnostic_record(["not", "a", "record"]))

    def test_autoencoder_update_exposes_only_compact_background_metrics(self) -> None:
        update = background_autoencoder_update({
            "record_type": "acoustic_diagnostic",
            "feature_generation": 42,
            "background_mse": 0.012,
            "background_threshold": 0.02,
            "background_anomaly": False,
            "identifier_status": 4,
            "identifier_status_name": "TARGET",
            "summary": [1, 2, 3],
        })

        self.assertEqual(update, {
            "_type": "background_ae",
            "valid": True,
            "mse": 0.012,
            "threshold": 0.02,
            "anomaly": False,
            "feature_generation": 42,
            "match_status": "TARGET",
        })

    def test_autoencoder_update_maps_identifier_status_when_name_is_missing(self) -> None:
        update = background_autoencoder_update({
            "record_type": "acoustic_diagnostic",
            "feature_generation": 43,
            "background_mse": 0.004,
            "background_threshold": 0.063,
            "background_anomaly": False,
            "identifier_status": 1,
        })

        self.assertEqual(update["match_status"], "INDETERMINATE")
        self.assertEqual(update["feature_generation"], 43)

    def test_autoencoder_update_does_not_treat_u16_overflow_as_a_real_limit(self) -> None:
        update = background_autoencoder_update({
            "record_type": "acoustic_diagnostic",
            "feature_generation": 44,
            "identifier_status_name": "NOT_TARGET",
            "background_mse": 23.727,
            "background_threshold": -1,
            "background_anomaly": False,
            "background_status": "limit_out_of_range",
        })

        self.assertEqual(update, {
            "_type": "background_ae",
            "valid": False,
            "state": "limit_out_of_range",
            "anomaly": False,
            "feature_generation": 44,
            "match_status": "NOT_TARGET",
            "mse": 23.727,
        })

    def test_legacy_autoencoder_update_recognizes_saturated_limit(self) -> None:
        update = background_autoencoder_update({
            "record_type": "acoustic_diagnostic",
            "feature_generation": 45,
            "background_mse": 23.727,
            "background_threshold": 65.535,
            "background_anomaly": False,
        })

        self.assertEqual(update, {
            "_type": "background_ae",
            "valid": False,
            "state": "limit_out_of_range",
            "anomaly": False,
            "feature_generation": 45,
            "mse": 23.727,
        })

    def test_autoencoder_update_requires_a_trained_threshold(self) -> None:
        update = background_autoencoder_update({
            "record_type": "acoustic_diagnostic",
            "background_mse": -1,
            "background_threshold": -1,
            "background_anomaly": False,
        })

        self.assertIsNotNone(update)
        self.assertFalse(update["valid"])
        self.assertEqual(update["state"], "no_baseline")

    def test_old_diagnostic_record_reports_missing_autoencoder_metrics(self) -> None:
        update = background_autoencoder_update({
            "record_type": "acoustic_diagnostic",
            "summary": [1, 2, 3],
        })

        self.assertEqual(update, {
            "_type": "background_ae",
            "valid": False,
            "state": "not_reported",
        })


if __name__ == "__main__":
    unittest.main()
