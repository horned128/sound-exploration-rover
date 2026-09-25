import unittest

from diagnostics import is_private_diagnostic_record


class DiagnosticRoutingTests(unittest.TestCase):
    def test_feature_diagnostics_are_private_records(self) -> None:
        self.assertTrue(is_private_diagnostic_record({"record_type": "acoustic_diagnostic"}))
        self.assertTrue(is_private_diagnostic_record({"record_type": "acoustic_sample"}))
        self.assertTrue(is_private_diagnostic_record({"record_type": "acoustic_pcm"}))
        self.assertTrue(is_private_diagnostic_record({"record_type": "acoustic_pcm_ch1"}))

    def test_status_telemetry_stays_on_the_normal_web_route(self) -> None:
        self.assertFalse(is_private_diagnostic_record({"schema": 2, "think": {"state": 3}}))
        self.assertFalse(is_private_diagnostic_record(["not", "a", "record"]))


if __name__ == "__main__":
    unittest.main()
