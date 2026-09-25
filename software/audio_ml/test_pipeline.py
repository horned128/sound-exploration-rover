"""Checks real log replay and loss-aware PCM reconstruction."""
from __future__ import annotations

import base64
import json
import tempfile
import unittest
import wave
from pathlib import Path

from legacy_audit import ROOT, audit
from pcm_dataset import convert, pcm_runs, pcm_sessions


class PipelineTest(unittest.TestCase):
    def test_existing_profile_replays_baseline(self) -> None:
        path = ROOT / "software/rover-monitor/logs/rover-monitor-20260924-201746.jsonl"
        if not path.exists():
            self.skipTest("field logs unavailable")
        rows, report = audit([path])
        counts = report["files"][path.name]["counters"]
        self.assertEqual(counts["acoustic_sample"], 5)
        self.assertGreater(counts["replay_count"], 0)
        self.assertEqual(counts["replay_count"], counts["replay_agree"])
        self.assertEqual(len(rows), counts["acoustic_diagnostic"])
        self.assertIsNone(report["measured_recall_precision_fp_per_hour"])

    def test_pcm_gap_rejected_and_wav_exact(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "record.jsonl"
            with path.open("w", encoding="utf-8") as stream:
                for first, payload in [(0, b"\x01\x02\x03\x04"), (2, b"\x05\x06\x07\x08"),
                                       (8, b"\x09\x0a")]:
                    stream.write(json.dumps({"record_type": "acoustic_pcm", "schema": 1,
                                             "first_sample": first, "sample_count": len(payload) // 2,
                                             "pcm16le_b64": base64.b64encode(payload).decode()}) + "\n")
            runs, stats = pcm_runs(path)
            self.assertEqual(stats["missing_samples_between_runs"], 4)
            self.assertEqual(runs[0], (0, b"\x01\x02\x03\x04\x05\x06\x07\x08"))
            with self.assertRaisesRegex(ValueError, "missing audio"):
                convert([path], Path(tmp) / "clips", {path.name: [
                    {"start_s": 0, "end_s": 9 / 16000, "label": "target"}]})
            report = convert([path], Path(tmp) / "clips", {path.name: [
                {"start_s": 0, "end_s": 4 / 16000, "label": "target"}]})
            self.assertEqual(len(report["clips"]), 1)
            with wave.open(str(Path(tmp) / "clips" / report["clips"][0]["wav"])) as wav:
                self.assertEqual((wav.getframerate(), wav.getnframes()), (16000, 4))
                self.assertEqual(wav.readframes(4), runs[0][1])

    def test_rebooted_esp_reuses_sample_numbers_without_mixing_audio(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "restarted.jsonl"
            with path.open("w", encoding="utf-8") as stream:
                for first, ms, payload in [(0, 2400, b"\x01\x02\x03\x04"),
                                           (2, 2416, b"\x05\x06\x07\x08"),
                                           (0, 1200, b"\xf1\xf2\xf3\xf4"),
                                           (2, 1216, b"\xf5\xf6\xf7\xf8")]:
                    stream.write(json.dumps({"record_type": "acoustic_pcm", "schema": 1,
                                             "esp_ms": ms, "first_sample": first,
                                             "sample_count": len(payload) // 2,
                                             "pcm16le_b64": base64.b64encode(payload).decode()}) + "\n")
            sessions = pcm_sessions(path)
            self.assertEqual(len(sessions), 2)
            self.assertEqual(sessions[1]["runs"], [(0, b"\xf1\xf2\xf3\xf4\xf5\xf6\xf7\xf8")])
            with self.assertRaisesRegex(ValueError, "multiple ESP sessions"):
                pcm_runs(path)
            with self.assertRaisesRegex(ValueError, "session_index"):
                convert([path], Path(tmp) / "clips", {path.name: [
                    {"start_s": 0, "end_s": 4 / 16000, "label": "motor"}]})
            report = convert([path], Path(tmp) / "clips", {path.name: [
                {"start_s": 0, "end_s": 4 / 16000, "label": "motor", "session_index": 1}]})
            with wave.open(str(Path(tmp) / "clips" / report["clips"][0]["wav"])) as wav:
                self.assertEqual(wav.readframes(4), sessions[1]["runs"][0][1])


if __name__ == "__main__":
    unittest.main()
