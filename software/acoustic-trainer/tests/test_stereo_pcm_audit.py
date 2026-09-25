"""Channel pairing never treats a reboot or a missing UDP packet as stereo."""
from __future__ import annotations

import base64
import json
import struct
import sys
import wave
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from stereo_pcm_audit import analyze  # noqa: E402


def test_pair_by_sample_and_reboot_without_fabricating_channel(tmp_path: Path) -> None:
    path = tmp_path / "stereo.jsonl"
    def packet(boot: int, sample: int, channel: int, amplitude: int) -> dict:
        pcm = struct.pack("<4h", *([amplitude, -amplitude] * 2))
        return {"record_type": "acoustic_pcm" if channel == 0 else "acoustic_pcm_ch1",
                "schema": 1, "channel": channel, "first_sample": sample, "sample_count": 4,
                "esp_ms": 5000 if boot == 0 else 1000,
                "pcm16le_b64": base64.b64encode(pcm).decode()}
    rows = [packet(0, 80000, 0, 300), packet(0, 80000, 1, 100),
            packet(0, 80004, 0, 300),  # second channel lost on UDP
            packet(1, 0, 0, 500), packet(1, 0, 1, 500)]
    path.write_text("".join(json.dumps(row) + "\n" for row in rows), encoding="utf-8")
    report = analyze(path, tmp_path, min_clip_seconds=0)
    assert report["channel_packets"] == [3, 2]
    assert len(report["boots"]) == 2
    assert report["boots"][0]["paired_blocks"] == 1
    assert report["boots"][0]["unpaired_channel_0"] == 1
    assert report["boots"][1]["identical_sample_fraction"] == 1.0
    with wave.open(str(tmp_path / report["boots"][0]["stereo_wav_clips"][0]["file"])) as out:
        assert (out.getnchannels(), out.getsampwidth(), out.getframerate()) == (2, 2, 16000)
        assert out.getnframes() == 4
