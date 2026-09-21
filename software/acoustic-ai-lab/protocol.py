"""Wire protocol for the direct J11 Acoustic AI Lab USB CDC link.

This module intentionally has no serial or web dependency so recorded traffic can
be decoded and regression-tested independently of the dashboard.
"""

from __future__ import annotations

from dataclasses import dataclass
import struct
from typing import Iterable

MAGIC = b"SR"
VERSION = 1
HEADER_SIZE = 14
CRC_SIZE = 2
MAX_PAYLOAD_SIZE = 96

MESSAGE_SNAPSHOT = 0x30
MESSAGE_SUMMARY_CHUNK = 0x31
MESSAGE_COMMAND = 0x32
MESSAGE_COMMAND_RESULT = 0x33
MESSAGE_PROFILE_CHUNK = 0x34

COMMAND_STATUS = 0
COMMAND_LEARNING_START = 1
COMMAND_LEARNING_COMMIT = 2
COMMAND_LEARNING_CANCEL = 3
COMMAND_PROFILE_READ = 4

SNAPSHOT_FLAGS = {
    "link_ready": 1 << 0,
    "summary_valid": 1 << 1,
    "background_anomaly": 1 << 2,
    "learning_active": 1 << 3,
    "storage_valid": 1 << 4,
}


@dataclass(frozen=True)
class Frame:
    message_type: int
    sequence: int
    uptime_ms: int
    payload: bytes


def crc16_ccitt_false(data: bytes) -> int:
    """CRC-16/CCITT-FALSE, matching firmware/common/acoustic_protocol.c."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def encode_frame(message_type: int, sequence: int, uptime_ms: int, payload: bytes = b"") -> bytes:
    if not 0 < message_type < 256:
        raise ValueError("message_type must be a nonzero byte")
    if len(payload) > MAX_PAYLOAD_SIZE:
        raise ValueError("payload is too large")
    header = struct.pack("<2sBBHII", MAGIC, VERSION, message_type, len(payload), sequence, uptime_ms)
    body = header + payload
    return body + struct.pack("<H", crc16_ccitt_false(body[2:]))


class FrameParser:
    """Incremental parser which recovers after malformed serial input."""

    def __init__(self) -> None:
        self._buffer = bytearray()
        self.crc_errors = 0
        self.format_errors = 0

    def feed(self, data: bytes) -> list[Frame]:
        self._buffer.extend(data)
        frames: list[Frame] = []
        while True:
            magic_at = self._buffer.find(MAGIC)
            if magic_at < 0:
                self._buffer[:] = self._buffer[-1:] if self._buffer.endswith(MAGIC[:1]) else b""
                return frames
            if magic_at:
                del self._buffer[:magic_at]
            if len(self._buffer) < HEADER_SIZE:
                return frames
            version = self._buffer[2]
            payload_length = struct.unpack_from("<H", self._buffer, 4)[0]
            if version != VERSION or payload_length > MAX_PAYLOAD_SIZE:
                self.format_errors += 1
                del self._buffer[0]
                continue
            frame_length = HEADER_SIZE + payload_length + CRC_SIZE
            if len(self._buffer) < frame_length:
                return frames
            raw = bytes(self._buffer[:frame_length])
            actual_crc = struct.unpack_from("<H", raw, frame_length - CRC_SIZE)[0]
            if actual_crc != crc16_ccitt_false(raw[2:-CRC_SIZE]):
                self.crc_errors += 1
                del self._buffer[0]
                continue
            frames.append(
                Frame(
                    message_type=raw[3],
                    sequence=struct.unpack_from("<I", raw, 6)[0],
                    uptime_ms=struct.unpack_from("<I", raw, 10)[0],
                    payload=raw[HEADER_SIZE:-CRC_SIZE],
                )
            )
            del self._buffer[:frame_length]


def _require(payload: bytes, length: int, kind: str) -> None:
    if len(payload) != length:
        raise ValueError(f"{kind} payload length is {len(payload)}, expected {length}")


def decode_snapshot(payload: bytes) -> dict[str, object]:
    """Decode the 48-byte inference/learning status sample."""
    _require(payload, 48, "snapshot")
    flags = payload[1]
    return {
        "schema_version": payload[0],
        "flags": flags,
        "link_ready": bool(flags & SNAPSHOT_FLAGS["link_ready"]),
        "summary_valid": bool(flags & SNAPSHOT_FLAGS["summary_valid"]),
        "background_anomaly": bool(flags & SNAPSHOT_FLAGS["background_anomaly"]),
        "learning_active": bool(flags & SNAPSHOT_FLAGS["learning_active"]),
        "storage_valid": bool(flags & SNAPSHOT_FLAGS["storage_valid"]),
        "think_state": payload[2],
        "infer_status": payload[3],
        "doa_deg": struct.unpack_from("<H", payload, 4)[0],
        "level_dbfs": struct.unpack_from("<h", payload, 6)[0] / 100.0,
        "peak_dbfs": struct.unpack_from("<h", payload, 8)[0] / 100.0,
        "vad": bool(payload[10]),
        "xvf_status": payload[11],
        "audio_flags": payload[12],
        "learning_samples": payload[13],
        "target_peak_bin": payload[14],
        "current_peak_bin": payload[15],
        "nearest_sample": payload[16],
        "active_frame_count": payload[17],
        "cosine_distance": struct.unpack_from("<H", payload, 18)[0] / 1000.0,
        "identifier_threshold": struct.unpack_from("<H", payload, 20)[0] / 1000.0,
        "similarity_permille": struct.unpack_from("<H", payload, 22)[0],
        "background_mse": struct.unpack_from("<H", payload, 24)[0] / 1000.0,
        "background_threshold": struct.unpack_from("<H", payload, 26)[0] / 1000.0,
        "observation_sequence": struct.unpack_from("<I", payload, 28)[0],
        "feature_generation": struct.unpack_from("<I", payload, 32)[0],
        "inference_count": struct.unpack_from("<I", payload, 36)[0],
        "match_count": struct.unpack_from("<I", payload, 40)[0],
        "storage_result": payload[44],
    }


def decode_summary_chunk(payload: bytes) -> dict[str, object]:
    _require(payload, 72, "summary chunk")
    return {
        "feature_generation": struct.unpack_from("<I", payload)[0],
        "chunk_index": payload[4],
        "chunk_count": payload[5],
        "summary_valid": bool(payload[6]),
        "data": list(struct.unpack("<64b", payload[8:])),
    }


def decode_profile_chunk(payload: bytes) -> dict[str, object]:
    _require(payload, 72, "profile chunk")
    return {
        "storage_generation": struct.unpack_from("<I", payload)[0],
        "sample_index": payload[4],
        "chunk_index": payload[5],
        "chunk_count": payload[6],
        "sample_count": payload[7],
        "data": list(struct.unpack("<64b", payload[8:])),
    }


def decode_command_result(payload: bytes) -> dict[str, int]:
    _require(payload, 4, "command result")
    return {"command": payload[0], "result": payload[1], "learning_active": payload[2], "learning_samples": payload[3]}


def make_command(command: int, sequence: int, uptime_ms: int = 0) -> bytes:
    if command not in range(5):
        raise ValueError("unknown AI Lab command")
    return encode_frame(MESSAGE_COMMAND, sequence, uptime_ms, bytes([command]))


def frame_records(data: bytes) -> Iterable[Frame]:
    """Convenience iterator used by offline tools and tests."""
    yield from FrameParser().feed(data)
