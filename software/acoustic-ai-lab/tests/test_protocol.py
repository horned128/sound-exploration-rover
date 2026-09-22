import struct
import unittest

from protocol import (
    COMMAND_RESTART,
    COMMAND_LEARNING_START,
    MESSAGE_COMMAND,
    MESSAGE_SNAPSHOT,
    VERSION,
    FrameParser,
    decode_profile_chunk,
    decode_snapshot,
    decode_summary_chunk,
    encode_frame,
    make_command,
)


class ProtocolTest(unittest.TestCase):
    def test_wire_version_matches_cpu0_protocol(self) -> None:
        self.assertEqual(VERSION, 2)

    def test_fragmented_round_trip(self) -> None:
        wire = encode_frame(MESSAGE_COMMAND, 42, 1234, bytes([COMMAND_LEARNING_START]))
        parser = FrameParser()
        self.assertEqual(parser.feed(wire[:3]), [])
        frames = parser.feed(wire[3:11]) + parser.feed(wire[11:])
        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0].sequence, 42)
        self.assertEqual(frames[0].payload, bytes([COMMAND_LEARNING_START]))

    def test_crc_error_resynchronizes(self) -> None:
        damaged = bytearray(make_command(COMMAND_LEARNING_START, 4))
        damaged[-1] ^= 0x01
        parser = FrameParser()
        self.assertEqual(parser.feed(bytes(damaged)), [])
        self.assertEqual(parser.crc_errors, 1)
        self.assertEqual(len(parser.feed(make_command(COMMAND_LEARNING_START, 5))), 1)

    def test_restart_uses_a_distinct_command_code(self) -> None:
        parser = FrameParser()
        frames = parser.feed(make_command(COMMAND_RESTART, 6))
        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0].payload, bytes([COMMAND_RESTART]))

    def test_snapshot_fixed_point_fields(self) -> None:
        payload = bytearray(48)
        payload[0] = 1
        payload[1] = 0b1_1111
        payload[2] = 3
        payload[3] = 1
        struct.pack_into("<Hhh", payload, 4, 270, -1234, -876)
        payload[10:18] = bytes([1, 1, 4, 5, 7, 8, 2, 9])
        struct.pack_into("<HHHHH", payload, 18, 187, 200, 813, 25, 31)
        struct.pack_into("<IIII", payload, 28, 99, 101, 102, 103)
        payload[44] = 0
        decoded = decode_snapshot(bytes(payload))
        self.assertTrue(decoded["storage_valid"])
        self.assertEqual(decoded["doa_deg"], 270)
        self.assertEqual(decoded["level_dbfs"], -12.34)
        self.assertEqual(decoded["cosine_distance"], 0.187)
        self.assertEqual(decoded["feature_generation"], 101)

    def test_chunks_decode_signed_features(self) -> None:
        summary = struct.pack("<IBBBB", 12, 2, 3, 1, 0) + struct.pack("<64b", *range(-32, 32))
        profile = struct.pack("<IBBBB", 8, 4, 1, 3, 5) + struct.pack("<64b", *range(-32, 32))
        self.assertEqual(decode_summary_chunk(summary)["data"][0], -32)
        decoded = decode_profile_chunk(profile)
        self.assertEqual(decoded["sample_index"], 4)
        self.assertEqual(decoded["sample_count"], 5)


if __name__ == "__main__":
    unittest.main()
