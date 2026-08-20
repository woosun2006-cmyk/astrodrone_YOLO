#!/usr/bin/env python3
import struct
import unittest
import zlib
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from video_protocol import HEADER, MAGIC, VERSION, DATA, PARITY, VideoReassembler


def packet(kind, sequence, index, count, payload, chunk_size=8, timestamp_ns=123):
    padded = payload.ljust(chunk_size, b"\0")
    header = HEADER.pack(MAGIC, VERSION, kind, 0, sequence, timestamp_ns,
                         index, count, len(payload), chunk_size,
                         zlib.crc32(padded) & 0xFFFFFFFF)
    return header + padded


class VideoProtocolTest(unittest.TestCase):
    def test_out_of_order_chunks_reassemble(self):
        chunks = [b"\xff\xd8JPE", b"G-DATA!!", b"\xff\xd9"]
        parity = bytes(a ^ b ^ c for a, b, c in zip(
            chunks[0].ljust(8, b"\0"), chunks[1].ljust(8, b"\0"), chunks[2].ljust(8, b"\0")))
        decoder = VideoReassembler()
        result = None
        for raw in (packet(DATA, 4, 2, 3, chunks[2]),
                    packet(DATA, 4, 0, 3, chunks[0]),
                    packet(DATA, 4, 1, 3, chunks[1])):
            result = decoder.add_packet(raw)
        self.assertIsNotNone(result)
        self.assertEqual(result["jpeg"], b"\xff\xd8JPEG-DATA!!\xff\xd9")

    def test_single_missing_chunk_is_recovered_by_xor_parity(self):
        chunks = [b"\xff\xd8JPE", b"G-DATA!!", b"\xff\xd9"]
        padded = [chunk.ljust(8, b"\0") for chunk in chunks]
        parity = bytes(a ^ b ^ c for a, b, c in zip(*padded))
        decoder = VideoReassembler()
        result = None
        for raw in (packet(DATA, 5, 0, 3, chunks[0]),
                    packet(DATA, 5, 2, 3, chunks[2]),
                    packet(PARITY, 5, 0xFFFF, 3, parity)):
            result = decoder.add_packet(raw)
        self.assertIsNotNone(result)
        self.assertEqual(result["jpeg"], b"\xff\xd8JPEG-DATA!!\xff\xd9")
        self.assertEqual(decoder.recovered, 1)

    def test_old_frame_is_not_allowed_to_overwrite_latest(self):
        decoder = VideoReassembler()
        jpeg = b"\xff\xd8JPEG\xff\xd9"
        self.assertIsNotNone(decoder.add_packet(packet(DATA, 10, 0, 1, jpeg)))
        self.assertIsNone(decoder.add_packet(packet(DATA, 9, 0, 1, jpeg)))


if __name__ == "__main__":
    unittest.main()
