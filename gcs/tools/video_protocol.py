"""Decoder for the Astrodrone debug-video UDP chunks.

The video channel is best effort. It is never used as a control input. One
XOR parity chunk can recover one lost chunk per frame; incomplete frames are
discarded when a newer frame is accepted.
"""

import struct
import zlib

HEADER = struct.Struct("<IBBHqqHHHHI")
MAGIC = 0x31475641  # AGV1
VERSION = 1
DATA = 0
PARITY = 1


class VideoReassembler:
    def __init__(self):
        self._frames = {}
        self.latest_sequence = -1
        self.recovered = 0
        self.dropped = 0

    def add_packet(self, packet: bytes):
        if len(packet) < HEADER.size:
            self.dropped += 1
            return None
        (magic, version, kind, _reserved, sequence, timestamp_ns, index,
         count, payload_len, chunk_size, checksum) = HEADER.unpack_from(packet)
        payload = packet[HEADER.size:]
        if (magic != MAGIC or version != VERSION or count == 0 or
                chunk_size == 0 or len(payload) != chunk_size or
                zlib.crc32(payload) & 0xFFFFFFFF != checksum):
            self.dropped += 1
            return None
        if sequence < self.latest_sequence:
            self.dropped += 1
            return None
        frame = self._frames.setdefault(sequence, {
            "timestamp_ns": timestamp_ns,
            "count": count,
            "chunk_size": chunk_size,
            "data": {},
            "lengths": {},
            "parity": None,
        })
        if kind == DATA and index < count:
            frame["data"][index] = payload
            frame["lengths"][index] = payload_len
        elif kind == PARITY and index == 0xFFFF:
            frame["parity"] = payload
        else:
            self.dropped += 1
            return None
        result = self._complete(sequence, frame)
        if result is not None:
            self.latest_sequence = sequence
            self._frames = {k: v for k, v in self._frames.items() if k > sequence}
        return result

    def _complete(self, sequence, frame):
        count = frame["count"]
        data = frame["data"]
        if len(data) < count:
            if len(data) != count - 1 or frame["parity"] is None:
                return None
            missing = next(i for i in range(count) if i not in data)
            recovered = bytearray(frame["parity"])
            for chunk in data.values():
                for i, value in enumerate(chunk):
                    recovered[i] ^= value
            data[missing] = bytes(recovered)
            frame["lengths"][missing] = count and frame["chunk_size"]
            self.recovered += 1
        result = b"".join(data[i][:frame["lengths"].get(i, frame["chunk_size"])]
                           for i in range(count))
        end = result.rfind(b"\xff\xd9")
        if end >= 0:
            result = result[:end + 2]
        if not result.startswith(b"\xff\xd8"):
            self.dropped += 1
            return None
        return {
            "sequence": sequence,
            "timestamp_ns": frame["timestamp_ns"],
            "jpeg": result,
        }
