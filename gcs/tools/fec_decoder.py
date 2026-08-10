"""Reference decoder for gcs/sender's interleaved single-parity XOR FEC.

Standalone verification tool, not the final laptop GCS -- proves the wire
format (fec_packet.hpp) and recovery logic work end to end, independent of
whatever stack the real laptop app ends up using.

Usage:
    python fec_decoder.py --listen-port 15550 --stats-every 5

Wire format (see gcs/sender/fec_packet.hpp, must stay in sync):
    magic(u32) type(u8) seq(u32) lane(u8) lane_group(u16) group_seq(u8)
    group_size(u8) parity_count(u8) payload_len(u16) crc32(u32)   -- 21 bytes
    payload (payload_len bytes)
"""

import argparse
import socket
import struct
import time
import zlib
from collections import defaultdict

MAGIC = 0x31544741
TYPE_DATA = 0
TYPE_PARITY = 1
HEADER_FMT = "<IBIBHBBBHI"
HEADER_LEN = struct.calcsize(HEADER_FMT)
assert HEADER_LEN == 21, HEADER_LEN

# How long a group waits for stragglers before it's finalized as-is (lost
# packets that can't be recovered are simply reported lost). Bounds latency
# so the receiver never blocks the live feed waiting on a group.
GROUP_TIMEOUT_S = 1.0


class Group:
    def __init__(self, group_size):
        self.group_size = group_size
        self.data = {}          # group_seq -> payload bytes
        self.parity = None      # payload bytes of the (single) parity packet
        self.first_seen = time.monotonic()

    def add_data(self, group_seq, payload):
        self.data[group_seq] = payload

    def add_parity(self, payload):
        self.parity = payload

    def try_recover(self):
        """Returns dict(group_seq -> payload) once complete/recoverable, else None."""
        missing = [i for i in range(self.group_size) if i not in self.data]
        if not missing:
            return dict(self.data)
        if len(missing) == 1 and self.parity is not None:
            recovered = bytearray(self.parity)
            for payload in self.data.values():
                for i, b in enumerate(payload):
                    recovered[i] ^= b
            trimmed = bytes(recovered).rstrip(b"\x00")
            result = dict(self.data)
            result[missing[0]] = trimmed
            return result
        return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--listen-port", type=int, default=15550)
    ap.add_argument("--stats-every", type=float, default=5.0, help="seconds between stats prints")
    args = ap.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", args.listen_port))
    sock.settimeout(0.2)

    lanes = defaultdict(dict)  # lane -> {lane_group: Group}
    # Highest lane_group already finalized (delivered/recovered/lost) per
    # lane. A packet for a lane_group at or below this watermark is a
    # straggler for a group we've already closed out -- most commonly the
    # parity packet arriving after its group already completed on data
    # alone (the encoder deliberately delays parity by one slot, see
    # fec_encoder.hpp, so this is the common case, not an edge case).
    # Without this check, that straggler would spawn a phantom all-missing
    # Group that times out and double-counts an already-delivered group as
    # lost too (caught by testing at 0% simulated loss, where every group
    # completes on data alone and every parity arrives "late").
    high_water = defaultdict(lambda: -1)
    stats = dict(data_pkts=0, parity_pkts=0, crc_fail=0, delivered=0, recovered=0, lost=0, stale_dropped=0)
    last_report = time.monotonic()

    print(f"fec_decoder: listening on udp/{args.listen_port}")

    while True:
        try:
            raw, _addr = sock.recvfrom(4096)
        except socket.timeout:
            raw = None

        if raw is not None and len(raw) >= HEADER_LEN:
            magic, ptype, seq, lane, lane_group, group_seq, group_size, parity_count, plen, crc = (
                struct.unpack_from(HEADER_FMT, raw, 0)
            )
            payload = raw[HEADER_LEN : HEADER_LEN + plen]
            if magic == MAGIC and zlib.crc32(payload) == crc:
                if lane_group not in lanes[lane] and lane_group <= high_water[lane]:
                    stats["stale_dropped"] += 1
                else:
                    g = lanes[lane].setdefault(lane_group, Group(group_size))
                    if ptype == TYPE_DATA:
                        g.add_data(group_seq, payload)
                        stats["data_pkts"] += 1
                    elif ptype == TYPE_PARITY:
                        g.add_parity(payload)
                        stats["parity_pkts"] += 1
            else:
                stats["crc_fail"] += 1

        now = time.monotonic()
        for lane, groups in list(lanes.items()):
            for lane_group, g in list(groups.items()):
                complete = g.try_recover()
                timed_out = (now - g.first_seen) > GROUP_TIMEOUT_S
                if complete is not None:
                    got = len(g.data)
                    stats["delivered"] += got
                    stats["recovered"] += (g.group_size - got) if got < g.group_size else 0
                    del groups[lane_group]
                    high_water[lane] = max(high_water[lane], lane_group)
                elif timed_out:
                    stats["delivered"] += len(g.data)
                    stats["lost"] += g.group_size - len(g.data)
                    del groups[lane_group]
                    high_water[lane] = max(high_water[lane], lane_group)

        if now - last_report >= args.stats_every:
            print(
                f"[{time.strftime('%H:%M:%S')}] data={stats['data_pkts']} "
                f"parity={stats['parity_pkts']} crc_fail={stats['crc_fail']} "
                f"stale_dropped={stats['stale_dropped']} "
                f"delivered={stats['delivered']} recovered={stats['recovered']} "
                f"lost={stats['lost']}"
            )
            last_report = now


if __name__ == "__main__":
    main()
