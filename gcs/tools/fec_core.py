"""Shared decode/recovery logic for gcs/sender's interleaved single-parity
XOR FEC, used by both fec_decoder.py (CLI verification tool) and
gcs_bridge.py (the laptop GCS bridge). Wire format lives in
gcs/sender/fec_packet.hpp -- keep this in sync with it.
"""

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
        self.data = {}  # group_seq -> payload bytes
        self.parity = None  # payload bytes of the (single) parity packet
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


class FecDecoder:
    """Binds a UDP socket and reassembles delivered/recovered payloads.

    Call poll() in a loop; each call blocks up to `poll_timeout` waiting for
    one packet, processes it, sweeps any groups that are now resolvable or
    have timed out, and returns the list of payloads (bytes) that became
    available this call, in no particular cross-lane order.
    """

    def __init__(self, listen_port, poll_timeout=0.2, group_timeout_s=GROUP_TIMEOUT_S):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", listen_port))
        self.sock.settimeout(poll_timeout)
        self.group_timeout_s = group_timeout_s

        self.lanes = defaultdict(dict)  # lane -> {lane_group: Group}
        # See fec_decoder.py's module docstring history / gcs/PLAN.md: a
        # packet for a lane_group at or below this watermark is a straggler
        # for a group already closed out (most commonly the parity packet,
        # which the encoder deliberately delays by one slot -- see
        # fec_encoder.hpp) and must be dropped, not turned into a phantom
        # all-missing group that later times out and double-counts an
        # already-delivered group as lost too.
        self.high_water = defaultdict(lambda: -1)

        self.stats = dict(
            data_pkts=0,
            parity_pkts=0,
            crc_fail=0,
            stale_dropped=0,
            delivered=0,
            recovered=0,
            lost=0,
        )

    def poll(self):
        delivered_payloads = []

        try:
            raw, _addr = self.sock.recvfrom(4096)
        except socket.timeout:
            raw = None

        if raw is not None and len(raw) >= HEADER_LEN:
            magic, ptype, seq, lane, lane_group, group_seq, group_size, parity_count, plen, crc = (
                struct.unpack_from(HEADER_FMT, raw, 0)
            )
            payload = raw[HEADER_LEN : HEADER_LEN + plen]
            if magic == MAGIC and zlib.crc32(payload) == crc:
                if lane_group not in self.lanes[lane] and lane_group <= self.high_water[lane]:
                    self.stats["stale_dropped"] += 1
                else:
                    g = self.lanes[lane].setdefault(lane_group, Group(group_size))
                    if ptype == TYPE_DATA:
                        g.add_data(group_seq, payload)
                        self.stats["data_pkts"] += 1
                    elif ptype == TYPE_PARITY:
                        g.add_parity(payload)
                        self.stats["parity_pkts"] += 1
            else:
                self.stats["crc_fail"] += 1

        now = time.monotonic()
        for lane, groups in list(self.lanes.items()):
            for lane_group, g in list(groups.items()):
                complete = g.try_recover()
                timed_out = (now - g.first_seen) > self.group_timeout_s
                if complete is not None:
                    got = len(g.data)
                    self.stats["delivered"] += got
                    self.stats["recovered"] += (g.group_size - got) if got < g.group_size else 0
                    delivered_payloads.extend(complete.values())
                    del groups[lane_group]
                    self.high_water[lane] = max(self.high_water[lane], lane_group)
                elif timed_out:
                    self.stats["delivered"] += len(g.data)
                    self.stats["lost"] += g.group_size - len(g.data)
                    delivered_payloads.extend(g.data.values())
                    del groups[lane_group]
                    self.high_water[lane] = max(self.high_water[lane], lane_group)

        return delivered_payloads
