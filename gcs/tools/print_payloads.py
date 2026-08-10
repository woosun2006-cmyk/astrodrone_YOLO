"""Prints DATA packet payloads as-is (no FEC logic) -- quick eyeball check
of what telem_sender is actually putting on the wire."""
import socket
import struct
import sys

HEADER_FMT = "<IBIBHBBBHI"
HEADER_LEN = struct.calcsize(HEADER_FMT)

port = int(sys.argv[1]) if len(sys.argv) > 1 else 15550
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.bind(("0.0.0.0", port))
s.settimeout(8)
count = 0
try:
    while count < 8:
        raw, _ = s.recvfrom(4096)
        _, ptype, seq, lane, lane_group, group_seq, group_size, parity_count, plen, crc = (
            struct.unpack_from(HEADER_FMT, raw, 0)
        )
        if ptype == 0:
            payload = raw[HEADER_LEN:HEADER_LEN + plen]
            print(f"seq={seq} lane={lane} {payload.decode('utf-8', 'replace')}")
            count += 1
except socket.timeout:
    print("(timeout waiting for more)")
