#!/usr/bin/env python3
"""Loopback-only telemetry fan-out and command sink for shadow SITL.

MAVProxy sends telemetry to this process. Telemetry is copied to the C++
control, target-distance, and GCS observer ports. Datagrams sent back from
the control or target-distance ports are dropped and never reach MAVProxy.
"""

from __future__ import annotations

import argparse
import signal
import socket
import sys


LOOPBACK = "127.0.0.1"


def port(value: str) -> int:
    number = int(value)
    if not 1 <= number <= 65535:
        raise argparse.ArgumentTypeError("port must be in 1..65535")
    return number


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--listen", type=port, required=True)
    parser.add_argument("--control", type=port, required=True)
    parser.add_argument("--target", type=port, required=True)
    parser.add_argument("--gcs", type=port, required=True)
    args = parser.parse_args()

    running = True

    def stop(_signum: int, _frame: object) -> None:
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    dropped = 0
    forwarded = 0
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((LOOPBACK, args.listen))
    sock.settimeout(0.25)
    print(
        f"SHADOW_ROUTER_LISTEN udp:{LOOPBACK}:{args.listen} "
        f"control=udp:{LOOPBACK}:{args.control} "
        f"target=udp:{LOOPBACK}:{args.target} "
        f"gcs=udp:{LOOPBACK}:{args.gcs}",
        flush=True,
    )

    try:
        while running:
            try:
                payload, peer = sock.recvfrom(65535)
            except socket.timeout:
                continue
            if peer[0] != LOOPBACK:
                print(f"SHADOW_REJECT non_loopback_peer={peer[0]}", flush=True)
                continue

            # The C++ transports bind to these ports and reply to this
            # router's source port. Dropping both directions prevents command
            # and telemetry-request packets from reaching MAVProxy/SITL.
            if peer[1] in {args.control, args.target}:
                dropped += 1
                print(
                    f"SHADOW_DROP source_port={peer[1]} bytes={len(payload)} "
                    f"count={dropped}",
                    flush=True,
                )
                continue

            for destination in (args.control, args.target, args.gcs):
                sock.sendto(payload, (LOOPBACK, destination))
            forwarded += 1
            if forwarded == 1:
                print("SHADOW_TELEMETRY_FORWARDING active", flush=True)
    finally:
        sock.close()
        print(
            f"SHADOW_ROUTER_STOP forwarded={forwarded} dropped={dropped} "
            "vehicle_command_forwarded=0",
            flush=True,
        )
    return 0


if __name__ == "__main__":
    sys.exit(main())
