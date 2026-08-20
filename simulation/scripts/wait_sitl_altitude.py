#!/usr/bin/env python3
"""Read-only SITL altitude gate for simulation-only control startup."""

import argparse
import sys
import time

from pymavlink import mavutil


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--connect", default="udp:127.0.0.1:14552")
    parser.add_argument("--min-altitude", type=float, required=True)
    parser.add_argument("--timeout", type=float, default=30.0)
    args = parser.parse_args()

    master = mavutil.mavlink_connection(args.connect, source_system=250)
    heartbeat = master.wait_heartbeat(timeout=args.timeout)
    if heartbeat is None:
        print("altitude gate: HEARTBEAT timeout", file=sys.stderr)
        return 1

    deadline = time.monotonic() + args.timeout
    while time.monotonic() < deadline:
        message = master.recv_match(
            type=["GLOBAL_POSITION_INT", "HEARTBEAT"],
            blocking=True,
            timeout=1.0,
        )
        if message is None:
            continue
        if message.get_type() == "HEARTBEAT":
            # This is a read-only readiness gate. Ignore GCS/foreign sources.
            if message.get_srcSystem() != 1 or message.get_srcComponent() != 1:
                continue
            continue
        if message.get_srcSystem() != 1 or message.get_srcComponent() != 1:
            continue
        altitude_m = float(message.relative_alt) / 1000.0
        print(f"altitude gate: relative_alt={altitude_m:.2f}m", flush=True)
        if altitude_m >= args.min_altitude:
            print("altitude gate: ready", flush=True)
            return 0

    print(
        f"altitude gate: did not reach {args.min_altitude:.2f}m within {args.timeout:.1f}s",
        file=sys.stderr,
    )
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
