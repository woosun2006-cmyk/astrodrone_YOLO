#!/usr/bin/env python3
"""Receive-only direct telemetry probe for the observe simulation profile.

This process binds the router's UDP outputs and never sends a packet. It is
intentionally separate from the TCP MAVLink audit relay so observe exercises
the normal SITL -> MAVProxy/router path.
"""

import argparse
import collections
import datetime
import json
import math
import select
import signal
import socket
import time

from pymavlink.dialects.v20 import ardupilotmega as mavlink2


ARDUPILOT_SYSID = 1
AUTOPILOT1_COMPONENT = 1
ARDUPILOT_AUTOPILOT = 3
HEARTBEAT_MSG_ID = 0


def endpoint(value):
    if not value.startswith("udp:"):
        raise argparse.ArgumentTypeError("only udp: loopback endpoints are accepted")
    host, port_text = value[4:].rsplit(":", 1)
    if host == "localhost":
        host = "127.0.0.1"
    if host != "127.0.0.1":
        raise argparse.ArgumentTypeError("only 127.0.0.1/localhost is accepted")
    port = int(port_text)
    if not 1 <= port <= 65535:
        raise argparse.ArgumentTypeError("port must be in 1..65535")
    return host, port


def percentile(values, percent):
    if not values:
        return None
    ordered = sorted(values)
    index = max(0, min(len(ordered) - 1, math.ceil(percent * len(ordered)) - 1))
    return ordered[index]


def heartbeat_statistics(samples):
    intervals = [
        later.get("timestamp_monotonic_ns", later["timestamp_unix_ms"] * 1_000_000)
        - earlier.get("timestamp_monotonic_ns", earlier["timestamp_unix_ms"] * 1_000_000)
        for earlier, later in zip(samples, samples[1:])
    ]
    intervals_s = [value / 1_000_000_000.0 for value in intervals]
    return {
        "valid_ardupilot_count": len(samples),
        "first_timestamp": samples[0]["timestamp"] if samples else None,
        "last_timestamp": samples[-1]["timestamp"] if samples else None,
        "average_interval_s": (
            sum(intervals_s) / len(intervals_s) if intervals_s else None
        ),
        "maximum_interval_s": max(intervals_s) if intervals_s else None,
        "p95_interval_s": percentile(intervals_s, 0.95),
        "timeout_sec": 3.0,
        "timeout_exceeded": any(value > 3.0 for value in intervals_s),
        "samples": samples,
    }


class Stream:
    def __init__(self, label, endpoint_text, sock):
        self.label = label
        self.endpoint_text = endpoint_text
        self.sock = sock
        self.parser = mavlink2.MAVLink(None)
        self.parser.robust_parsing = True
        self.message_counts = collections.Counter()
        self.bad_data_count = 0
        self.heartbeat_samples = []
        self.datagram_count = 0
        self.byte_count = 0

    def feed(self, data, timestamp_unix_ns, timestamp_monotonic_ns):
        self.datagram_count += 1
        self.byte_count += len(data)
        messages = self.parser.parse_buffer(data) or []
        for message in messages:
            if message.get_type() == "BAD_DATA":
                self.bad_data_count += 1
                continue

            msgid = message.get_msgId()
            msgtype = message.get_type()
            self.message_counts[f"{msgid}:{msgtype}"] += 1
            source_system = message.get_srcSystem()
            source_component = message.get_srcComponent()
            if (
                msgid == HEARTBEAT_MSG_ID
                and source_system == ARDUPILOT_SYSID
                and source_component == AUTOPILOT1_COMPONENT
                and int(getattr(message, "autopilot", -1)) == ARDUPILOT_AUTOPILOT
            ):
                timestamp_unix_ms = timestamp_unix_ns // 1_000_000
                timestamp = datetime.datetime.fromtimestamp(
                    timestamp_unix_ms / 1000.0, datetime.timezone.utc
                ).isoformat(timespec="milliseconds").replace("+00:00", "Z")
                self.heartbeat_samples.append(
                    {
                        "timestamp": timestamp,
                        "timestamp_unix_ms": timestamp_unix_ms,
                        "timestamp_monotonic_ns": timestamp_monotonic_ns,
                        "message_id": msgid,
                        "message_type": msgtype,
                        "source_system": source_system,
                        "source_component": source_component,
                        "autopilot": int(message.autopilot),
                        "mode": int(message.custom_mode),
                        "armed": bool(message.base_mode & 128),
                        "system_status": int(message.system_status),
                    }
                )

    def report(self):
        return {
            "endpoint": self.endpoint_text,
            "route": "SITL_AUTOPILOT_TO_MAVPROXY_ROUTER",
            "observer_target": self.label,
            "message_counts": dict(sorted(self.message_counts.items())),
            "bad_data_count": self.bad_data_count,
            "datagram_count": self.datagram_count,
            "byte_count": self.byte_count,
            "heartbeat": heartbeat_statistics(self.heartbeat_samples),
        }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--control", required=True, type=endpoint)
    parser.add_argument("--telemetry", required=True, type=endpoint)
    parser.add_argument("--report", required=True)
    args = parser.parse_args()

    streams = []
    sockets = []
    for label, address in (("control", args.control), ("telemetry", args.telemetry)):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
        sock.bind(address)
        sock.setblocking(False)
        sockets.append(sock)
        streams.append(Stream(label, f"udp:{address[0]}:{address[1]}", sock))

    running = True

    def stop(_signum, _frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    print(
        "TELEMETRY_PROBE_READY control=udp:127.0.0.1:%d "
        "telemetry=udp:127.0.0.1:%d" % (args.control[1], args.telemetry[1]),
        flush=True,
    )
    try:
        while running:
            readable, _, _ = select.select(sockets, [], [], 0.25)
            for sock in readable:
                data, _ = sock.recvfrom(65535)
                timestamp_unix_ns = time.time_ns()
                timestamp_monotonic_ns = time.monotonic_ns()
                for stream in streams:
                    if stream.sock is sock:
                        stream.feed(data, timestamp_unix_ns, timestamp_monotonic_ns)
                        break
    finally:
        for sock in sockets:
            sock.close()
        report = {
            "generated_utc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
            "heartbeat_filter": {
                "message_id": HEARTBEAT_MSG_ID,
                "source_system": ARDUPILOT_SYSID,
                "source_component": AUTOPILOT1_COMPONENT,
                "autopilot": ARDUPILOT_AUTOPILOT,
            },
            "direction": "SITL_AUTOPILOT_TO_MAVPROXY_ROUTER",
            "streams": {stream.label: stream.report() for stream in streams},
        }
        with open(args.report, "w", encoding="utf-8") as output:
            json.dump(report, output, indent=2, sort_keys=True)
            output.write("\n")


if __name__ == "__main__":
    main()
