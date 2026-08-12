#!/usr/bin/env python3
"""Transparent loopback TCP relay that audits MAVProxy -> SITL MAVLink traffic."""

import argparse
import collections
import datetime
import json
import os
import select
import signal
import socket
import sys
import time

from pymavlink import mavutil
from pymavlink.dialects.v20 import ardupilotmega as mavlink2


DIRECT_FORBIDDEN = {
    mavutil.mavlink.MAVLINK_MSG_ID_SET_MODE: "SET_MODE",
    mavutil.mavlink.MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE: "RC_CHANNELS_OVERRIDE",
    mavutil.mavlink.MAVLINK_MSG_ID_MANUAL_CONTROL: "MANUAL_CONTROL",
    mavutil.mavlink.MAVLINK_MSG_ID_PARAM_SET: "PARAM_SET",
    mavutil.mavlink.MAVLINK_MSG_ID_SET_ATTITUDE_TARGET: "SET_ATTITUDE_TARGET",
    mavutil.mavlink.MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED: "SET_POSITION_TARGET_LOCAL_NED",
    mavutil.mavlink.MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT: "SET_POSITION_TARGET_GLOBAL_INT",
    mavutil.mavlink.MAVLINK_MSG_ID_MISSION_ITEM: "MISSION_ITEM",
    mavutil.mavlink.MAVLINK_MSG_ID_MISSION_ITEM_INT: "MISSION_ITEM_INT",
    mavutil.mavlink.MAVLINK_MSG_ID_MISSION_COUNT: "MISSION_COUNT",
    mavutil.mavlink.MAVLINK_MSG_ID_MISSION_CLEAR_ALL: "MISSION_CLEAR_ALL",
    mavutil.mavlink.MAVLINK_MSG_ID_MISSION_WRITE_PARTIAL_LIST: "MISSION_WRITE_PARTIAL_LIST",
}
if hasattr(mavutil.mavlink, "MAVLINK_MSG_ID_PARAM_EXT_SET"):
    DIRECT_FORBIDDEN[mavutil.mavlink.MAVLINK_MSG_ID_PARAM_EXT_SET] = "PARAM_EXT_SET"

FORBIDDEN_COMMANDS = {
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM: "ARM_DISARM",
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF: "TAKEOFF",
    mavutil.mavlink.MAV_CMD_NAV_LAND: "LAND",
    mavutil.mavlink.MAV_CMD_DO_SET_MODE: "DO_SET_MODE",
    mavutil.mavlink.MAV_CMD_NAV_GUIDED_ENABLE: "NAV_GUIDED_ENABLE",
    mavutil.mavlink.MAV_CMD_DO_REPOSITION: "DO_REPOSITION",
}


def endpoint(value):
    if not value.startswith("tcp:"):
        raise argparse.ArgumentTypeError("only tcp: loopback endpoints are accepted")
    host, port_text = value[4:].rsplit(":", 1)
    if host == "localhost":
        host = "127.0.0.1"
    if host != "127.0.0.1":
        raise argparse.ArgumentTypeError("only 127.0.0.1/localhost is accepted")
    port = int(port_text)
    if not 1 <= port <= 65535:
        raise argparse.ArgumentTypeError("port must be in 1..65535")
    return host, port


class DirectionAudit:
    def __init__(self, name):
        self.name = name
        self.parser = mavlink2.MAVLink(None)
        # ArduPilot's TCP serial endpoint may contain a startup newline before
        # the first MAVLink frame. Keep forwarding it, but do not let that
        # non-frame byte terminate the transparent audit relay.
        self.parser.robust_parsing = True
        self.counts = collections.Counter()
        self.forbidden = collections.Counter()
        self.commands = collections.Counter()
        self.first_s = None
        self.last_s = None

    def feed(self, data, elapsed_s):
        for value in data:
            message = self.parser.parse_char(bytes((value,)))
            if message is None:
                continue
            msgid = message.get_msgId()
            msgtype = message.get_type()
            self.counts[f"{msgid}:{msgtype}"] += 1
            if self.first_s is None:
                self.first_s = elapsed_s
            self.last_s = elapsed_s
            if msgid in DIRECT_FORBIDDEN:
                self.forbidden[DIRECT_FORBIDDEN[msgid]] += 1
            if msgid in (mavutil.mavlink.MAVLINK_MSG_ID_COMMAND_LONG,
                         mavutil.mavlink.MAVLINK_MSG_ID_COMMAND_INT):
                command = int(message.command)
                enum_entry = mavutil.mavlink.enums.get("MAV_CMD", {}).get(command)
                command_name = enum_entry.name if enum_entry is not None else "UNKNOWN"
                self.commands[f"{command}:{command_name}"] += 1
                if command in FORBIDDEN_COMMANDS:
                    self.forbidden[f"COMMAND:{command}:{FORBIDDEN_COMMANDS[command]}"] += 1

    def as_dict(self):
        return {
            "first_s": self.first_s,
            "last_s": self.last_s,
            "message_counts": dict(sorted(self.counts.items())),
            "command_counts": dict(sorted(self.commands.items())),
            "forbidden_counts": dict(sorted(self.forbidden.items())),
            "forbidden_total": sum(self.forbidden.values()),
        }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--upstream", required=True, type=endpoint)
    parser.add_argument("--listen", required=True, type=endpoint)
    parser.add_argument("--report", required=True)
    args = parser.parse_args()

    running = True

    def stop(_signum, _frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    started = time.monotonic()
    upstream_audit = DirectionAudit("sitl_to_mavproxy")
    downstream_audit = DirectionAudit("mavproxy_to_sitl")
    listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    listener.bind(args.listen)
    listener.listen(1)
    listener.settimeout(0.25)
    print(f"AUDIT_LISTEN tcp:{args.listen[0]}:{args.listen[1]}", flush=True)

    client = None
    upstream = None
    try:
        while running and client is None:
            try:
                client, peer = listener.accept()
                print(f"MAVPROXY_CONNECTED peer={peer[0]}:{peer[1]}", flush=True)
            except socket.timeout:
                pass
        if client is not None and running:
            upstream = socket.create_connection(args.upstream, timeout=5)
            upstream.settimeout(None)
            print(f"SITL_CONNECTED tcp:{args.upstream[0]}:{args.upstream[1]}", flush=True)
        while running and client is not None and upstream is not None:
            readable, _, _ = select.select([client, upstream], [], [], 0.25)
            for source in readable:
                data = source.recv(65535)
                if not data:
                    running = False
                    break
                now = time.monotonic() - started
                if source is client:
                    downstream_audit.feed(data, now)
                    upstream.sendall(data)
                else:
                    upstream_audit.feed(data, now)
                    client.sendall(data)
    finally:
        for stream in (client, upstream, listener):
            if stream is not None:
                stream.close()
        report = {
            "generated_utc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
            "duration_s": time.monotonic() - started,
            "listen": f"tcp:{args.listen[0]}:{args.listen[1]}",
            "upstream": f"tcp:{args.upstream[0]}:{args.upstream[1]}",
            "sitl_to_mavproxy": upstream_audit.as_dict(),
            "mavproxy_to_sitl": downstream_audit.as_dict(),
            "vehicle_affecting_command_count": sum(downstream_audit.forbidden.values()),
        }
        report_dir = os.path.dirname(os.path.abspath(args.report))
        os.makedirs(report_dir, exist_ok=True)
        with open(args.report, "w", encoding="utf-8") as output:
            json.dump(report, output, indent=2, sort_keys=True)
            output.write("\n")
        print("AUDIT_RESULT vehicle_affecting_command_count="
              f"{report['vehicle_affecting_command_count']} report={args.report}", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
