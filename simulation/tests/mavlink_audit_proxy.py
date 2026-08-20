#!/usr/bin/env python3
"""Transparent loopback TCP relay for simulation-only MAVLink auditing.

The relay records both SITL-autopilot -> MAVProxy and MAVProxy -> SITL
directions. It is not used by production control or transport code.
"""

import argparse
import collections
import datetime
import json
import math
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
    mavutil.mavlink.MAVLINK_MSG_ID_MISSION_SET_CURRENT: "MISSION_SET_CURRENT",
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
    mavutil.mavlink.MAV_CMD_MISSION_START: "MISSION_START",
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


def iso_timestamp(unix_ms):
    return datetime.datetime.fromtimestamp(
        unix_ms / 1000.0, datetime.timezone.utc
    ).isoformat(timespec="milliseconds").replace("+00:00", "Z")


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


def command_type_for(message):
    msgid = message.get_msgId()
    if msgid == mavutil.mavlink.MAVLINK_MSG_ID_SET_MODE:
        return "SET_MODE"
    if msgid in (
        mavutil.mavlink.MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED,
        mavutil.mavlink.MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT,
    ):
        if msgid == mavutil.mavlink.MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED:
            try:
                if all(float(getattr(message, field)) == 0.0 for field in
                       ("vx", "vy", "vz", "yaw_rate")):
                    return "ZERO_VELOCITY_SETPOINT"
            except (AttributeError, TypeError, ValueError):
                pass
        return "VELOCITY_SETPOINT"
    if msgid in DIRECT_FORBIDDEN:
        return DIRECT_FORBIDDEN[msgid]
    if msgid in (mavutil.mavlink.MAVLINK_MSG_ID_COMMAND_LONG,
                 mavutil.mavlink.MAVLINK_MSG_ID_COMMAND_INT):
        command = int(message.command)
        # REQUEST_MESSAGE, SET_MESSAGE_INTERVAL and other telemetry setup
        # commands remain in the regular message/command counters, but are
        # deliberately excluded from vehicle-affecting event accounting.
        return FORBIDDEN_COMMANDS.get(command)
    return None


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
        self.events = []
        self.bad_data_count = 0
        self.heartbeat_samples = []
        self.first_s = None
        self.last_s = None

    def feed(self, data, elapsed_s, timestamp_unix_ms):
        for value in data:
            message = self.parser.parse_char(bytes((value,)))
            if message is None:
                continue
            if message.get_type() == "BAD_DATA":
                self.bad_data_count += 1
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

            command_type = command_type_for(message)
            if command_type is not None:
                source_system = None
                source_component = None
                try:
                    source_system = message.get_srcSystem()
                    source_component = message.get_srcComponent()
                except AttributeError:
                    pass
                event = {
                    "event": "vehicle_command",
                    "source": "mavlink_audit",
                    "direction": self.name,
                    "timestamp": iso_timestamp(timestamp_unix_ms),
                    "timestamp_unix_ms": timestamp_unix_ms,
                    "command_type": command_type,
                    "allowed": True,
                    "sent": True,
                    "blocked": False,
                    "block_reason": None,
                    "control_lock_reason": None,
                    "vehicle_affecting": True,
                    "message_id": msgid,
                    "message_type": msgtype,
                    "source_system": source_system,
                    "source_component": source_component,
                }
                if msgid == mavutil.mavlink.MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED:
                    for field in (
                        "time_boot_ms", "target_system", "target_component",
                        "coordinate_frame", "type_mask", "vx", "vy", "vz",
                        "yaw", "yaw_rate",
                    ):
                        if hasattr(message, field):
                            event[field] = getattr(message, field)
                self.events.append(event)

            if self.name == "sitl_to_mavproxy" and msgid == mavutil.mavlink.MAVLINK_MSG_ID_HEARTBEAT:
                source_system = message.get_srcSystem()
                source_component = message.get_srcComponent()
                autopilot = int(getattr(message, "autopilot", -1))
                if (
                    source_system == 1
                    and source_component == 1
                    and autopilot == mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA
                ):
                    heartbeat = {
                        "timestamp": iso_timestamp(timestamp_unix_ms),
                        "timestamp_unix_ms": timestamp_unix_ms,
                        "timestamp_monotonic_ns": time.monotonic_ns(),
                        "message_id": msgid,
                        "message_type": msgtype,
                        "source_system": source_system,
                        "source_component": source_component,
                        "autopilot": autopilot,
                        "mode": mavutil.mode_string_v10(message),
                        "armed": bool(message.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED),
                        "system_status": int(message.system_status),
                    }
                    self.heartbeat_samples.append(heartbeat)
                    self.events.append({
                        "event": "actual_heartbeat",
                        "source": "mavlink_audit",
                        "direction": "SITL_AUTOPILOT_TO_MAVPROXY",
                        "timestamp": heartbeat["timestamp"],
                        "timestamp_unix_ms": timestamp_unix_ms,
                        "command_type": None,
                        "allowed": None,
                        "sent": False,
                        "blocked": False,
                        "block_reason": None,
                        "control_lock_reason": None,
                        "vehicle_affecting": False,
                        "message_id": msgid,
                        "message_type": msgtype,
                        "source_system": source_system,
                        "source_component": source_component,
                        "autopilot": autopilot,
                        "mode": heartbeat["mode"],
                        "armed": heartbeat["armed"],
                        "system_status": heartbeat["system_status"],
                    })
            elif self.name == "sitl_to_mavproxy" and msgid == mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED:
                self.events.append({
                    "event": "actual_local_position",
                    "source": "mavlink_audit",
                    "direction": self.name,
                    "timestamp": iso_timestamp(timestamp_unix_ms),
                    "timestamp_unix_ms": timestamp_unix_ms,
                    "command_type": None,
                    "allowed": None,
                    "sent": False,
                    "blocked": False,
                    "block_reason": None,
                    "control_lock_reason": None,
                    "vehicle_affecting": False,
                    "message_id": msgid,
                    "message_type": msgtype,
                    "source_system": message.get_srcSystem(),
                    "source_component": message.get_srcComponent(),
                    "time_boot_ms": int(message.time_boot_ms),
                    "local_x_m": float(message.x),
                    "local_y_m": float(message.y),
                    "local_z_m": float(message.z),
                    "local_vx_mps": float(message.vx),
                    "local_vy_mps": float(message.vy),
                    "local_vz_mps": float(message.vz),
                })
            elif self.name == "sitl_to_mavproxy" and msgid == mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
                self.events.append({
                    "event": "actual_global_position",
                    "source": "mavlink_audit",
                    "direction": self.name,
                    "timestamp": iso_timestamp(timestamp_unix_ms),
                    "timestamp_unix_ms": timestamp_unix_ms,
                    "command_type": None,
                    "allowed": None,
                    "sent": False,
                    "blocked": False,
                    "block_reason": None,
                    "control_lock_reason": None,
                    "vehicle_affecting": False,
                    "message_id": msgid,
                    "message_type": msgtype,
                    "source_system": message.get_srcSystem(),
                    "source_component": message.get_srcComponent(),
                    "time_boot_ms": int(message.time_boot_ms),
                    "global_lat_deg": float(message.lat) / 1e7,
                    "global_lon_deg": float(message.lon) / 1e7,
                    "global_alt_m": float(message.alt) / 1000.0,
                    "relative_alt_m": float(message.relative_alt) / 1000.0,
                    "global_vx_mps": float(message.vx) / 100.0,
                    "global_vy_mps": float(message.vy) / 100.0,
                    "global_vz_mps": float(message.vz) / 100.0,
                    "global_heading_deg": float(message.hdg) / 100.0,
                })
            elif self.name == "sitl_to_mavproxy" and msgid == mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE:
                self.events.append({
                    "event": "actual_attitude",
                    "source": "mavlink_audit",
                    "direction": self.name,
                    "timestamp": iso_timestamp(timestamp_unix_ms),
                    "timestamp_unix_ms": timestamp_unix_ms,
                    "command_type": None,
                    "allowed": None,
                    "sent": False,
                    "blocked": False,
                    "block_reason": None,
                    "control_lock_reason": None,
                    "vehicle_affecting": False,
                    "message_id": msgid,
                    "message_type": msgtype,
                    "source_system": message.get_srcSystem(),
                    "source_component": message.get_srcComponent(),
                    "time_boot_ms": int(message.time_boot_ms),
                    "roll_rad": float(message.roll),
                    "pitch_rad": float(message.pitch),
                    "yaw_rad": float(message.yaw),
                    "rollspeed_rad_s": float(message.rollspeed),
                    "pitchspeed_rad_s": float(message.pitchspeed),
                    "yawspeed_rad_s": float(message.yawspeed),
                })

    def as_dict(self):
        return {
            "direction": {
                "sitl_to_mavproxy": "SITL_AUTOPILOT_TO_MAVPROXY",
                "mavproxy_to_sitl": "MAVPROXY_TO_SITL_AUTOPILOT",
            }[self.name],
            "first_s": self.first_s,
            "last_s": self.last_s,
            "message_counts": dict(sorted(self.counts.items())),
            "command_counts": dict(sorted(self.commands.items())),
            "forbidden_counts": dict(sorted(self.forbidden.items())),
            "forbidden_total": sum(self.forbidden.values()),
            "bad_data_count": self.bad_data_count,
            "heartbeat": heartbeat_statistics(self.heartbeat_samples),
        }


def main():
    process_started = time.monotonic()
    parser = argparse.ArgumentParser()
    parser.add_argument("--upstream", required=True, type=endpoint)
    parser.add_argument("--listen", required=True, type=endpoint)
    parser.add_argument("--report", required=True)
    parser.add_argument("--control-events", default="")
    args = parser.parse_args()

    running = True

    def stop(_signum, _frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    started = time.monotonic()
    started_unix_ms = int(time.time() * 1000)
    upstream_audit = DirectionAudit("sitl_to_mavproxy")
    downstream_audit = DirectionAudit("mavproxy_to_sitl")
    listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    listener.bind(args.listen)
    listener.listen(1)
    listener.settimeout(0.25)
    print(f"AUDIT_LISTEN tcp:{args.listen[0]}:{args.listen[1]}", flush=True)
    print(f"AUDIT_PROXY_TIMING listen_ms={(time.monotonic() - process_started) * 1000:.3f}", flush=True)

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
                try:
                    data = source.recv(65535)
                except (ConnectionResetError, BrokenPipeError) as error:
                    print(f"AUDIT_PEER_CLOSED direction="
                          f"{'mavproxy' if source is client else 'sitl'} "
                          f"reason={type(error).__name__}", flush=True)
                    running = False
                    break
                if not data:
                    running = False
                    break
                now = time.monotonic() - started
                timestamp_unix_ms = started_unix_ms + int(now * 1000)
                if source is client:
                    downstream_audit.feed(data, now, timestamp_unix_ms)
                    try:
                        upstream.sendall(data)
                    except (ConnectionResetError, BrokenPipeError):
                        running = False
                        break
                else:
                    upstream_audit.feed(data, now, timestamp_unix_ms)
                    try:
                        client.sendall(data)
                    except (ConnectionResetError, BrokenPipeError):
                        running = False
                        break
    finally:
        for stream in (client, upstream, listener):
            if stream is not None:
                stream.close()
        events = list(downstream_audit.events) + list(upstream_audit.events)
        if args.control_events:
            try:
                with open(args.control_events, encoding="utf-8") as control_events:
                    for line in control_events:
                        line = line.strip()
                        if not line:
                            continue
                        try:
                            events.append(json.loads(line))
                        except json.JSONDecodeError:
                            print(f"AUDIT_WARNING invalid control event ignored: {line}",
                                  file=sys.stderr, flush=True)
            except FileNotFoundError:
                pass

        events.sort(key=lambda event: (event.get("timestamp_unix_ms", 0),
                                       event.get("source", "")))
        lock_events = [event for event in events if event.get("event") == "control_lock"]
        lock_event = min(lock_events,
                         key=lambda event: event.get("timestamp_unix_ms", 0),
                         default=None)
        lock_timestamp_unix_ms = (
            lock_event.get("timestamp_unix_ms") if lock_event is not None else None
        )
        vehicle_events = [event for event in events
                          if event.get("source") == "mavlink_audit" and
                          event.get("vehicle_affecting")]
        blocked_after_lock = [event for event in events
                              if event.get("source") == "command_gate" and
                              event.get("blocked") and
                              lock_timestamp_unix_ms is not None and
                              event.get("timestamp_unix_ms", 0) >= lock_timestamp_unix_ms]
        if lock_timestamp_unix_ms is not None:
            for event in events:
                event["control_lock_timestamp_unix_ms"] = lock_timestamp_unix_ms
                event["control_lock_timestamp"] = iso_timestamp(lock_timestamp_unix_ms)
        else:
            for event in events:
                event["control_lock_timestamp_unix_ms"] = None
                event["control_lock_timestamp"] = None

        lock_before_count = sum(1 for event in vehicle_events
                                if lock_timestamp_unix_ms is not None and
                                event.get("timestamp_unix_ms", 0) < lock_timestamp_unix_ms)
        lock_after_count = sum(1 for event in vehicle_events
                               if lock_timestamp_unix_ms is not None and
                               event.get("timestamp_unix_ms", 0) >= lock_timestamp_unix_ms)
        report = {
            "generated_utc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
            "duration_s": time.monotonic() - started,
            "listen": f"tcp:{args.listen[0]}:{args.listen[1]}",
            "upstream": f"tcp:{args.upstream[0]}:{args.upstream[1]}",
            "sitl_to_mavproxy": upstream_audit.as_dict(),
            "mavproxy_to_sitl": downstream_audit.as_dict(),
            "vehicle_affecting_command_count": sum(downstream_audit.forbidden.values()),
            "events": events,
            "event_count": len(events),
            "control_lock_timestamp": (
                iso_timestamp(lock_timestamp_unix_ms)
                if lock_timestamp_unix_ms is not None else None
            ),
            "control_lock_timestamp_unix_ms": lock_timestamp_unix_ms,
            "lock_before_vehicle_command_count": lock_before_count,
            "lock_after_vehicle_command_count": lock_after_count,
            "lock_after_blocked_command_count": len(blocked_after_lock),
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
