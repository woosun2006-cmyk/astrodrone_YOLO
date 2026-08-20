#!/usr/bin/env python3
"""Summarize commanded and actual path angles from a loopback SITL audit."""

import argparse
import csv
import json
import math
import statistics
from pathlib import Path


def percentile(values, fraction):
    if not values:
        return None
    ordered = sorted(values)
    index = max(0, min(len(ordered) - 1, math.ceil(fraction * len(ordered)) - 1))
    return ordered[index]


def summary(values):
    return {
        "count": len(values),
        "median_rad": statistics.median(values) if values else None,
        "p95_rad": percentile(values, 0.95),
        "min_rad": min(values) if values else None,
        "max_rad": max(values) if values else None,
    }


def finite_float(value):
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def load_audit(path):
    with path.open(encoding="utf-8") as stream:
        return json.load(stream)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--audit", required=True, type=Path)
    parser.add_argument("--flight-csv", type=Path)
    parser.add_argument("--stop-distance", type=float, default=2.0)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()

    audit = load_audit(args.audit)
    events = audit.get("events", [])
    command_samples = []
    actual_samples = []
    for event in events:
        if event.get("event") == "vehicle_command" and \
                event.get("direction") == "mavproxy_to_sitl" and \
                event.get("message_type") == "SET_POSITION_TARGET_LOCAL_NED":
            vx = finite_float(event.get("vx"))
            vy = finite_float(event.get("vy"))
            vz = finite_float(event.get("vz"))
            if None not in (vx, vy, vz):
                horizontal = math.hypot(vx, vy)
                angle = math.atan2(abs(vz), horizontal) if horizontal > 1e-9 else None
                command_samples.append({
                    "timestamp": event.get("timestamp"),
                    "timestamp_unix_ms": event.get("timestamp_unix_ms"),
                    "vx": vx,
                    "vy": vy,
                    "vz": vz,
                    "horizontal_speed_mps": horizontal,
                    "path_angle_rad": angle,
                    "coordinate_frame": event.get("coordinate_frame"),
                    "type_mask": event.get("type_mask"),
                })
        elif event.get("event") == "actual_local_position":
            vx = finite_float(event.get("local_vx_mps"))
            vy = finite_float(event.get("local_vy_mps"))
            vz = finite_float(event.get("local_vz_mps"))
            if None not in (vx, vy, vz):
                horizontal = math.hypot(vx, vy)
                angle = math.atan2(abs(vz), horizontal) if horizontal > 1e-9 else None
                actual_samples.append({
                    "timestamp": event.get("timestamp"),
                    "timestamp_unix_ms": event.get("timestamp_unix_ms"),
                    "x_m": event.get("local_x_m"),
                    "y_m": event.get("local_y_m"),
                    "z_m": event.get("local_z_m"),
                    "vx_mps": vx,
                    "vy_mps": vy,
                    "vz_mps": vz,
                    "horizontal_speed_mps": horizontal,
                    "path_angle_rad": angle,
                })

    command_angles = [sample["path_angle_rad"] for sample in command_samples
                      if sample["path_angle_rad"] is not None]
    actual_angles = [sample["path_angle_rad"] for sample in actual_samples
                     if sample["path_angle_rad"] is not None]
    report = {
        "source": "simulation/tests/mavlink_audit_proxy.py",
        "audit": str(args.audit),
        "command_path_angle": summary(command_angles),
        "actual_local_position_path_angle": summary(actual_angles),
        "command_samples": command_samples,
        "actual_local_position_samples": actual_samples,
        "three_axis_command_count": sum(
            abs(sample["vx"]) > 1e-6 and abs(sample["vy"]) > 1e-6 and
            abs(sample["vz"]) > 1e-6 for sample in command_samples
        ),
    }

    if args.flight_csv and args.flight_csv.exists():
        with args.flight_csv.open(newline="", encoding="utf-8") as stream:
            rows = list(csv.DictReader(stream))
        approach = [row for row in rows if finite_float(row.get("dist_m")) is not None
                    and float(row["dist_m"]) > args.stop_distance]
        final_centering = [row for row in rows
                           if row.get("event") == "FINAL_CENTERING"]
        final_centering_vz_nonzero = [row for row in final_centering
                                      if abs(float(row.get("vz", 0))) > 1e-6]
        final_centering_horizontal = [row for row in final_centering
                                      if math.hypot(float(row.get("vx", 0)),
                                                    float(row.get("vy", 0))) > 1e-6]
        simultaneous = [row for row in approach
                        if abs(float(row.get("vx", 0))) > 1e-6 and
                        abs(float(row.get("vz", 0))) > 1e-6]
        hold_rows = [row for row in rows if row.get("safety_override") == "target_centered_hold"]
        first_hold = next((index for index, row in enumerate(rows)
                           if row.get("safety_override") == "target_centered_hold"), None)
        auto_after_hold = 0 if first_hold is None else sum(
            row.get("mode") == "AUTO" for row in rows[first_hold:]
        )
        report["flight_csv"] = str(args.flight_csv)
        report["approach"] = {
            "stop_distance_m": args.stop_distance,
            "rows_distance_gt_stop_distance": len(approach),
            "rows_vx_and_vz_nonzero": len(simultaneous),
            "final_centering_rows": len(final_centering),
            "final_centering_vz_nonzero": len(final_centering_vz_nonzero),
            "final_centering_horizontal_command_rows": len(final_centering_horizontal),
            "hold_rows": len(hold_rows),
            "hold_entered": first_hold is not None,
            "auto_after_hold_count": auto_after_hold,
        }

    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", encoding="utf-8") as stream:
        json.dump(report, stream, indent=2, sort_keys=True)
        stream.write("\n")
    print(
        "PATH_ANGLE_RESULT command_median_rad=%s command_p95_rad=%s "
        "actual_median_rad=%s actual_p95_rad=%s three_axis=%s report=%s" % (
            report["command_path_angle"]["median_rad"],
            report["command_path_angle"]["p95_rad"],
            report["actual_local_position_path_angle"]["median_rad"],
            report["actual_local_position_path_angle"]["p95_rad"],
            report["three_axis_command_count"],
            args.output,
        ),
        flush=True,
    )


if __name__ == "__main__":
    main()
