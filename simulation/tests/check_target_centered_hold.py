#!/usr/bin/env python3
"""Regression check for a completed 120-second loopback SITL hold run."""

import csv
import sys
from pathlib import Path


def fail(message: str) -> int:
    print(f"FAIL: {message}", file=sys.stderr)
    return 1


def main() -> int:
    if len(sys.argv) != 2:
        print(f"usage: {sys.argv[0]} RUN_LOG_DIR", file=sys.stderr)
        return 2

    root = Path(sys.argv[1])
    autonomy_log = root / "autonomy.log"
    if not autonomy_log.is_file():
        return fail(f"missing {autonomy_log}")

    lines = autonomy_log.read_text(errors="replace").splitlines()
    hold_indices = [
        i for i, line in enumerate(lines)
        if "TargetCenteredHold 진입" in line
    ]
    if not hold_indices:
        return fail("TargetCenteredHold was not entered")

    first_hold = hold_indices[0]
    after_hold = lines[first_hold + 1 :]
    if any("AUTO 복귀" in line for line in after_hold):
        return fail("AUTO return occurred after TargetCenteredHold")

    csv_files = sorted(root.glob("flight_*.csv"))
    hold_rows = []
    for csv_path in csv_files:
        with csv_path.open(newline="") as stream:
            for row in csv.DictReader(stream):
                if row.get("safety_override") == "target_centered_hold":
                    hold_rows.append(row)
    if not hold_rows:
        return fail("no TargetCenteredHold CSV rows")
    if any(
        abs(float(row.get(axis, "0"))) > 1e-9
        for row in hold_rows
        for axis in ("vx", "vy", "vz")
    ):
        return fail("non-zero velocity found during TargetCenteredHold")

    print(
        f"PASS TargetCenteredHold rows={len(hold_rows)}; "
        "no AUTO return after hold; zero velocity maintained"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
