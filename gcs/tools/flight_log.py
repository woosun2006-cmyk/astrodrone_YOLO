"""Flight-log cmd tool: run this in its own terminal window when a flight
starts. Prints every telemetry message live and appends it to timestamped
.jsonl/.csv files under --log-dir.

Reads gcs_bridge.py's local relay (127.0.0.1:--relay-port), not the Jetson
directly -- the bridge is the sole owner of the Jetson-facing UDP port and
has already done FEC recovery, so this is plain UDP with no FEC logic of
its own (loopback doesn't drop packets). Run gcs_bridge.py first.

Usage:
    python flight_log.py --relay-port 15551 --log-dir ./logs
"""

import argparse
import csv
import json
import socket
import time
from pathlib import Path

# Fixed column order so the CSV header doesn't shuffle if a field is
# missing from a given message (e.g. alt_m/target_detected can be null).
CSV_FIELDS = ["recv_ts", "seq", "ts_ms", "mode", "armed", "link_ok", "alt_m", "target_detected",
              "target_x_px", "target_y_px", "target_distance_m"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--relay-port", type=int, default=15551, help="gcs_bridge.py's --relay-port")
    ap.add_argument("--log-dir", default="./logs")
    args = ap.parse_args()

    log_dir = Path(args.log_dir)
    log_dir.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    jsonl_path = log_dir / f"flight_{stamp}.jsonl"
    csv_path = log_dir / f"flight_{stamp}.csv"

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", args.relay_port))
    sock.settimeout(0.5)

    print(f"flight_log: listening on 127.0.0.1:{args.relay_port} (gcs_bridge.py's relay)")
    print(f"flight_log: writing {jsonl_path.name} and {csv_path.name} in {log_dir}/")
    print("flight_log: Ctrl+C to stop\n")

    count = 0
    with jsonl_path.open("w", encoding="utf-8") as jf, csv_path.open("w", newline="", encoding="utf-8") as cf:
        writer = csv.DictWriter(cf, fieldnames=CSV_FIELDS, extrasaction="ignore")
        writer.writeheader()
        cf.flush()

        try:
            while True:
                try:
                    raw, _addr = sock.recvfrom(4096)
                except socket.timeout:
                    continue

                try:
                    msg = json.loads(raw.decode("utf-8"))
                except (UnicodeDecodeError, json.JSONDecodeError):
                    continue

                print(json.dumps(msg, ensure_ascii=False))

                jf.write(json.dumps(msg, ensure_ascii=False) + "\n")
                jf.flush()

                row = dict(msg)
                row["recv_ts"] = time.strftime("%H:%M:%S")
                writer.writerow(row)
                cf.flush()
                count += 1
        except KeyboardInterrupt:
            pass

    print(f"\nflight_log: stopped. {count} messages logged to {jsonl_path} / {csv_path}")


if __name__ == "__main__":
    main()
