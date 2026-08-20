"""CLI reference decoder for gcs/sender's interleaved single-parity XOR FEC.
Prints running stats -- a link-quality/debugging tool, not the GCS itself
(see gcs_bridge.py for that). Recovery logic lives in fec_core.py, shared
with the bridge.

Usage:
    python fec_decoder.py --listen-port 15550 --stats-every 5
"""

import argparse
import time

from fec_core import FecDecoder


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--listen-port", type=int, default=15550)
    ap.add_argument("--stats-every", type=float, default=5.0, help="seconds between stats prints")
    args = ap.parse_args()

    decoder = FecDecoder(args.listen_port)
    last_report = time.monotonic()
    print(f"fec_decoder: listening on udp/{args.listen_port}")

    while True:
        decoder.poll()

        now = time.monotonic()
        if now - last_report >= args.stats_every:
            s = decoder.stats
            print(
                f"[{time.strftime('%H:%M:%S')}] data={s['data_pkts']} parity={s['parity_pkts']} "
                f"crc_fail={s['crc_fail']} stale_dropped={s['stale_dropped']} "
                f"delivered={s['delivered']} recovered={s['recovered']} lost={s['lost']}"
            )
            last_report = now


if __name__ == "__main__":
    main()
