"""UDP relay that randomly drops packets, for testing FEC recovery without
needing real flaky wifi. Forwards recv_port -> dest_port, dropping each
packet independently with probability --loss, plus optional burst drops.

Usage:
    python lossy_relay.py --listen-port 25550 --dest-port 15551 --loss 0.15 --burst 3
"""

import argparse
import random
import socket


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--listen-port", type=int, required=True)
    ap.add_argument("--dest-host", default="127.0.0.1")
    ap.add_argument("--dest-port", type=int, required=True)
    ap.add_argument("--loss", type=float, default=0.1, help="per-packet drop probability")
    ap.add_argument("--burst", type=int, default=1, help="on a drop, also drop this many following packets")
    args = ap.parse_args()

    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.bind(("0.0.0.0", args.listen_port))
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    burst_remaining = 0
    forwarded = 0
    dropped = 0

    print(f"lossy_relay: udp/{args.listen_port} -> {args.dest_host}:{args.dest_port} "
          f"loss={args.loss} burst={args.burst}")

    while True:
        data, _addr = rx.recvfrom(4096)
        drop = False
        if burst_remaining > 0:
            burst_remaining -= 1
            drop = True
        elif random.random() < args.loss:
            drop = True
            burst_remaining = args.burst - 1

        if drop:
            dropped += 1
        else:
            tx.sendto(data, (args.dest_host, args.dest_port))
            forwarded += 1

        if (forwarded + dropped) % 50 == 0:
            rate = dropped / max(1, forwarded + dropped)
            print(f"forwarded={forwarded} dropped={dropped} actual_loss={rate:.2%}")


if __name__ == "__main__":
    main()
