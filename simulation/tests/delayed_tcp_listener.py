#!/usr/bin/env python3
"""Test-only delayed listener used to exercise process-aware readiness."""

import argparse
import socket
import time

parser = argparse.ArgumentParser()
parser.add_argument("--port", required=True, type=int)
parser.add_argument("--delay", required=True, type=float)
args = parser.parse_args()
time.sleep(args.delay)
with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as listener:
    listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    listener.bind(("127.0.0.1", args.port))
    listener.listen(1)
    print(f"AUDIT_LISTEN tcp:127.0.0.1:{args.port}", flush=True)
    time.sleep(30)
