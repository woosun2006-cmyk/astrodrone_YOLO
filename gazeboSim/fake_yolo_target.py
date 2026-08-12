#!/usr/bin/env python3
"""Gazebo-only stand-in for YOLO_MODEL/cpp/yolo_headless.cpp's /target endpoint.

yolo_headless.cpp opens the camera through a hardcoded nvarguscamerasrc
GStreamer pipeline (Jetson CSI camera API via L4T Argus) - it cannot read a
Gazebo-rendered frame without a code change to that pipeline. This script
serves the same JSON schema control/target_distance.cpp expects from
GET /target (see control/target_json.hpp and target_distance.cpp's polling
code: found, confirmed, age_ms, x_px, y_px), so the rest of the real
pipeline - mavlink_proxy, target_distance.cpp, control.cpp - can be
exercised against the Gazebo/SITL vehicle. It performs no perception; the
detection it reports is static and synthetic. See for_gazebo/README.md.
"""
import argparse
import http.server
import json
import threading


class State:
    lock = threading.Lock()
    confirmed = True
    x_px = 0.0
    y_px = 0.0


class Handler(http.server.BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        pass

    def do_GET(self):
        if self.path != "/target":
            self.send_response(404)
            self.end_headers()
            return
        with State.lock:
            body = json.dumps({
                "found": State.confirmed,
                "confirmed": State.confirmed,
                "age_ms": 0,
                "x_px": State.x_px,
                "y_px": State.y_px,
            }).encode()
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--port", type=int, default=8002)
    args = p.parse_args()
    # HTTPServer (not ThreadingHTTPServer, which needs Python 3.7+): the
    # only client is target_distance.cpp's single-threaded poll loop, so no
    # concurrency is needed.
    srv = http.server.HTTPServer(("127.0.0.1", args.port), Handler)
    print(
        f"fake_yolo_target: serving /target on 127.0.0.1:{args.port} "
        f"(confirmed={State.confirmed} x_px={State.x_px} y_px={State.y_px})",
        flush=True,
    )
    srv.serve_forever()


if __name__ == "__main__":
    main()
