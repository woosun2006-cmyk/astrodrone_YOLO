"""Laptop-side GCS bridge: receives the Jetson's FEC telemetry and chunked
debug video over separate UDP sockets, then serves both to the browser.

Telemetry and video are independent: a stale or missing video frame never
blocks telemetry delivery. The dashboard draws the latest detection metadata
over the received JPEG instead of requiring the Jetson's HTTP server.

Only one process can bind the Jetson-facing UDP port, so this is the single
owner of it. flight_log.py (the separate cmd-window raw log viewer) doesn't
talk to the Jetson directly -- it reads a local relay of the same decoded
messages this process re-sends to 127.0.0.1:--relay-port. That hop is plain
UDP with no FEC needed (loopback doesn't drop packets).

Usage:
    python gcs_bridge.py --listen-port 15550 --http-port 8080 \\
        --jetson-host 192.168.0.216 --yolo-port 8002 --relay-port 15551
"""

import argparse
import json
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

from fec_core import FecDecoder
from video_protocol import VideoReassembler

DASHBOARD_HTML_PATH = Path(__file__).with_name("dashboard.html")


class TelemetryStore:
    """Thread-safe latest-telemetry holder. The FEC receive thread writes,
    HTTP handler threads read -- one lock, tiny critical sections.

    Readers get new data by *waiting* on the condition variable rather than
    polling it (see wait_for_update) -- a blocked thread costs no CPU, so
    the dashboard only does work when there's actually something new to
    show, not on a fixed timer."""

    def __init__(self):
        self._lock = threading.Lock()
        self._cond = threading.Condition(self._lock)
        self._latest = {}
        self._version = 0
        self._last_update_monotonic = None
        self._parse_errors = 0

    def update(self, payload_bytes):
        try:
            msg = json.loads(payload_bytes.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError):
            with self._lock:
                self._parse_errors += 1
            return
        with self._cond:
            # Payloads can arrive slightly out of order (recovered ones
            # especially); only move the displayed state forward.
            if not self._latest or msg.get("seq", -1) >= self._latest.get("seq", -1):
                self._latest = msg
                self._last_update_monotonic = time.monotonic()
                self._version += 1
                self._cond.notify_all()

    def snapshot(self, link_stats):
        with self._lock:
            latest = dict(self._latest)
            age_ms = (
                (time.monotonic() - self._last_update_monotonic) * 1000.0
                if self._last_update_monotonic is not None
                else None
            )
            parse_errors = self._parse_errors
        return {
            "telemetry": latest,
            "age_ms": age_ms,
            "link": link_stats,
            "parse_errors": parse_errors,
        }

    def wait_for_update(self, last_seen_version, timeout):
        """Blocks until a newer version than last_seen_version is available
        or timeout elapses. Returns (version, changed) -- changed is False
        on a plain timeout, so the caller can send an SSE keep-alive comment
        instead of a data event."""
        with self._cond:
            changed = self._cond.wait_for(lambda: self._version != last_seen_version, timeout=timeout)
            return self._version, changed


class VideoStore:
    """Latest decoded JPEG only. Video loss never affects telemetry."""

    def __init__(self):
        self._lock = threading.Lock()
        self._cond = threading.Condition(self._lock)
        self._latest = None
        self._version = 0
        self._last_update = None

    def update(self, frame):
        with self._cond:
            self._latest = frame
            self._last_update = time.monotonic()
            self._version += 1
            self._cond.notify_all()

    def wait(self, version, timeout=1.0):
        with self._cond:
            self._cond.wait_for(lambda: self._version != version, timeout=timeout)
            return self._version, self._latest

    def snapshot(self):
        with self._lock:
            age_ms = None if self._last_update is None else (time.monotonic() - self._last_update) * 1000
            return self._version, self._latest, age_ms


def receive_loop(decoder: FecDecoder, store: TelemetryStore, stop_event: threading.Event,
                  relay_sock: socket.socket, relay_port: int):
    while not stop_event.is_set():
        for payload in decoder.poll():
            store.update(payload)
            # Fire-and-forget local fan-out for flight_log.py -- see module
            # docstring for why this exists instead of a second UDP bind.
            relay_sock.sendto(payload, ("127.0.0.1", relay_port))


def receive_video_loop(sock: socket.socket, store: VideoStore,
                       stop_event: threading.Event):
    decoder = VideoReassembler()
    sock.settimeout(0.2)
    while not stop_event.is_set():
        try:
            packet, _ = sock.recvfrom(65535)
        except socket.timeout:
            continue
        except OSError:
            break
        frame = decoder.add_packet(packet)
        if frame is not None:
            store.update(frame)


def make_handler(store: TelemetryStore, decoder: FecDecoder, video_store: VideoStore):
    def snapshot():
        _version, frame, age_ms = video_store.snapshot()
        result = store.snapshot(dict(decoder.stats))
        result["video"] = {
            "available": frame is not None,
            "age_ms": age_ms,
            "frame_sequence": frame.get("sequence") if frame is not None else None,
        }
        return result

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            pass  # keep stdout to the receive loop's own status, not HTTP access noise

        def do_GET(self):
            if self.path == "/" or self.path == "/index.html":
                self._send_html()
            elif self.path == "/telemetry":
                self._send_json(snapshot())
            elif self.path == "/telemetry/stream":
                self._send_sse()
            elif self.path == "/config":
                self._send_json({"video_url": "/video.mjpg"})
            elif self.path == "/video.mjpg":
                self._send_video()
            else:
                self.send_response(404)
                self.end_headers()

        def _send_sse(self):
            # Push-only: this thread sits blocked in store.wait_for_update()
            # (a condition variable, not a sleep/poll loop) and only does
            # anything when the receive loop actually decodes a new
            # telemetry message, or every 15s to keep the connection alive.
            # No fixed-interval work happens here.
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-cache")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            last_version = -1
            try:
                while True:
                    version, changed = store.wait_for_update(last_version, timeout=15)
                    if changed:
                        last_version = version
                        data = json.dumps(snapshot())
                        self.wfile.write(f"data: {data}\n\n".encode("utf-8"))
                    else:
                        self.wfile.write(b": keep-alive\n\n")
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError, OSError):
                pass

        def _send_html(self):
            body = DASHBOARD_HTML_PATH.read_text(encoding="utf-8").encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def _send_json(self, obj):
            body = json.dumps(obj).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(body)

        def _send_video(self):
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=FRAME")
            self.send_header("Cache-Control", "no-cache")
            self.end_headers()
            version = -1
            try:
                while True:
                    version, frame = video_store.wait(version, timeout=1.0)
                    if frame is None:
                        continue
                    jpeg = frame["jpeg"]
                    header = (b"--FRAME\r\nContent-Type: image/jpeg\r\nContent-Length: " +
                              str(len(jpeg)).encode("ascii") + b"\r\n\r\n")
                    self.wfile.write(header + jpeg + b"\r\n")
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError, OSError):
                pass

    return Handler


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--listen-port", type=int, default=15550, help="UDP port, matches setting/port.yaml gcs_telem")
    ap.add_argument("--http-port", type=int, default=8080, help="local HTTP port for the dashboard")
    ap.add_argument("--jetson-host", default="192.168.0.216")
    ap.add_argument("--yolo-port", type=int, default=8002, help="setting/port.yaml yolo_live")
    ap.add_argument("--relay-port", type=int, default=15551, help="local fan-out for flight_log.py")
    ap.add_argument("--video-port", type=int, default=15560, help="onboard debug-video UDP port")
    args = ap.parse_args()

    decoder = FecDecoder(args.listen_port)
    store = TelemetryStore()
    stop_event = threading.Event()
    relay_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    video_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    video_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    video_sock.bind(("0.0.0.0", args.video_port))
    video_store = VideoStore()

    receiver_thread = threading.Thread(
        target=receive_loop, args=(decoder, store, stop_event, relay_sock, args.relay_port), daemon=True
    )
    receiver_thread.start()
    video_thread = threading.Thread(
        target=receive_video_loop, args=(video_sock, video_store, stop_event), daemon=True
    )
    video_thread.start()

    handler_cls = make_handler(store, decoder, video_store)
    httpd = ThreadingHTTPServer(("0.0.0.0", args.http_port), handler_cls)

    print(f"gcs_bridge: telemetry udp/{args.listen_port}, dashboard http://localhost:{args.http_port}/")
    print(f"gcs_bridge: debug video udp/{args.video_port} -> /video.mjpg")
    print(f"gcs_bridge: relaying decoded messages to 127.0.0.1:{args.relay_port} for flight_log.py")

    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        video_sock.close()
        httpd.shutdown()


if __name__ == "__main__":
    main()
