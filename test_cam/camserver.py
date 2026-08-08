#!/usr/bin/env python3
"""MJPEG camera server for the Jetson CSI camera."""
import os

os.environ.setdefault("OPENBLAS_CORETYPE", "ARMV8")

import argparse
import json
import socket
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn
from urllib.parse import parse_qs, urlparse

import cv2
import numpy as np
import yaml

SETTING_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "setting")
PORT_SETTINGS_PATH = os.path.join(SETTING_DIR, "port.yaml")
CAM_SETTINGS_PATH = os.path.join(SETTING_DIR, "cam_sets.yaml")

with open(PORT_SETTINGS_PATH, encoding="utf-8") as f:
    DEFAULT_PORT = yaml.safe_load(f)["camserver"]

with open(CAM_SETTINGS_PATH, encoding="utf-8") as f:
    _cam_yaml = yaml.safe_load(f)
    _cam_settings = _cam_yaml["camserver"]
    _wb_correction = _cam_yaml["wb_correction"]

DEFAULT_HOST = "0.0.0.0"
DEFAULT_SENSOR_ID = _cam_settings["sensor_id"]
DEFAULT_WIDTH = _cam_settings["width"]
DEFAULT_HEIGHT = _cam_settings["height"]
DEFAULT_FPS = _cam_settings["fps"]
DEFAULT_QUALITY = _cam_settings["quality"]

# This IMX219 module's default AWB ("auto"/off) leaves a strong red cast
# (measured R/G ~1.36). wbmode=3 (fluorescent) was the closest to neutral
# out of the presets tried (R/G ~1.27, B/G ~1.00) - a partial mitigation
# until camera_overrides.isp is installed on the system (needs root).
WBMODE = _cam_settings["wbmode"]

# Extra per-channel gain on top of wbmode, tuned live with fpslog.py's r/g/b
# sliders and saved from there - see setting/cam_sets.yaml's wb_correction.
WB_GAINS_BGR = (_wb_correction["blue_gain"], _wb_correction["green_gain"],
                _wb_correction["red_gain"])


def apply_wb_correction(frame):
    if WB_GAINS_BGR == (1.0, 1.0, 1.0):
        return frame
    out = frame.astype(np.float32)
    for i, gain in enumerate(WB_GAINS_BGR):
        if gain != 1.0:
            out[:, :, i] *= gain
    return np.clip(out, 0, 255).astype(np.uint8)


def gst_pipeline(sensor_id, width, height, fps, flip_method=0):
    return (
        f"nvarguscamerasrc sensor-id={sensor_id} wbmode={WBMODE} ! "
        f"video/x-raw(memory:NVMM), width={width}, height={height}, "
        f"format=NV12, framerate={fps}/1 ! "
        f"nvvidconv flip-method={flip_method} ! "
        f"video/x-raw, width={width}, height={height}, format=BGRx ! "
        "videoconvert ! video/x-raw, format=BGR ! "
        "appsink drop=true max-buffers=1 sync=false"
    )


class Camera:
    def __init__(self, sensor_id, width, height, fps, quality, flip_method):
        self.sensor_id = sensor_id
        self.width = width
        self.height = height
        self.requested_fps = fps
        self.quality = quality
        self.pipeline = gst_pipeline(sensor_id, width, height, fps, flip_method)
        self.cap = cv2.VideoCapture(self.pipeline, cv2.CAP_GSTREAMER)
        if not self.cap.isOpened():
            raise RuntimeError(
                f"Could not open Jetson CSI camera sensor-id={sensor_id}"
            )

        self.frame = None
        self.condition = threading.Condition()
        self.running = True
        self.times = deque(maxlen=60)
        self.sizes = deque(maxlen=60)
        self.count = 0
        self.thread = threading.Thread(target=self._capture, daemon=True)
        self.thread.start()

    def _capture(self):
        while self.running:
            ok, image = self.cap.read()
            if not ok:
                time.sleep(0.05)
                continue
            image = apply_wb_correction(image)

            now = time.monotonic()
            with self.condition:
                self.times.append(now)
                fps = self.measured_fps_locked()

            label = f"cam {self.sensor_id}  {self.width}x{self.height}  {fps:.1f} fps"
            cv2.rectangle(image, (8, 8), (360, 44), (0, 0, 0), -1)
            cv2.putText(
                image,
                label,
                (16, 34),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )

            ok, jpeg = cv2.imencode(
                ".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, self.quality]
            )
            if not ok:
                continue

            data = jpeg.tobytes()
            with self.condition:
                self.frame = data
                self.sizes.append(len(data))
                self.count += 1
                self.condition.notify_all()

    def measured_fps_locked(self):
        if len(self.times) < 2:
            return 0.0
        span = self.times[-1] - self.times[0]
        if span <= 0:
            return 0.0
        return (len(self.times) - 1) / span

    def snapshot(self, previous=None, timeout=2.0):
        with self.condition:
            self.condition.wait_for(
                lambda: self.frame is not None and self.frame is not previous,
                timeout=timeout,
            )
            return self.frame

    def stats(self):
        with self.condition:
            fps = self.measured_fps_locked()
            avg_size = sum(self.sizes) / len(self.sizes) if self.sizes else 0
            return {
                "sensor_id": self.sensor_id,
                "width": self.width,
                "height": self.height,
                "requested_fps": self.requested_fps,
                "measured_fps": round(fps, 2),
                "avg_frame_kb": round(avg_size / 1024, 1),
                "mbps": round(fps * avg_size * 8 / 1000000, 2),
                "frames_total": self.count,
            }

    def close(self):
        self.running = False
        with self.condition:
            self.condition.notify_all()
        self.thread.join(timeout=2)
        self.cap.release()


class ThreadingHTTPServer(ThreadingMixIn, HTTPServer):
    daemon_threads = True


class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        parsed = urlparse(self.path)
        if parsed.path in ("/", "/index.html"):
            return self.index()
        if parsed.path == "/stream.mjpg":
            sensor_id = self.sensor_from_query(parsed.query)
            return self.stream(sensor_id)
        if parsed.path == "/stats.json":
            return self.stats()
        self.send_error(404)

    def sensor_from_query(self, query):
        params = parse_qs(query)
        values = params.get("camera") or params.get("sensor")
        if not values:
            return self.server.default_sensor_id
        try:
            return int(values[0])
        except ValueError:
            return self.server.default_sensor_id

    def index(self):
        sensors = sorted(self.server.cameras)
        links = "".join(
            f"<li><a href='/stream.mjpg?camera={sid}'>camera {sid}</a></li>"
            for sid in sensors
        )
        first = sensors[0]
        body = (
            "<!doctype html><html><head><meta name='viewport' "
            "content='width=device-width,initial-scale=1'>"
            "<title>Jetson Camera</title>"
            "<style>body{margin:0;background:#111;color:#eee;"
            "font-family:sans-serif;text-align:center}"
            "img{max-width:100vw;max-height:88vh}a{color:#8cc8ff}</style>"
            "</head><body><h2>Jetson Camera</h2>"
            f"<img src='/stream.mjpg?camera={first}'>"
            f"<ul>{links}</ul></body></html>"
        ).encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def stream(self, sensor_id):
        camera = self.server.cameras.get(sensor_id)
        if camera is None:
            self.send_error(404, f"Unknown camera {sensor_id}")
            return

        self.send_response(200)
        self.send_header(
            "Content-Type", "multipart/x-mixed-replace; boundary=frame"
        )
        self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
        self.end_headers()
        previous = None
        try:
            while True:
                frame = camera.snapshot(previous)
                if frame is None:
                    continue
                previous = frame
                self.wfile.write(b"--frame\r\n")
                self.wfile.write(b"Content-Type: image/jpeg\r\n")
                self.wfile.write(f"Content-Length: {len(frame)}\r\n\r\n".encode())
                self.wfile.write(frame)
                self.wfile.write(b"\r\n")
        except (BrokenPipeError, ConnectionResetError):
            pass

    def stats(self):
        data = {
            "cameras": [camera.stats() for camera in self.server.cameras.values()]
        }
        body = json.dumps(data, indent=2).encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, fmt, *args):
        print(f"{self.client_address[0]} - {fmt % args}", flush=True)


def local_ip():
    sock = None
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.connect(("8.8.8.8", 80))
        return sock.getsockname()[0]
    except OSError:
        return "127.0.0.1"
    finally:
        if sock is not None:
            sock.close()


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default=DEFAULT_HOST)
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--sensor-id", type=int, action="append", dest="sensors")
    parser.add_argument("--width", type=int, default=DEFAULT_WIDTH)
    parser.add_argument("--height", type=int, default=DEFAULT_HEIGHT)
    parser.add_argument("--fps", type=int, default=DEFAULT_FPS)
    parser.add_argument("--quality", type=int, default=DEFAULT_QUALITY)
    parser.add_argument("--flip-method", type=int, default=0)
    return parser.parse_args()


def main():
    args = parse_args()
    sensor_ids = args.sensors or [DEFAULT_SENSOR_ID]
    cameras = {}
    server = None
    try:
        for sensor_id in sensor_ids:
            cameras[sensor_id] = Camera(
                sensor_id=sensor_id,
                width=args.width,
                height=args.height,
                fps=args.fps,
                quality=args.quality,
                flip_method=args.flip_method,
            )

        server = ThreadingHTTPServer((args.host, args.port), Handler)
        server.cameras = cameras
        server.default_sensor_id = sensor_ids[0]
        print(f"Camera stream listening on http://{args.host}:{args.port}", flush=True)
        if args.host == "0.0.0.0":
            print(f"LAN URL: http://{local_ip()}:{args.port}", flush=True)
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nstopping...", flush=True)
    finally:
        if server is not None:
            server.server_close()
        for camera in cameras.values():
            camera.close()
        print("camera released. bye.", flush=True)


if __name__ == "__main__":
    main()
