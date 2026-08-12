#!/usr/bin/env python3
"""Serve control/target_distance.cpp's /target endpoint from the Gazebo camera.

Runs on the PC that hosts Gazebo (not on the Jetson): the simulated camera
only exists there, and YOLO_MODEL/cpp/yolo_headless.cpp cannot be used at all
in simulation because it opens the camera through a hardcoded
`nvarguscamerasrc` pipeline - the Jetson CSI/Argus stack, which has no Gazebo
equivalent. Rather than modify that file, this serves the identical HTTP
contract target_distance.cpp already polls, so the Jetson pipeline runs
unmodified.

Frames come from ardupilot_gazebo's GstCameraPlugin, which the iris_with_gimbal
model already loads. Publishing `data: true` on the camera's
`.../image/enable_streaming` topic makes it emit H.264 RTP to udp://<host>:5600
(the plugin's configured sink), which OpenCV reads back through GStreamer.

Detection is deliberately simple (HSV colour threshold + largest contour),
not a YOLO model: the point of the simulation harness is to exercise
target_distance.cpp's range maths and control.cpp's GUIDED takeover with a
plausible moving pixel target, not to benchmark perception. Swap
`detect()` for a real inference call when validating the model itself.

Output schema matches control/target_json.hpp + target_distance.cpp's poll:
  found, confirmed, age_ms, x_px, y_px
x_px/y_px follow setting/cam_sets.yaml's `coord_origin: center` convention -
origin at frame centre, +x right, +y UP:
  x = px - width / 2
  y = height / 2 - py
"""
import argparse
import http.server
import json
import threading
import time

import cv2
import numpy as np

# TARGET_CONFIRM_FRAMES equivalent: yolo_live.py only sets "confirmed" after a
# detection persists across this many consecutive frames; target_distance.cpp
# requires confirmed=true, so mirror that behaviour rather than confirming
# instantly.
CONFIRM_FRAMES = 3


class State:
    lock = threading.Lock()
    found = False
    confirmed = False
    x_px = 0.0
    y_px = 0.0
    stamp = 0.0          # time.monotonic() of the last detection
    frames = 0
    streak = 0


class Handler(http.server.BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        pass

    def do_GET(self):
        if self.path != "/target":
            self.send_response(404)
            self.end_headers()
            return
        with State.lock:
            age_ms = (time.monotonic() - State.stamp) * 1000.0 if State.stamp else 1e9
            payload = {
                "found": State.found,
                "confirmed": State.confirmed,
                "age_ms": round(age_ms, 1),
                "x_px": State.x_px,
                "y_px": State.y_px,
                "frames": State.frames,
            }
        body = json.dumps(payload).encode()
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)


def detect(frame, lo, hi):
    """Largest blob within an HSV range -> its centre in pixel coords."""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array(lo), np.array(hi))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    biggest = max(contours, key=cv2.contourArea)
    if cv2.contourArea(biggest) < 80:      # ignore speckle
        return None
    m = cv2.moments(biggest)
    if m["m00"] == 0:
        return None
    return (m["m10"] / m["m00"], m["m01"] / m["m00"])


def grabber(pipeline, lo, hi, show_fps):
    while True:
        cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        if not cap.isOpened():
            print(f"camera: cannot open pipeline, retrying in 3s\n  {pipeline}", flush=True)
            time.sleep(3)
            continue
        print("camera: stream open", flush=True)
        last_report = time.monotonic()
        count = 0
        while True:
            ok, frame = cap.read()
            if not ok:
                print("camera: stream ended, reopening", flush=True)
                break
            count += 1
            h, w = frame.shape[:2]
            hit = detect(frame, lo, hi)
            with State.lock:
                State.frames += 1
                if hit is None:
                    State.found = False
                    State.streak = 0
                    State.confirmed = False
                else:
                    px, py = hit
                    State.found = True
                    State.streak += 1
                    State.confirmed = State.streak >= CONFIRM_FRAMES
                    # setting/cam_sets.yaml coord_origin: center, +y UP
                    State.x_px = px - w / 2.0
                    State.y_px = h / 2.0 - py
                    State.stamp = time.monotonic()
            now = time.monotonic()
            if show_fps and now - last_report >= 5.0:
                with State.lock:
                    print(f"camera: {count / (now - last_report):.1f} fps "
                          f"found={State.found} confirmed={State.confirmed} "
                          f"x_px={State.x_px:.1f} y_px={State.y_px:.1f}", flush=True)
                count = 0
                last_report = now
        cap.release()


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", type=int, default=8002,
                   help="HTTP port for /target (setting/MAVLink.yaml target_track.yolo_port)")
    p.add_argument("--udp-port", type=int, default=5600,
                   help="UDP port GstCameraPlugin streams RTP H.264 to")
    p.add_argument("--pipeline", default=None,
                   help="override the full GStreamer pipeline")
    p.add_argument("--hsv-lo", default="0,120,70",
                   help="lower HSV bound of the target colour")
    p.add_argument("--hsv-hi", default="10,255,255",
                   help="upper HSV bound of the target colour")
    p.add_argument("--quiet", action="store_true")
    args = p.parse_args()

    pipeline = args.pipeline or (
        f"udpsrc port={args.udp_port} caps=application/x-rtp,media=video,"
        "encoding-name=H264,payload=96 ! rtph264depay ! avdec_h264 ! "
        "videoconvert ! video/x-raw,format=BGR ! appsink drop=1 sync=false"
    )
    lo = [int(v) for v in args.hsv_lo.split(",")]
    hi = [int(v) for v in args.hsv_hi.split(",")]

    t = threading.Thread(target=grabber, args=(pipeline, lo, hi, not args.quiet), daemon=True)
    t.start()

    srv = http.server.HTTPServer(("0.0.0.0", args.port), Handler)
    print(f"gazebo_camera_target: /target on 0.0.0.0:{args.port}, "
          f"video from udp:{args.udp_port}", flush=True)
    srv.serve_forever()


if __name__ == "__main__":
    main()
