#!/usr/bin/env python3
"""Serve control/target_distance.cpp's /target endpoint from a REAL YOLO model.

This is the YOLO counterpart to gazebo_camera_target.py. Both serve the exact
same HTTP contract on the same port; the only difference is what produces the
detection:

  gazebo_camera_target.py  HSV colour threshold + largest contour  (no model)
  gazebo_yolo_target.py    YOLO_MODEL/0812best.onnx via cv2.dnn    (this file)

Runs on the PC that hosts Gazebo, NOT on the Jetson. YOLO_MODEL/cpp's
yolo_headless.cpp cannot be used in simulation at all: its GStreamer pipeline
is hardcoded to `nvarguscamerasrc` (Jetson L4T Argus), which has no Gazebo
equivalent and no source-selection flag. Rather than edit that file, this
serves the identical contract target_distance.cpp already polls, so the whole
Jetson pipeline downstream of perception runs unmodified.

WHY cv2.dnn AND NOT torch/ultralytics
-------------------------------------
This PC has no torch, no torchvision, no ultralytics and no onnxruntime
(checked on 2026-08-15). It does have OpenCV 4.6.0 built WITH GStreamer, and
that build's dnn module loads 0812best.onnx directly. So the real trained
weights run with zero installation. 0812best.engine (TensorRT) is NOT usable
here - a TensorRT engine is serialised for one specific GPU + TensorRT version
(the Jetson's), which is exactly why the .onnx is the portable artefact.

MODEL PROVENANCE
----------------
0812best.onnx lives on the Jetson at ~/astro-drone/YOLO_MODEL/ and was copied
to this PC on 2026-08-15 (md5 8b938b4690879b3d2622395570f15efd, 7489758 bytes,
verified identical on both ends). The PC's older checkout of the repo still
carried prototype.onnx/prototype.pt, which no longer exist on the Jetson - do
not use those, they are a stale 320x320 export of an earlier model.

Verified model geometry (measured with cv2.dnn on 2026-08-15, not assumed):
  input   1x3x640x640   FIXED - 320x320, 416x416 and 640x480 all fail in the
                        model.24 Reshape node, whose target shape is baked in
  output  1x25200x6     25200 anchors = (80^2 + 40^2 + 20^2) x 3
                        6 = [cx, cy, w, h, obj_conf, cls0_score]
  classes YOLO_MODEL/classes.txt -> a single class, "astro-drone"

CONFIRMATION LOGIC
------------------
Mirrors the real detector rather than confirming instantly, because
target_distance.cpp and hybrid_guidance treat `confirmed` as a trust gate:

  - DEFAULT_CONF = 0.25            same as yolo_live.py / yolo_headless.cpp
  - TARGET_CONFIRM_FRAMES = 5      5 consecutive passes agreeing on one class
  - a frame with >1 detection counts as a MISS for the streak, because we
    cannot tell whether the top-confidence box is the same physical target

That is the first of the two false-positive gates described in
Document/algorithm.md; the second (the 5 m shell re-verify) lives in
hybrid_guidance and is unaffected by this file.

OUTPUT SCHEMA
-------------
Matches control/target_json.hpp + target_distance.cpp's poll:
  found, confirmed, age_ms, x_px, y_px
x_px/y_px follow setting/cam_sets.yaml's `coord_origin: center` - origin at
frame centre, +x right, +y UP:
  x = px - width / 2
  y = height / 2 - py

USAGE
-----
  # live, against the Gazebo camera RTP stream
  /usr/bin/python3 gazebo_yolo_target.py --port 8002

  # offline self-test: no Gazebo needed, proves the model + postprocess path
  /usr/bin/python3 gazebo_yolo_target.py --test-image ../setting/camera-current.jpg

NOTE: must be run with /usr/bin/python3 (apt OpenCV, GStreamer YES).
venv-ardupilot's cv2 is a pip wheel built WITHOUT GStreamer and cannot open
the pipeline.
"""
import argparse
import http.server
import json
import os
import sys
import threading
import time

import cv2
import numpy as np

# Shared with YOLO_MODEL/yolo_live.py and yolo_headless.cpp. Keep in sync.
DEFAULT_CONF = 0.25
TARGET_CONFIRM_FRAMES = 5
NMS_IOU = 0.45

# setting/rate.yaml: infer_max_fps. target_distance.cpp only consumes at
# sense_cycle_ms (10 Hz), so inferring faster than ~2x that just burns CPU on
# detections thrown away before anyone reads them.
INFER_MAX_FPS = 20

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_MODEL = os.path.join(HERE, "..", "YOLO_MODEL", "0812best.onnx")
DEFAULT_CLASSES = os.path.join(HERE, "..", "YOLO_MODEL", "classes.txt")

# This export's Reshape nodes are baked to 640x640; anything else throws.
DEFAULT_IMGSZ = 640


class State:
    lock = threading.Lock()
    found = False
    confirmed = False
    x_px = 0.0
    y_px = 0.0
    stamp = 0.0          # time.monotonic() of the last detection
    frames = 0           # inference passes completed
    streak = []          # last TARGET_CONFIRM_FRAMES class names (None = miss)
    name = ""
    conf = 0.0
    infer_ms = 0.0


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
                # extra, ignored by target_distance.cpp - useful when watching
                # the endpoint by hand with curl
                "name": State.name,
                "conf": State.conf,
                "infer_ms": round(State.infer_ms, 1),
                "backend": "yolo-onnx",
            }
        body = json.dumps(payload).encode()
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)


def letterbox(frame, size):
    """Resize preserving aspect ratio onto a grey square. Returns (img, scale, padx, pady)."""
    h, w = frame.shape[:2]
    scale = min(size / w, size / h)
    nw, nh = int(round(w * scale)), int(round(h * scale))
    resized = cv2.resize(frame, (nw, nh), interpolation=cv2.INTER_LINEAR)
    canvas = np.full((size, size, 3), 114, np.uint8)
    padx, pady = (size - nw) // 2, (size - nh) // 2
    canvas[pady:pady + nh, padx:padx + nw] = resized
    return canvas, scale, padx, pady


class Detector:
    def __init__(self, model_path, classes_path, conf, imgsz):
        if not os.path.isfile(model_path):
            sys.exit(f"model not found: {model_path}")
        self.net = cv2.dnn.readNetFromONNX(model_path)
        # No CUDA backend here: this OpenCV is the stock apt build, compiled
        # without cuDNN, so DNN_TARGET_CUDA would silently fall back to CPU.
        self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
        self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
        self.conf = conf
        self.imgsz = imgsz
        if os.path.isfile(classes_path):
            with open(classes_path) as fh:
                self.names = [l.strip() for l in fh if l.strip()]
        else:
            self.names = []

        # Fail loudly and immediately on a size mismatch rather than throwing
        # once a frame per second inside the grabber thread. This export's
        # Reshape nodes carry a baked-in target shape, so a wrong --imgsz
        # dies in model.24 with an unhelpful assertion.
        try:
            dummy = np.zeros((imgsz, imgsz, 3), np.uint8)
            blob = cv2.dnn.blobFromImage(dummy, 1 / 255.0, (imgsz, imgsz),
                                         swapRB=True, crop=False)
            self.net.setInput(blob)
            out = np.squeeze(self.net.forward())
        except cv2.error as e:
            sys.exit(f"model rejects --imgsz {imgsz}: {str(e).splitlines()[-1]}\n"
                     f"0812best.onnx is baked to 640x640.")
        self.anchors, self.stride = out.shape[0], out.shape[1]
        print(f"model self-check: input {imgsz}x{imgsz} -> output {out.shape} "
              f"({self.anchors} anchors, {self.stride - 5} class(es))", flush=True)

    def name_of(self, idx):
        return self.names[idx] if idx < len(self.names) else f"class{idx}"

    def detect(self, frame):
        """Return a list of dicts: {name, conf, cx, cy} in ORIGINAL frame pixels."""
        img, scale, padx, pady = letterbox(frame, self.imgsz)
        blob = cv2.dnn.blobFromImage(img, 1 / 255.0, (self.imgsz, self.imgsz),
                                     swapRB=True, crop=False)
        self.net.setInput(blob)
        out = self.net.forward()
        out = np.squeeze(out)              # (6300, 6)
        if out.ndim != 2 or out.shape[1] < 6:
            return []

        obj = out[:, 4]
        cls_scores = out[:, 5:]
        cls_id = np.argmax(cls_scores, axis=1)
        cls_conf = cls_scores[np.arange(len(cls_id)), cls_id]
        score = obj * cls_conf

        keep = score >= self.conf
        if not np.any(keep):
            return []
        rows = out[keep]
        score = score[keep]
        cls_id = cls_id[keep]

        # rows are [cx, cy, w, h, ...] in letterboxed-input pixels
        cx, cy, w, h = rows[:, 0], rows[:, 1], rows[:, 2], rows[:, 3]
        boxes = np.stack([cx - w / 2, cy - h / 2, w, h], axis=1)

        idxs = cv2.dnn.NMSBoxes(boxes.tolist(), score.tolist(), self.conf, NMS_IOU)
        if len(idxs) == 0:
            return []
        idxs = np.array(idxs).flatten()

        dets = []
        for i in idxs:
            # undo the letterbox: remove padding, then the resize scale
            ocx = (cx[i] - padx) / scale
            ocy = (cy[i] - pady) / scale
            dets.append({"name": self.name_of(int(cls_id[i])),
                         "conf": float(score[i]),
                         "cx": float(ocx), "cy": float(ocy)})
        return dets


def publish(dets, w, h):
    """Apply the 5-frame / multi-detection confirm rules and update State."""
    with State.lock:
        State.frames += 1

        # A multi-detection frame is a MISS for the streak: we cannot tell
        # whether the top-confidence box is the same physical target as last
        # frame's. Same rule as yolo_live.py's `multi`.
        multi = len(dets) > 1
        top = max(dets, key=lambda d: d["conf"]) if dets else None

        if top is None:
            State.found = False
            State.name = ""
            State.conf = 0.0
        else:
            State.found = True
            State.name = top["name"]
            State.conf = round(top["conf"], 3)
            # setting/cam_sets.yaml coord_origin: center, +y UP
            State.x_px = top["cx"] - w / 2.0
            State.y_px = h / 2.0 - top["cy"]
            State.stamp = time.monotonic()

        State.streak.append(None if (top is None or multi) else top["name"])
        if len(State.streak) > TARGET_CONFIRM_FRAMES:
            State.streak.pop(0)

        State.confirmed = (
            len(State.streak) == TARGET_CONFIRM_FRAMES
            and State.streak[0] is not None
            and all(s == State.streak[0] for s in State.streak)
        )


def grabber(pipeline, det, show_fps):
    min_period = 1.0 / INFER_MAX_FPS
    while True:
        cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        if not cap.isOpened():
            print(f"camera: cannot open pipeline, retrying in 3s\n  {pipeline}", flush=True)
            time.sleep(3)
            continue
        print("camera: stream open", flush=True)
        last_report = time.monotonic()
        last_infer = 0.0
        count = 0
        while True:
            ok, frame = cap.read()
            if not ok:
                print("camera: stream ended, reopening", flush=True)
                break
            now = time.monotonic()
            if now - last_infer < min_period:
                continue                      # rate cap, drop the frame
            last_infer = now

            t0 = time.monotonic()
            dets = det.detect(frame)
            infer_ms = (time.monotonic() - t0) * 1000.0
            h, w = frame.shape[:2]
            publish(dets, w, h)
            with State.lock:
                State.infer_ms = infer_ms
            count += 1

            now = time.monotonic()
            if show_fps and now - last_report >= 5.0:
                with State.lock:
                    print(f"yolo: {count / (now - last_report):.1f} infer/s "
                          f"({State.infer_ms:.0f} ms) found={State.found} "
                          f"confirmed={State.confirmed} name={State.name or '-'} "
                          f"conf={State.conf} x_px={State.x_px:.1f} "
                          f"y_px={State.y_px:.1f}", flush=True)
                count = 0
                last_report = now
        cap.release()


def run_test_image(det, path):
    frame = cv2.imread(path)
    if frame is None:
        sys.exit(f"cannot read image: {path}")
    h, w = frame.shape[:2]
    print(f"test image: {path}  {w}x{h}")
    t0 = time.monotonic()
    dets = det.detect(frame)
    ms = (time.monotonic() - t0) * 1000.0
    print(f"inference: {ms:.0f} ms on CPU")
    print(f"detections at conf>={det.conf}: {len(dets)}")
    for d in dets:
        x = d["cx"] - w / 2.0
        y = h / 2.0 - d["cy"]
        print(f"  {d['name']:14s} conf={d['conf']:.3f}  "
              f"centre=({d['cx']:.1f},{d['cy']:.1f})  x_px={x:+.1f} y_px={y:+.1f}")
    if not dets:
        print("  (none - the model path works, this image just has no target of"
              " a class the model was trained on)")
    print("\nmodel + postprocess path OK.")


def main():
    p = argparse.ArgumentParser(description="Real-YOLO /target server for the Gazebo harness",
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", type=int, default=8002,
                   help="HTTP port for /target (setting/MAVLink.yaml target_track.yolo_port)")
    p.add_argument("--udp-port", type=int, default=5600,
                   help="UDP port GstCameraPlugin streams RTP H.264 to")
    p.add_argument("--model", default=DEFAULT_MODEL, help="ONNX weights")
    p.add_argument("--classes", default=DEFAULT_CLASSES, help="classes.txt")
    p.add_argument("--imgsz", type=int, default=DEFAULT_IMGSZ,
                   help="model input size; 0812best.onnx is baked to 640")
    p.add_argument("--conf", type=float, default=DEFAULT_CONF,
                   help=f"confidence threshold (yolo_live.py default {DEFAULT_CONF})")
    p.add_argument("--pipeline", default=None, help="override the full GStreamer pipeline")
    p.add_argument("--test-image", default=None,
                   help="run once on a still image and exit (no Gazebo needed)")
    p.add_argument("--quiet", action="store_true")
    args = p.parse_args()

    det = Detector(args.model, args.classes, args.conf, args.imgsz)
    print(f"model: {os.path.realpath(args.model)}", flush=True)
    print(f"classes: {det.names or '(none loaded)'}", flush=True)

    if args.test_image:
        run_test_image(det, args.test_image)
        return

    pipeline = args.pipeline or (
        f"udpsrc port={args.udp_port} caps=application/x-rtp,media=video,"
        "encoding-name=H264,payload=96 ! rtph264depay ! avdec_h264 ! "
        "videoconvert ! video/x-raw,format=BGR ! appsink drop=1 sync=false"
    )

    t = threading.Thread(target=grabber, args=(pipeline, det, not args.quiet), daemon=True)
    t.start()

    srv = http.server.HTTPServer(("0.0.0.0", args.port), Handler)
    print(f"gazebo_yolo_target: /target on 0.0.0.0:{args.port}, "
          f"video from udp:{args.udp_port}, conf>={args.conf}, "
          f"confirm={TARGET_CONFIRM_FRAMES} frames", flush=True)
    srv.serve_forever()


if __name__ == "__main__":
    main()
