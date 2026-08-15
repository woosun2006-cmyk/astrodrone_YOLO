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
import math
import os
import sys
import threading
import time

import cv2
import numpy as np

# Shared with YOLO_MODEL/yolo_live.py and yolo_headless.cpp. Keep in sync.
DEFAULT_CONF = 0.25
TARGET_CONFIRM_FRAMES = 5

# NMS IoU. WHITE_BASKET_SETUP.md 의 재현 조건은 0.45 인데, 근접 구간에서 그
# 값으로는 같은 타겟이 두 박스로 남는다.
#
# 2026-08-15 실측(고도별 12프레임, 타겟 바로 위):
#   고도 1.60 / 1.35 / 1.25 m -> 12/12 프레임 모두 검출 1개
#   고도 1.15 m               -> 8/12 프레임에서 검출 2개
#       conf=0.736 중심=(343.0, 245.0) 크기=97x93
#       conf=0.270 중심=(326.5, 263.9) 크기=76x65
#       두 박스의 IoU = 0.431  ← 0.45 를 간발의 차로 못 넘어 둘 다 살아남는다
#
# 둘 다 화면 중앙 부근이라 기체 자신의 구조물이 아니라 타겟이 쪼개진 것이다.
# 그런데 아래 confirm 규칙은 다중검출 프레임을 '미검출'로 처리하므로, 근접하면
# 확정 스트릭이 매 프레임 리셋되어 confirmed 가 영원히 서지 않는다. 실제로
# 2026-08-15 비행에서 고도 1.24m 부터 target=no 로 끊기고 10초 뒤 긴급착륙했다.
#
# 0.30 이면 IoU 0.431 인 중복 박스가 합쳐진다. 단일 클래스이고 물리적으로 다른
# 두 타겟이 30% 넘게 겹칠 일은 없으므로, 다중검출 규칙의 원래 의도("서로 다른
# 물체가 둘 있으면 어느 쪽이 직전 프레임과 같은지 알 수 없다")는 그대로 지켜진다.
#
# 실기 TensorRT 경로(trt_engine.cpp)도 같은 문제를 겪을 수 있다 - 그쪽 NMS
# 임계값도 함께 점검할 것.
NMS_IOU = 0.30

# 근접 병합 임계값 (박스 크기 대비 중심 간 거리).
#
# NMS 만으로는 부족하다는 것이 2026-08-15 대조 실험에서 드러났다. 타겟 바로 위
# 정지 호버로 20초씩 10Hz 폴링한 결과:
#
#   고도 2.45 m : found 100% (171/171),  confirmed 100%
#   고도 1.20 m : found 100% (167/167),  confirmed   0%   ← 전송 지연 아님
#                 (age 중앙값 61ms, 최대 132ms 로 전부 신선했다)
#
# 1.20 m 에서 검출기 로그의 x_px 가 -28 과 +26 을 오갔다. 두 박스가 약 55 px
# 떨어져 있어 겹치지 않으므로, NMS 는 IoU 를 아무리 낮춰도 이 둘을 합치지
# 못한다. 고도 1.2 m 에서 타겟 평면은 화면에서 약 174 px 이고, 모델이 그 안에서
# 부위별로 박스를 여러 개 낸다. 한 물체인데 다중검출로 간주되어 확정 스트릭이
# 매 프레임 리셋되고, confirmed 가 영원히 서지 않는다.
#
# 그래서 IoU(겹침 비율) 대신 박스 **가장자리 사이의 간격**으로 같은 물체인지
# 판정한다. 중심 간 거리만 보면 임계값 잡기가 애매한데(실측 비율이 0.85~0.89 로
# 한 덩어리 판정선에 아슬아슬하게 걸린다), 가장자리 간격은 부호가 갈린다.
#
# 2026-08-15 실측 — 타겟 바로 위 정지 호버, 두 박스가 타겟의 좌/우 절반이었다:
#
#   고도 1.2 m : 중심 분리 54 px, 반폭 합 61 px -> 간격 -7 px  (겹침)
#   고도 1.0 m : 중심 분리 68 px, 반폭 합 71 px -> 간격 -3 px  (겹침)
#   고도 0.9 m : 중심 분리 74 px, 반폭 합 80 px -> 간격 -6 px  (겹침)
#
# 반면 물리적으로 다른 두 타겟은 떨어진다. 고도 4 m 에서 바구니 박스가 44 px 인데
# 1 m 간격이면 중심 분리 51 px -> 간격 +7 px.
#
# 즉 "한 물체의 조각들은 서로 붙어 있거나 겹치고, 다른 물체는 떨어진다."
# 이 값은 그 판정에 주는 여유폭이며, 큰 쪽 박스 변 길이에 대한 비율이다.
# 0.1 이면 위 두 경우를 모두 안전하게 가른다(4 m 사례의 +7 px 보다 작은 4.4 px).
#
# 다중검출 규칙의 원래 의도("서로 다른 물체가 둘 있으면 어느 쪽이 직전 프레임과
# 같은지 알 수 없다")는 그대로 지켜진다.
MERGE_GAP_FRAC = 0.1

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
    n_raw = 0            # NMS 후 박스 수 (병합 전)
    n_obj = 0            # 근접 병합 후 = 서로 다른 물체 수


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
                "boxes_raw": State.n_raw,
                "objects": State.n_obj,
                "backend": "yolo-onnx",
            }
        body = json.dumps(payload).encode()
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)


def _touching(a, b, gap_frac):
    """두 박스가 붙어 있거나 겹치는가 = 한 물체의 조각인가."""
    gap_x = abs(a["cx"] - b["cx"]) - (a["w"] + b["w"]) / 2.0
    gap_y = abs(a["cy"] - b["cy"]) - (a["h"] + b["h"]) / 2.0
    margin = gap_frac * max(a["w"], a["h"], b["w"], b["h"])
    return gap_x <= margin and gap_y <= margin


def merge_close(dets, gap_frac):
    """붙어 있는 검출들을 한 물체로 묶는다.

    dets 는 conf 내림차순. 각 군집은 최고신뢰 멤버로 대표되고, 그 대표만
    반환한다. 반환 길이가 곧 '프레임에 보이는 서로 다른 물체의 수'이고,
    publish() 의 다중검출 판정이 그 수를 본다.

    NMS 를 대체하는 것이 아니라 그 뒤에 덧붙는다. NMS 는 많이 겹치는 중복을
    지우고, 이건 겹침이 적어 NMS 를 통과했지만 실은 한 물체에 붙어 있는
    조각들을 묶는다.

    군집에 새로 들어온 박스는 대표의 경계를 넓힌다. 타겟이 세 조각 이상으로
    쪼개졌을 때 사슬처럼 이어 붙이기 위해서다.
    """
    if gap_frac < 0 or len(dets) <= 1:
        return dets
    reps = []          # 각 군집의 대표(최고신뢰) + 누적 경계
    for d in dets:                       # conf 내림차순 전제
        hit = None
        for r in reps:
            if _touching(r["_box"], d, gap_frac):
                hit = r
                break
        if hit is None:
            reps.append({**d, "_box": dict(d)})
        else:
            # 대표의 conf/좌표는 유지하고, 경계만 합집합으로 넓힌다.
            b = hit["_box"]
            x0 = min(b["cx"] - b["w"] / 2, d["cx"] - d["w"] / 2)
            x1 = max(b["cx"] + b["w"] / 2, d["cx"] + d["w"] / 2)
            y0 = min(b["cy"] - b["h"] / 2, d["cy"] - d["h"] / 2)
            y1 = max(b["cy"] + b["h"] / 2, d["cy"] + d["h"] / 2)
            b["cx"], b["cy"] = (x0 + x1) / 2, (y0 + y1) / 2
            b["w"], b["h"] = x1 - x0, y1 - y0
    # 대표 위치를 군집 합집합의 중심으로 바꾼다. 조각 하나의 중심을 그대로 쓰면
    # 최고신뢰가 좌/우 조각을 오갈 때마다 x_px 가 튄다 (실측: 1.2 m 에서 -28 과
    # +26 사이를 왕복). 합집합 중심은 그 진동을 없앤다.
    for r in reps:
        b = r.pop("_box")
        r["cx"], r["cy"], r["w"], r["h"] = b["cx"], b["cy"], b["w"], b["h"]
    return reps


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
    def __init__(self, model_path, classes_path, conf, imgsz, nms_iou=NMS_IOU,
                 merge_frac=MERGE_GAP_FRAC):
        self.nms_iou = nms_iou
        self.merge_frac = merge_frac
        self.last_raw = 0          # 병합 전 박스 수 (진단용)
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

        idxs = cv2.dnn.NMSBoxes(boxes.tolist(), score.tolist(), self.conf, self.nms_iou)
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
                         "cx": float(ocx), "cy": float(ocy),
                         "w": float(w[i] / scale), "h": float(h[i] / scale)})
        dets.sort(key=lambda d: d["conf"], reverse=True)
        self.last_raw = len(dets)
        # 한 물체가 여러 박스로 쪼개진 경우를 묶는다. 이 뒤의 개수가 곧
        # '서로 다른 물체의 수'이고, publish() 의 다중검출 판정이 그것을 본다.
        return merge_close(dets, self.merge_frac)


def publish(dets, w, h, n_raw=None):
    """Apply the 5-frame / multi-detection confirm rules and update State."""
    with State.lock:
        State.frames += 1
        State.n_obj = len(dets)
        State.n_raw = n_raw if n_raw is not None else len(dets)

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
            publish(dets, w, h, det.last_raw)
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
                          f"y_px={State.y_px:.1f} "
                          f"박스={State.n_raw}→{State.n_obj}", flush=True)
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
    p.add_argument("--nms-iou", type=float, default=NMS_IOU,
                   help=f"NMS IoU (기본 {NMS_IOU}; 근접에서 타겟이 두 박스로 "
                        f"쪼개지는 것을 막는다. 0.45 는 IoU 0.43 인 중복을 못 걸러낸다)")
    p.add_argument("--merge-frac", type=float, default=MERGE_GAP_FRAC,
                   help=f"근접 병합 여유폭, 박스 변 길이 대비 가장자리 간격 "
                        f"(기본 {MERGE_GAP_FRAC}; 음수면 병합 안 함)")
    p.add_argument("--pipeline", default=None, help="override the full GStreamer pipeline")
    p.add_argument("--test-image", default=None,
                   help="run once on a still image and exit (no Gazebo needed)")
    p.add_argument("--quiet", action="store_true")
    args = p.parse_args()

    det = Detector(args.model, args.classes, args.conf, args.imgsz,
                   args.nms_iou, args.merge_frac)
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
          f"nms_iou={args.nms_iou}, merge_frac={args.merge_frac}, "
          f"confirm={TARGET_CONFIRM_FRAMES} frames", flush=True)
    srv.serve_forever()


if __name__ == "__main__":
    main()
