#!/usr/bin/env python3
"""Always-on camera stream with live YOLO detection, sibling to
../test_cam/fpslog.py (same dashboard shape, different payload).

Start it once and leave it running. Open the printed URL: the annotated
video and the inference-fps chart are always live. Drag the conf slider
while watching real drone footage - info.txt says the threshold was never
tuned offline ("실사용 임계값은 웹캠 스트리밍으로 직접 찾을 것"), this is that
tool. Type how many seconds you want to measure, press Record, and it
writes a plain-text report next to this script when the timer runs out.
"""
import os

os.environ.setdefault("OPENBLAS_CORETYPE", "ARMV8")

import http.server
import json
import socket
import socketserver
import statistics
import sys
import threading
import time
import urllib.parse
import warnings
from collections import deque

import cv2
import numpy as np
import yaml

SETTING_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "setting")
CAM_SETTINGS_PATH = os.path.join(SETTING_DIR, "cam_sets.yaml")
RATE_SETTINGS_PATH = os.path.join(SETTING_DIR, "rate.yaml")

with open(CAM_SETTINGS_PATH, encoding="utf-8") as f:
    _wb_correction = yaml.safe_load(f)["wb_correction"]

with open(RATE_SETTINGS_PATH, encoding="utf-8") as f:
    # Shared with control/target_distance.cpp - see setting/rate.yaml for
    # why these two live together.
    _rate = yaml.safe_load(f)

# ---------------------------------------------------------------------------
# SETTINGS - edit these
# ---------------------------------------------------------------------------
DEFAULT_RECORD_SEC = 30    # value pre-filled in the browser box
SAMPLE_INTERVAL_SEC = 0.5  # how often the chart/report samples
LIVE_WINDOW_SEC = 40       # how much history the idle chart shows

LOG_PREFIX = "yolo_log"    # saved as <prefix>_MMDD_HHMM.txt

PORT = 8002                # camserver.py uses 8000, fpslog.py uses 8001;
                            # different port so all three can run at once

HERE = os.path.dirname(os.path.abspath(__file__))
# TensorRT engine (control/README.md), built FP32 - not --half, see HALF's
# comment below. DetectMultiBackend (yolov5/models/common.py) auto-detects
# the .engine extension and loads it through the same torch.hub.load(...,
# "custom", ...) call below as a .pt file would use, no other code here
# needs to change for this swap.
WEIGHTS = os.path.join(HERE, "prototype.engine")
YOLOV5_ROOT = "/home/astro/yolov5"  # local clone; hubconf.py loads from here,
                                     # no network / torch.hub cache needed

# inferer()'s own detection-rate cap, in fps - see setting/rate.yaml for why
# this lives there instead of as a bare constant here.
INFER_MAX_FPS = _rate["infer_max_fps"]

IMG_SIZE = 320       # inference resolution. run_yolov5n.sh used 320 on this
                     # Jetson Nano for interactive fps; info.txt's accuracy
                     # numbers were measured at 640 - raise this to 640 to
                     # match that report, at the cost of fps.
DEFAULT_CONF = 0.25  # starting point only - drag the slider in the UI.
HALF = False         # info.txt: FP16 collapsed accuracy on the dev GTX 1660
                     # SUPER (mAP50 0.995 -> 0.659). Unverified on this
                     # Jetson's GPU - leave False until checked. Only
                     # affects a .pt weights path (calls model.model.half());
                     # prototype.engine above was deliberately built FP32
                     # (not --half) for the same reason, so this is doubly
                     # moot until FP16 is actually verified safe here.

# /target's age_ms only tells a consumer how *recent* a detection is, not
# how *stable* it is - a single-frame misdetection is just as "fresh" as a
# real target. TARGET_CONFIRM_FRAMES requires this many consecutive
# inference passes to agree (found=True, same class) before /target reports
# confirmed=True; consumers like control/target_distance.cpp should treat
# confirmed=False the same as found=False. Placeholder value - tune against
# this model's actual false-positive rate.
TARGET_CONFIRM_FRAMES = 5

# /target's age_ms only tells a consumer how *recent* a detection is, not
# how *stable* it is - a single-frame misdetection is just as "fresh" as a
# real target. TARGET_CONFIRM_FRAMES requires this many consecutive
# inference passes to agree (found=True, same class) before /target reports
# confirmed=True; consumers like control/target_distance.cpp should treat
# confirmed=False the same as found=False. Placeholder value - tune against
# this model's actual false-positive rate.
TARGET_CONFIRM_FRAMES = 5

# Jetson Nano + IMX219 via nvarguscamerasrc, same camera fpslog.py uses, but
# through OpenCV/GStreamer (appsink) instead of a raw fdsink pipe, since we
# need numpy frames for the model rather than a JPEG byte stream. Pipeline
# matches the one already proven working in ~/yolo-jetson/run_yolov5n.sh.
SENSOR_ID = 0
WIDTH, HEIGHT, CAM_FPS = 640, 480, 30

# This IMX219 module's default AWB leaves a strong red cast (measured
# R/G ~1.36 with AWB off/auto). wbmode=3 (fluorescent), same as
# test_cam/camserver.py and fpslog.py, was closest to neutral out of the
# presets tried.
WBMODE = 3
# ---------------------------------------------------------------------------

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

model = None  # set once by _load_model(), read by inferer()

state = {"raw_frame": None, "cap_times": deque(maxlen=30), "cap_count": 0,
         "frame": None, "infer_times": deque(maxlen=30), "last_dets": [],
         "last_latency_ms": 0.0, "cap": None,
         "target": {"found": False, "confirmed": False, "t": 0.0},
         "target_streak": deque(maxlen=TARGET_CONFIRM_FRAMES)}
cap_cond = threading.Condition()
frame_cond = threading.Condition()
stopping = threading.Event()

conf_state = {"value": DEFAULT_CONF}
conf_lock = threading.Lock()

history = deque(maxlen=int(LIVE_WINDOW_SEC / SAMPLE_INTERVAL_SEC) + 2)
hist_lock = threading.Lock()

rec = {"active": False, "rows": [], "t0": 0.0, "duration": 0.0,
       "saved": "", "started_str": ""}
rec_lock = threading.Lock()


def grabber():
    """Feeds raw BGR frames from nvarguscamerasrc into state["raw_frame"]."""
    pipeline = (
        f"nvarguscamerasrc sensor-id={SENSOR_ID} wbmode={WBMODE} ! "
        f"video/x-raw(memory:NVMM),width={WIDTH},height={HEIGHT},"
        f"format=NV12,framerate={CAM_FPS}/1 ! "
        "nvvidconv flip-method=0 ! "
        f"video/x-raw,width={WIDTH},height={HEIGHT},format=BGRx ! "
        "videoconvert ! video/x-raw,format=BGR ! appsink drop=1"
    )
    while not stopping.is_set():
        cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        if not cap.isOpened():
            time.sleep(3)
            continue
        state["cap"] = cap
        while not stopping.is_set():
            ok, frame = cap.read()
            if not ok:
                break
            frame = apply_wb_correction(frame)
            now = time.monotonic()
            with cap_cond:
                state["raw_frame"] = frame
                state["cap_times"].append(now)
                state["cap_count"] += 1
                cap_cond.notify_all()
        cap.release()
        if stopping.is_set():
            return
        time.sleep(3)


def inferer():
    """Runs the model on whatever frame is newest, drops backlog frames.

    render() draws boxes in place on the RGB view of the frame; since it's
    a view (frame[:, :, ::-1]), not a copy, the underlying buffer is shared
    with the BGR array we then re-flip for JPEG encoding - no extra copy.

    Throttled to INFER_MAX_FPS (setting/rate.yaml) rather than running flat
    out at whatever the model/camera can sustain: target_distance.cpp only
    ever consumes a detection every sense_cycle_ms (same file), so inference
    beyond roughly 2x that rate just burns GPU/thermal budget on results
    nobody reads before they're superseded.
    """
    last = None
    min_interval_sec = 1.0 / INFER_MAX_FPS
    last_infer_start = 0.0
    while not stopping.is_set():
        with cap_cond:
            while state["raw_frame"] is last or state["raw_frame"] is None:
                cap_cond.wait(timeout=1)
                if stopping.is_set():
                    return
            frame = state["raw_frame"]
        last = frame

        wait_sec = min_interval_sec - (time.monotonic() - last_infer_start)
        if wait_sec > 0:
            time.sleep(wait_sec)
        last_infer_start = time.monotonic()

        with conf_lock:
            model.conf = conf_state["value"]

        t0 = time.monotonic()
        results = model(frame[:, :, ::-1], size=IMG_SIZE)
        latency_ms = (time.monotonic() - t0) * 1000

        annotated_rgb = results.render()[0]
        ok, jpg = cv2.imencode(".jpg", annotated_rgb[:, :, ::-1],
                               [cv2.IMWRITE_JPEG_QUALITY, 85])
        if not ok:
            continue

        df = results.pandas().xyxy[0]
        dets = [{"name": row["name"], "conf": round(float(row["confidence"]), 3)}
                for _, row in df.iterrows()]

        # Highest-confidence detection's box center, converted to the
        # center-origin frame from ../setting/cam_sets.yaml (coord_origin:
        # center; +x right, +y up) - this is what target_distance.cpp polls
        # over /target to compute the target's offset from the camera axis.
        multi = len(df) > 1
        if not df.empty:
            top = df.loc[df["confidence"].idxmax()]
            px = float(top["xmin"] + top["xmax"]) / 2.0
            py = float(top["ymin"] + top["ymax"]) / 2.0
            top_name = str(top["name"])
            target = {"found": True, "name": top_name,
                      "conf": round(float(top["confidence"]), 3),
                      "x_px": round(px - WIDTH / 2.0, 1),
                      "y_px": round(HEIGHT / 2.0 - py, 1),
                      "detections": len(df), "multi": multi}
        else:
            top_name = None
            target = {"found": False, "detections": 0, "multi": False}

        now = time.monotonic()
        with frame_cond:
            state["frame"] = jpg.tobytes()
            state["infer_times"].append(now)
            state["last_dets"] = dets
            state["last_latency_ms"] = round(latency_ms, 1)

            # None breaks the streak the same way a missing detection or a
            # class change does. A multi-detection frame also counts as
            # None for streak purposes: with two+ boxes in frame we can't
            # tell if the top-confidence one is the same physical target
            # frame-to-frame (could be swapping between look-alikes), so it
            # can't be allowed to confirm a streak even though something
            # was technically found.
            streak = state["target_streak"]
            streak.append(None if multi else top_name)
            target["confirmed"] = (
                not multi
                and top_name is not None
                and len(streak) == TARGET_CONFIRM_FRAMES
                and all(n == top_name for n in streak)
            )
            target["t"] = now
            state["target"] = target
            frame_cond.notify_all()


_cpu_prev = {}
_thermal_zone_path = None


def _cpu_thermal_zone():
    """Path to the CPU thermal zone's temp file, found by name.

    thermal_zone0 on this board is "AO-therm" (always-on rail), not the CPU -
    the CPU one is named "CPU-therm" elsewhere in the list.
    """
    global _thermal_zone_path
    if _thermal_zone_path:
        return _thermal_zone_path
    base = "/sys/class/thermal"
    try:
        for name in sorted(os.listdir(base)):
            if not name.startswith("thermal_zone"):
                continue
            with open(os.path.join(base, name, "type")) as f:
                if f.read().strip() == "CPU-therm":
                    _thermal_zone_path = os.path.join(base, name, "temp")
                    return _thermal_zone_path
    except Exception:
        pass
    _thermal_zone_path = "/sys/class/thermal/thermal_zone0/temp"
    return _thermal_zone_path


def sysinfo():
    """CPU / RAM / clock / temperature of the board itself."""
    out = {}
    try:
        with open("/proc/stat") as f:
            parts = [float(x) for x in f.readline().split()[1:8]]
        idle, total = parts[3] + parts[4], sum(parts)
        if _cpu_prev:
            dt, di = total - _cpu_prev["total"], idle - _cpu_prev["idle"]
            out["cpu_pct"] = round(100 * (dt - di) / dt, 1) if dt > 0 else 0.0
        _cpu_prev["total"], _cpu_prev["idle"] = total, idle
    except Exception:
        pass
    try:
        info = {}
        with open("/proc/meminfo") as f:
            for line in f:
                k, _, v = line.partition(":")
                info[k] = float(v.split()[0])  # kB
        tot, av = info.get("MemTotal", 0), info.get("MemAvailable", 0)
        if tot:
            out["mem_pct"] = round(100 * (tot - av) / tot, 1)
            out["mem_used_mb"] = round((tot - av) / 1024)
            out["mem_total_mb"] = round(tot / 1024)
    except Exception:
        pass
    try:
        with open("/sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq") as f:
            out["clock_mhz"] = round(int(f.read().strip()) / 1000)
    except Exception:
        pass
    try:
        with open(_cpu_thermal_zone()) as f:
            out["temp_c"] = round(int(f.read().strip()) / 1000, 1)
    except Exception:
        pass
    return out


def sample():
    with cap_cond:
        cap_times = list(state["cap_times"])
    with frame_cond:
        infer_times = list(state["infer_times"])
        dets = list(state["last_dets"])
        latency_ms = state["last_latency_ms"]

    def fps_of(times):
        if len(times) < 2:
            return 0.0
        span = times[-1] - times[0]
        return round((len(times) - 1) / span, 2) if span > 0 else 0.0

    row = {
        "cap_fps": fps_of(cap_times), "infer_fps": fps_of(infer_times),
        "latency_ms": latency_ms, "det_count": len(dets),
        "avg_conf": round(sum(d["conf"] for d in dets) / len(dets), 3) if dets else 0.0,
        "top_conf": round(max((d["conf"] for d in dets), default=0.0), 3),
        "conf_threshold": conf_state["value"],
    }
    row.update(sysinfo())
    return row


def sampler():
    """Runs for the whole life of the program. Feeds the live chart, and
    when a recording is armed, collects its rows and saves on completion."""
    while not stopping.is_set():
        row = sample()
        now = time.monotonic()
        with hist_lock:
            history.append((now, row))
        with rec_lock:
            if rec["active"]:
                elapsed = now - rec["t0"]
                r = dict(row)
                r["elapsed_s"] = round(elapsed, 2)
                rec["rows"].append(r)
                if elapsed >= rec["duration"]:
                    finish_locked()
        time.sleep(SAMPLE_INTERVAL_SEC)


def finish_locked():
    """Write the report. Caller must hold rec_lock."""
    rows, started_str = rec["rows"], rec["started_str"]
    duration = rec["duration"]
    rec["active"] = False
    if not rows:
        return
    name = f"{LOG_PREFIX}_{time.strftime('%m%d_%H%M')}.txt"
    log_dir = os.path.join(HERE, "log")
    os.makedirs(log_dir, exist_ok=True)
    path = os.path.join(log_dir, name)
    text = report(rows, started_str, duration)
    with open(path, "w") as f:
        f.write(text + "\n")
    rec["saved"] = name
    print("\n" + text, flush=True)
    print(f"saved: {path}\n", flush=True)


# --- reporting --------------------------------------------------------------

def col(rows, key):
    out = []
    for r in rows:
        try:
            out.append(float(r.get(key, "")))
        except (TypeError, ValueError):
            pass
    return out


def describe(vals, unit="", scale=1.0):
    if not vals:
        return "n/a"
    v = [x * scale for x in vals]
    s = (f"mean {statistics.mean(v):7.2f}{unit}   "
         f"min {min(v):7.2f}{unit}   max {max(v):7.2f}{unit}")
    if len(v) > 1:
        s += f"   stdev {statistics.stdev(v):5.2f}"
    return s


def report(rows, started_str, duration):
    W = 68
    infer_fps = col(rows, "infer_fps")
    cap_fps = col(rows, "cap_fps")
    L = ["=" * W, f" YOLO LOG   {started_str}", "=" * W,
         f" requested      : {duration:.0f} s",
         f" weights        : {os.path.basename(WEIGHTS)}",
         f" classes        : {list(model.names.values()) if model else 'n/a'}",
         f" img size       : {IMG_SIZE}",
         f" conf threshold : {rows[-1].get('conf_threshold', DEFAULT_CONF)}",
         f" samples        : {len(rows)}", "",
         "-" * W, " SUMMARY", "-" * W,
         f" capture fps  {describe(cap_fps)}",
         f" infer fps    {describe(infer_fps)}",
         f" latency      {describe(col(rows, 'latency_ms'), ' ms')}",
         f" detections   {describe(col(rows, 'det_count'))}",
         f" avg conf     {describe(col(rows, 'avg_conf'))}",
         f" top conf     {describe(col(rows, 'top_conf'))}",
         f" cpu          {describe(col(rows, 'cpu_pct'), ' %')}",
         f" temp         {describe(col(rows, 'temp_c'), ' C')}",
         f" clock        {describe(col(rows, 'clock_mhz'), ' MHz')}", ""]

    if infer_fps and cap_fps:
        ratio = statistics.mean(infer_fps) / statistics.mean(cap_fps) if statistics.mean(cap_fps) else 0
        L += ["-" * W, " VERDICT", "-" * W]
        if ratio > 0.9:
            L.append(" inference keeps up with capture - not the bottleneck.")
        else:
            L.append(f" inference runs at {ratio*100:.0f}% of capture fps -")
            L.append(" the model, not the camera, is the ceiling here.")
            L.append(f" try lowering IMG_SIZE (currently {IMG_SIZE}) or set")
            L.append(" HALF=True and verify accuracy still holds on this GPU.")
        L.append("")

    L += ["-" * W, " INFER FPS OVER TIME", "-" * W]
    top = max(infer_fps) if infer_fps else 1.0
    for r in rows:
        n = int(round(r["infer_fps"] / top * 40)) if top else 0
        L.append(f" {r['elapsed_s']:6.1f}s {r['infer_fps']:6.2f} "
                 f"|{'#'*n}{'.'*(40-n)}|  det={r['det_count']}")
    L += ["", "=" * W]
    return "\n".join(L)


# --- web --------------------------------------------------------------------

PAGE = """<!doctype html><meta charset=utf-8>
<meta name=viewport content='width=device-width,initial-scale=1'>
<title>yolo monitor</title>
<style>
 *{box-sizing:border-box}
 body{margin:0;padding:16px;background:#0d0f12;color:#e8eaed;
      font-family:system-ui,-apple-system,sans-serif;max-width:1400px}
 h1{font-size:14px;font-weight:600;color:#9aa0a6;margin:0 0 10px}
 .ctl{display:flex;gap:8px;align-items:center;flex-wrap:wrap;
      background:#16191d;border:1px solid #23272c;border-radius:8px;
      padding:10px 12px;margin-bottom:12px}
 .ctl label{font-size:12.5px;color:#9aa0a6}
 input[type=number]{width:72px;background:#0d0f12;border:1px solid #3b4048;
       color:#e8eaed;border-radius:5px;padding:6px 8px;font:inherit;
       font-size:13px;font-variant-numeric:tabular-nums}
 input[type=range]{width:140px}
 button{background:#8ab4f8;color:#0d0f12;border:0;border-radius:5px;
        padding:7px 15px;font:inherit;font-size:13px;font-weight:600;
        cursor:pointer}
 button:hover{background:#a6c6fa}
 button.stop{background:#f28b82}
 button.stop:hover{background:#f5a29b}
 .st{font-size:12.5px;color:#6b7075;margin-left:auto}
 .bar{height:4px;background:#23272c;border-radius:2px;overflow:hidden;
      margin-bottom:14px}
 .bar i{display:block;height:100%;background:#8ab4f8;width:0;
        transition:width .4s linear}
 img{width:100%;border-radius:8px;display:block;background:#000;
     margin-bottom:14px}
 .row{display:grid;grid-template-columns:1fr 232px;gap:14px;align-items:start}
 @media(max-width:820px){.row{grid-template-columns:1fr}}
 canvas{width:100%;height:220px;background:#16191d;border:1px solid #23272c;
        border-radius:8px;display:block}
 .panel{background:#16191d;border:1px solid #23272c;border-radius:8px;
        padding:12px 14px}
 .panel h2{font-size:11px;font-weight:600;color:#6b7075;margin:0 0 9px;
           letter-spacing:.06em;text-transform:uppercase}
 .panel h2:not(:first-child){margin-top:15px;padding-top:13px;
                             border-top:1px solid #23272c}
 .r{display:flex;justify-content:space-between;align-items:baseline;
    padding:2.5px 0;font-size:12.5px}
 .r .k{color:#9aa0a6}
 .r .v{font-variant-numeric:tabular-nums;font-weight:600}
 .big{font-size:27px;font-weight:600;color:#8ab4f8;
      font-variant-numeric:tabular-nums;line-height:1.1}
 .big small{font-size:12px;color:#6b7075;font-weight:400;margin-left:3px}
 .warn{color:#f6c445} .bad{color:#f28b82} .ok{color:#81c995}
</style>
<body>
<h1>yolo monitor</h1>

<div class=ctl>
  <label for=sec>measure for</label>
  <input id=sec type=number min=1 max=3600 value=__DEF__>
  <label>seconds</label>
  <button id=go>Record</button>
  <label for=conf style="margin-left:10px">conf</label>
  <input id=conf type=range min=0 max=1 step=0.01 value=__CONF__>
  <span id=confval style="font-variant-numeric:tabular-nums">__CONF__</span>
  <span class=st id=st>streaming</span>
</div>
<div class=bar><i id=prog></i></div>

<img src="/stream">

<div class=row>
  <canvas id=chart></canvas>
  <div class=panel>
    <h2>detection</h2>
    <div class=big><span id=dc>--</span><small>boxes now</small></div>
    <div class=r><span class=k>top conf</span><span class=v id=topc>--</span></div>
    <div class=r><span class=k>avg conf</span><span class=v id=avgc>--</span></div>

    <h2>camera / inference</h2>
    <div class=r><span class=k>resolution</span><span class=v id=res>--</span></div>
    <div class=r><span class=k>capture fps</span><span class=v id=capfps>--</span></div>
    <div class=r><span class=k>infer fps</span><span class=v id=inffps>--</span></div>
    <div class=r><span class=k>latency</span><span class=v id=lat>--</span></div>

    <h2>jetson nano</h2>
    <div class=r><span class=k>cpu</span><span class=v id=cpu>--</span></div>
    <div class=r><span class=k>ram</span><span class=v id=ram>--</span></div>
    <div class=r><span class=k>clock</span><span class=v id=clk>--</span></div>
    <div class=r><span class=k>temp</span><span class=v id=tmp>--</span></div>

    <h2>last saved</h2>
    <div class=r><span class=v id=saved style="font-size:11.5px">--</span></div>
  </div>
</div>

<script>
const cv=document.getElementById('chart'), cx=cv.getContext('2d');
const $=id=>document.getElementById(id);
let recording=false, xmax=__WIN__, chartTop=1;

function draw(pts){
  const dpr=window.devicePixelRatio||1, W=cv.clientWidth, H=cv.clientHeight;
  cv.width=W*dpr; cv.height=H*dpr; cx.setTransform(dpr,0,0,dpr,0,0);
  cx.clearRect(0,0,W,H);
  const L=38,R=10,T=12,B=20, w=W-L-R, h=H-T-B, ymax=chartTop*1.15||1;

  cx.strokeStyle='#23272c'; cx.fillStyle='#6b7075';
  cx.font='10px system-ui'; cx.lineWidth=1;
  for(let i=0;i<=4;i++){
    const v=ymax*i/4, y=T+h-(v/ymax)*h;
    cx.beginPath(); cx.moveTo(L,y); cx.lineTo(L+w,y); cx.stroke();
    cx.fillText(v.toFixed(1), 8, y+3);
  }
  cx.fillStyle='#6b7075'; cx.fillText('infer fps', L+w-46, T+8);

  if(!pts.length) return;
  const xs=t=>L+(xmax?Math.min(t/xmax,1):0)*w;
  const ys=v=>T+h-(Math.min(v,ymax)/ymax)*h;
  const color=recording?'#8ab4f8':'#5f6976';

  cx.beginPath(); cx.moveTo(xs(pts[0].t), ys(pts[0].v));
  pts.forEach(p=>cx.lineTo(xs(p.t), ys(p.v)));
  cx.lineTo(xs(pts[pts.length-1].t), T+h); cx.lineTo(xs(pts[0].t), T+h);
  cx.closePath();
  const g=cx.createLinearGradient(0,T,0,T+h);
  g.addColorStop(0, recording?'rgba(138,180,248,.3)':'rgba(95,105,118,.22)');
  g.addColorStop(1,'rgba(0,0,0,0)');
  cx.fillStyle=g; cx.fill();

  cx.beginPath(); cx.moveTo(xs(pts[0].t), ys(pts[0].v));
  pts.forEach(p=>cx.lineTo(xs(p.t), ys(p.v)));
  cx.strokeStyle=color; cx.lineWidth=1.8; cx.stroke();

  cx.fillStyle='#6b7075';
  cx.fillText('0s', L, H-6);
  cx.fillText(xmax.toFixed(0)+'s', L+w-20, H-6);
}

$('go').onclick=async()=>{
  if(recording){ await fetch('/stop'); }
  else{
    const s=parseFloat($('sec').value)||30;
    await fetch('/start?sec='+s);
  }
  tick();
};

$('conf').oninput=()=>{ $('confval').textContent=$('conf').value; };
$('conf').onchange=async()=>{
  await fetch('/conf?value='+$('conf').value);
};

async function tick(){
  try{
    const s=await (await fetch('/data')).json();
    const c=s.current||{};
    recording=s.recording; xmax=s.xmax||__WIN__;

    $('dc').textContent=c.det_count??0;
    $('topc').textContent=c.top_conf!=null?c.top_conf.toFixed(2):'--';
    $('avgc').textContent=c.avg_conf!=null?c.avg_conf.toFixed(2):'--';

    $('res').textContent=s.width+'x'+s.height;
    $('capfps').textContent=(c.cap_fps??0).toFixed(1);
    $('inffps').textContent=(c.infer_fps??0).toFixed(1);
    $('lat').textContent=(c.latency_ms??0).toFixed(0)+' ms';

    const cpu=c.cpu_pct;
    $('cpu').textContent=cpu!=null?cpu.toFixed(0)+' %':'--';
    $('cpu').className='v'+(cpu>85?' bad':cpu>60?' warn':'');
    $('ram').textContent=c.mem_used_mb!=null
      ? c.mem_used_mb+' / '+c.mem_total_mb+' MB':'--';
    $('clk').textContent=c.clock_mhz?c.clock_mhz+' MHz':'--';
    const t=c.temp_c;
    $('tmp').textContent=t!=null?t.toFixed(1)+' C':'--';
    $('tmp').className='v'+(t>75?' bad':t>65?' warn':' ok');
    $('saved').textContent=s.saved||'--';

    $('go').textContent=recording?'Stop':'Record';
    $('go').className=recording?'stop':'';
    $('sec').disabled=recording;
    $('st').textContent=recording
      ? 'recording '+s.elapsed.toFixed(0)+' / '+s.duration.toFixed(0)+'s'
      : 'streaming (idle)';
    $('prog').style.width=recording
      ? Math.min(100,(s.elapsed/s.duration)*100)+'%' : '0%';

    const pts=(s.points||[]).map(p=>({t:p.t, v:p.infer_fps}));
    chartTop=Math.max(1, ...pts.map(p=>p.v), c.infer_fps||0);
    draw(pts);
  }catch(err){}
}
setInterval(tick, 500); tick();
window.addEventListener('resize', ()=>tick());
</script>
"""


class Handler(http.server.BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass

    def _send(self, body, ctype):
        self.send_response(200)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _json(self, obj):
        self._send(json.dumps(obj).encode(), "application/json")

    def do_GET(self):
        path, _, query = self.path.partition("?")
        q = urllib.parse.parse_qs(query)

        if path == "/":
            page = (PAGE.replace("__DEF__", str(DEFAULT_RECORD_SEC))
                        .replace("__CONF__", str(conf_state["value"]))
                        .replace("__WIN__", str(LIVE_WINDOW_SEC)))
            return self._send(page.encode(), "text/html; charset=utf-8")

        if path == "/conf":
            try:
                v = max(0.0, min(1.0, float(q.get("value", ["0.25"])[0])))
                with conf_lock:
                    conf_state["value"] = v
            except ValueError:
                pass
            return self._json({"ok": True, "value": conf_state["value"]})

        if path == "/start":
            try:
                sec = max(1.0, min(3600.0, float(q.get("sec", ["30"])[0])))
            except ValueError:
                sec = float(DEFAULT_RECORD_SEC)
            with rec_lock:
                rec.update(active=True, rows=[], t0=time.monotonic(),
                           duration=sec, saved="",
                           started_str=time.strftime("%Y-%m-%d %H:%M:%S"))
            print(f"recording {sec:.0f}s...", flush=True)
            return self._json({"ok": True, "duration": sec})

        if path == "/stop":
            with rec_lock:
                if rec["active"]:
                    finish_locked()
            return self._json({"ok": True})

        if path == "/data":
            return self._json(self._data())

        if path == "/target":
            with frame_cond:
                t = dict(state["target"])
            age_s = time.monotonic() - t["t"] if t.get("t") else None
            t["age_ms"] = round(age_s * 1000, 1) if age_s is not None else None
            t["width"], t["height"] = WIDTH, HEIGHT
            return self._json(t)

        if path == "/stream":
            return self.stream()

        self.send_error(404)

    def _data(self):
        with rec_lock:
            active, rows = rec["active"], list(rec["rows"])
            duration, saved = rec["duration"], rec["saved"]
            t0 = rec["t0"]
        with hist_lock:
            hist = list(history)

        if active and rows:
            pts = [{"t": r["elapsed_s"], "infer_fps": r["infer_fps"]} for r in rows]
            xmax = duration
            elapsed = time.monotonic() - t0
        else:
            base = hist[0][0] if hist else 0
            pts = [{"t": round(t - base, 2), "infer_fps": r["infer_fps"]} for t, r in hist]
            xmax = LIVE_WINDOW_SEC
            elapsed = 0.0

        cur = (rows[-1] if active and rows
               else (hist[-1][1] if hist else {}))
        return {"points": pts, "current": cur, "width": WIDTH, "height": HEIGHT,
                "recording": active, "elapsed": round(elapsed, 1),
                "duration": duration, "xmax": xmax, "saved": saved}

    def stream(self):
        self.send_response(200)
        self.send_header("Content-Type",
                         "multipart/x-mixed-replace; boundary=FRAME")
        self.end_headers()
        last = None
        try:
            while not stopping.is_set():
                with frame_cond:
                    while state["frame"] is last or state["frame"] is None:
                        frame_cond.wait(timeout=1)
                        if stopping.is_set():
                            return
                    frame = state["frame"]
                last = frame
                self.wfile.write(b"--FRAME\r\n")
                self.wfile.write(b"Content-Type: image/jpeg\r\n")
                self.wfile.write(f"Content-Length: {len(frame)}\r\n\r\n".encode())
                self.wfile.write(frame)
                self.wfile.write(b"\r\n")
        except (BrokenPipeError, ConnectionResetError):
            pass


class Server(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True


def lan_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))
        return s.getsockname()[0]
    except Exception:
        return "127.0.0.1"
    finally:
        s.close()


def _load_model(result):
    warnings.filterwarnings("ignore")
    sys.path.insert(0, YOLOV5_ROOT)
    import torch
    m = torch.hub.load(YOLOV5_ROOT, "custom", path=WEIGHTS, source="local")
    m.conf = conf_state["value"]
    if torch.cuda.is_available():
        m.to("cuda")
        if HALF:
            m.model.half()
    result["model"] = m


def main():
    global model

    print("loading model...", flush=True)
    model_result = {}
    load_thread = threading.Thread(target=_load_model, args=(model_result,))
    load_thread.start()

    threading.Thread(target=grabber, daemon=True).start()
    print("warming up camera...", flush=True)
    deadline = time.monotonic() + 12
    while state["cap_count"] < 5 and time.monotonic() < deadline:
        time.sleep(0.2)
    if state["cap_count"] == 0:
        print("no frames from the camera - is it connected?", flush=True)
        stopping.set()
        return

    load_thread.join(timeout=90)
    if "model" not in model_result:
        print("model did not finish loading in time.", flush=True)
        stopping.set()
        return
    model = model_result["model"]
    print(f"model loaded: {list(model.names.values())}", flush=True)

    threading.Thread(target=inferer, daemon=True).start()
    threading.Thread(target=sampler, daemon=True).start()

    httpd = Server(("0.0.0.0", PORT), Handler)
    # Lead with the plain IP: .local (mDNS) often fails to resolve on
    # Windows/Android clients even when avahi-daemon is running fine here.
    print(f"open this on your phone/PC : http://{lan_ip()}:{PORT}/")
    print(f"(mDNS alt, may not resolve): http://{socket.gethostname()}.local:{PORT}/")
    print("streaming. drag conf, or set the seconds and press Record.",
          flush=True)
    print("ctrl-c to quit.", flush=True)
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\nstopping...", flush=True)
    finally:
        stopping.set()
        with rec_lock:
            if rec["active"]:
                finish_locked()
        cap = state.get("cap")
        if cap is not None:
            cap.release()
        httpd.server_close()
        print("camera released. bye.", flush=True)


if __name__ == "__main__":
    main()
