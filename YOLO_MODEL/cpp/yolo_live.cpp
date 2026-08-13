// C++/TensorRT port of ../yolo_live.py: same always-on camera + live
// detection + web dashboard + timed recording report, but running
// prototype.onnx through a TensorRT engine instead of prototype.pt through
// torch.hub. The web UI (PAGE below) and every JSON endpoint shape are
// kept byte-for-byte compatible with the Python version so the existing
// front-end JS and control/target_distance.cpp's /target poller work
// against either binary unmodified.
#include <arpa/inet.h>
#include <climits>
#include <csignal>
#include <cstring>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <ctime>
#include <deque>
#include <fstream>
#include <future>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/opencv.hpp>

#include "http_server.hpp"
#include "report.hpp"
#include "sysinfo.hpp"
#include "trt_engine.hpp"
#include "yaml_settings.hpp"

namespace {

// ---------------------------------------------------------------------------
// SETTINGS - edit these (mirrors yolo_live.py's SETTINGS block)
// ---------------------------------------------------------------------------
constexpr int DEFAULT_RECORD_SEC = 30;
constexpr double SAMPLE_INTERVAL_SEC = 0.5;
constexpr double LIVE_WINDOW_SEC = 40;

const std::string LOG_PREFIX = "yolo_log";

constexpr int PORT = 8002;  // same default as yolo_live.py, so
                             // control/target_distance.cpp needs no changes

constexpr double DEFAULT_CONF = 0.25;
constexpr float IOU_THRES = 0.45f;  // yolov5 AutoShape's default iou, not exposed in the UI either
// See trt_engine.hpp's YoloTrt ctor doc: FP16 measured mAP50 0.995 -> 0.659
// on a GTX 1660 SUPER in this project; unverified on Jetson - leave false.
constexpr bool FP16 = false;

constexpr int TARGET_CONFIRM_FRAMES = 5;

constexpr int SENSOR_ID = 0;
constexpr int WIDTH = 640, HEIGHT = 480, CAM_FPS = 30;
constexpr int WBMODE = 3;
// ---------------------------------------------------------------------------

std::atomic<bool> g_stopping{false};
HttpServer* g_server_for_signal = nullptr;

void handle_signal(int) {
    g_stopping.store(true);
    if (g_server_for_signal) g_server_for_signal->stop();
}

double monotonic_seconds() {
    using namespace std::chrono;
    static const auto t0 = steady_clock::now();
    return duration<double>(steady_clock::now() - t0).count();
}

template <typename T>
void push_capped(std::deque<T>& dq, T value, size_t maxlen) {
    dq.push_back(std::move(value));
    while (dq.size() > maxlen) dq.pop_front();
}

double round_to(double v, int decimals) {
    double scale = std::pow(10.0, decimals);
    return std::round(v * scale) / scale;
}

std::string basename_of(const std::string& path) {
    size_t pos = path.find_last_of('/');
    return pos == std::string::npos ? path : path.substr(pos + 1);
}

std::string strftime_now(const char* fmt) {
    std::time_t t = std::time(nullptr);
    std::tm tm_buf{};
    localtime_r(&t, &tm_buf);
    char buf[128];
    std::strftime(buf, sizeof(buf), fmt, &tm_buf);
    return buf;
}

// ---------------------------------------------------------------------------
// path resolution - repo_root/setting/cam_sets.yaml, repo_root/YOLO_MODEL/*
// ---------------------------------------------------------------------------

std::string exe_dir() {
    char buf[PATH_MAX];
    ssize_t len = readlink("/proc/self/exe", buf, sizeof(buf) - 1);
    if (len == -1) return ".";
    buf[len] = '\0';
    std::string full(buf);
    size_t slash = full.find_last_of('/');
    return slash == std::string::npos ? "." : full.substr(0, slash);
}

// Walks up from the executable's directory until it finds
// setting/cam_sets.yaml, so the binary works regardless of which build
// directory (cpp/build, cpp/, ...) it ends up in.
std::string find_repo_root() {
    std::string dir = exe_dir();
    for (int i = 0; i < 8; ++i) {
        struct stat st {};
        if (stat((dir + "/setting/cam_sets.yaml").c_str(), &st) == 0) return dir;
        size_t slash = dir.find_last_of('/');
        if (slash == std::string::npos || slash == 0) break;
        dir = dir.substr(0, slash);
    }
    throw std::runtime_error("could not locate setting/cam_sets.yaml above " + exe_dir());
}

std::vector<std::string> load_class_names(const std::string& path, int num_classes) {
    std::vector<std::string> names;
    std::ifstream f(path);
    std::string line;
    while (f && std::getline(f, line)) {
        while (!line.empty() && (line.back() == '\r' || line.back() == ' ')) line.pop_back();
        if (!line.empty()) names.push_back(line);
    }
    while (static_cast<int>(names.size()) < num_classes) {
        names.push_back("class_" + std::to_string(names.size()));
    }
    return names;
}

std::string py_list_repr(const std::vector<std::string>& names) {
    std::ostringstream s;
    s << "[";
    for (size_t i = 0; i < names.size(); ++i) {
        if (i) s << ", ";
        s << "'" << names[i] << "'";
    }
    s << "]";
    return s.str();
}

// ---------------------------------------------------------------------------
// white balance correction - matches yolo_live.py's apply_wb_correction()
// ---------------------------------------------------------------------------

void apply_wb_correction(cv::Mat& frame, const cv::Vec3f& gains_bgr) {
    if (gains_bgr[0] == 1.0f && gains_bgr[1] == 1.0f && gains_bgr[2] == 1.0f) return;
    for (int y = 0; y < frame.rows; ++y) {
        uint8_t* row = frame.ptr<uint8_t>(y);
        for (int x = 0; x < frame.cols; ++x) {
            for (int c = 0; c < 3; ++c) {
                if (gains_bgr[c] == 1.0f) continue;
                row[x * 3 + c] = cv::saturate_cast<uint8_t>(row[x * 3 + c] * gains_bgr[c]);
            }
        }
    }
}

// ---------------------------------------------------------------------------
// shared state - mirrors yolo_live.py's module-level state/history/rec dicts
// ---------------------------------------------------------------------------

struct DetSummary {
    std::string name;
    double conf;
};

struct TargetInfo {
    bool found = false;
    bool confirmed = false;
    double t = 0.0;
    std::string name;
    double conf = 0.0, x_px = 0.0, y_px = 0.0;
    int detections = 0;
    bool multi = false;
};

struct SharedState {
    std::mutex cap_mutex;
    std::condition_variable cap_cv;
    cv::Mat raw_frame;
    uint64_t raw_frame_gen = 0;
    std::deque<double> cap_times;
    uint64_t cap_count = 0;

    std::mutex frame_mutex;
    std::condition_variable frame_cv;
    std::vector<uint8_t> jpg_frame;
    uint64_t frame_gen = 0;
    std::deque<double> infer_times;
    std::vector<DetSummary> last_dets;
    double last_latency_ms = 0.0;
    TargetInfo target;
};

struct ConfState {
    std::mutex mutex;
    double value = DEFAULT_CONF;
};

struct HistPoint {
    double t;
    SampleRow row;
};

struct RecState {
    std::mutex mutex;
    bool active = false;
    std::vector<SampleRow> rows;
    double t0 = 0.0;
    double duration = 0.0;
    std::string saved;
    std::string started_str;
};

struct AppPaths {
    std::string log_dir;
    std::string weights_name;
};

// finish_locked(): caller must hold rec.mutex. Mirrors yolo_live.py's
// finish_locked().
void finish_locked(RecState& rec, const AppPaths& paths, const std::vector<std::string>& class_names, int img_size) {
    if (rec.rows.empty()) {
        rec.active = false;
        return;
    }
    std::vector<SampleRow> rows = rec.rows;
    std::string started_str = rec.started_str;
    double duration = rec.duration;
    rec.active = false;

    std::string name = LOG_PREFIX + "_" + strftime_now("%m%d_%H%M") + ".txt";
    mkdir(paths.log_dir.c_str(), 0755);  // fine if it already exists
    std::string path = paths.log_dir + "/" + name;
    std::string text = build_report(rows, started_str, duration, paths.weights_name, class_names, img_size);

    std::ofstream f(path);
    if (f) f << text << "\n";
    rec.saved = name;

    std::cout << "\n" << text << std::endl;
    std::cout << "saved: " << path << "\n" << std::endl;
}

SampleRow build_sample(SharedState& state, ConfState& conf_state, SysInfoReader& sysinfo_reader) {
    std::vector<double> cap_times;
    std::vector<double> infer_times;
    std::vector<DetSummary> dets;
    double latency_ms;

    {
        std::lock_guard<std::mutex> lock(state.cap_mutex);
        cap_times.assign(state.cap_times.begin(), state.cap_times.end());
    }
    {
        std::lock_guard<std::mutex> lock(state.frame_mutex);
        infer_times.assign(state.infer_times.begin(), state.infer_times.end());
        dets = state.last_dets;
        latency_ms = state.last_latency_ms;
    }

    auto fps_of = [](const std::vector<double>& times) -> double {
        if (times.size() < 2) return 0.0;
        double span = times.back() - times.front();
        return span > 0 ? round_to((times.size() - 1) / span, 2) : 0.0;
    };

    SampleRow row;
    row.cap_fps = fps_of(cap_times);
    row.infer_fps = fps_of(infer_times);
    row.latency_ms = latency_ms;
    row.det_count = static_cast<int>(dets.size());
    if (!dets.empty()) {
        double sum = 0.0, top = 0.0;
        for (auto& d : dets) {
            sum += d.conf;
            top = std::max(top, d.conf);
        }
        row.avg_conf = round_to(sum / dets.size(), 3);
        row.top_conf = round_to(top, 3);
    }
    {
        std::lock_guard<std::mutex> lock(conf_state.mutex);
        row.conf_threshold = conf_state.value;
    }
    row.sys = sysinfo_reader.sample();
    return row;
}

// ---------------------------------------------------------------------------
// rendering - draws boxes+labels the way yolov5's results.render() would
// ---------------------------------------------------------------------------

void draw_detections(cv::Mat& img, const std::vector<Detection>& dets, const std::vector<std::string>& class_names) {
    static const cv::Scalar palette[] = {{56, 56, 255}, {255, 159, 56}, {56, 255, 159},
                                          {255, 56, 204}, {159, 56, 255}, {56, 204, 255}};
    for (const auto& d : dets) {
        cv::Scalar color = palette[d.cls % 6];
        cv::Point p1(static_cast<int>(d.x1), static_cast<int>(d.y1));
        cv::Point p2(static_cast<int>(d.x2), static_cast<int>(d.y2));
        cv::rectangle(img, p1, p2, color, 2);

        std::string cls_name =
            (d.cls >= 0 && d.cls < static_cast<int>(class_names.size())) ? class_names[d.cls] : "class_" + std::to_string(d.cls);
        std::ostringstream label_s;
        label_s << cls_name << " " << std::fixed << std::setprecision(2) << d.conf;
        std::string label = label_s.str();

        int baseline = 0;
        cv::Size ts = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);
        int label_top = std::max(p1.y - ts.height - 4, 0);
        cv::rectangle(img, cv::Point(p1.x, label_top), cv::Point(p1.x + ts.width + 2, p1.y), color, cv::FILLED);
        cv::putText(img, label, cv::Point(p1.x + 1, p1.y - 3), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255),
                    1, cv::LINE_AA);
    }
}

// ---------------------------------------------------------------------------
// threads - grabber() / inferer() / sampler(), mirrors yolo_live.py
// ---------------------------------------------------------------------------

void grabber(SharedState& state, cv::Vec3f wb_gains_bgr) {
    std::ostringstream p;
    p << "nvarguscamerasrc sensor-id=" << SENSOR_ID << " wbmode=" << WBMODE << " ! "
      << "video/x-raw(memory:NVMM),width=" << WIDTH << ",height=" << HEIGHT << ",format=NV12,framerate=" << CAM_FPS
      << "/1 ! "
      << "nvvidconv flip-method=0 ! "
      << "video/x-raw,width=" << WIDTH << ",height=" << HEIGHT << ",format=BGRx ! "
      << "videoconvert ! video/x-raw,format=BGR ! appsink drop=1";
    std::string pipeline = p.str();

    while (!g_stopping.load()) {
        cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);
        if (!cap.isOpened()) {
            std::this_thread::sleep_for(std::chrono::seconds(3));
            continue;
        }
        while (!g_stopping.load()) {
            cv::Mat frame;
            if (!cap.read(frame)) break;
            apply_wb_correction(frame, wb_gains_bgr);
            double now = monotonic_seconds();
            {
                std::lock_guard<std::mutex> lock(state.cap_mutex);
                state.raw_frame = frame;
                state.raw_frame_gen++;
                push_capped(state.cap_times, now, static_cast<size_t>(30));
                state.cap_count++;
            }
            state.cap_cv.notify_all();
        }
        cap.release();
        if (g_stopping.load()) return;
        std::this_thread::sleep_for(std::chrono::seconds(3));
    }
}

void inferer(SharedState& state, YoloTrt& model, ConfState& conf_state, const std::vector<std::string>& class_names) {
    uint64_t last_gen = 0;
    std::deque<std::optional<std::string>> streak;  // None-slot for "no/unstable detection this frame"

    while (!g_stopping.load()) {
        cv::Mat frame;
        {
            std::unique_lock<std::mutex> lock(state.cap_mutex);
            state.cap_cv.wait(lock, [&] { return state.raw_frame_gen != last_gen || g_stopping.load(); });
            if (g_stopping.load()) return;
            frame = state.raw_frame;
            last_gen = state.raw_frame_gen;
        }

        double conf_thres;
        {
            std::lock_guard<std::mutex> lock(conf_state.mutex);
            conf_thres = conf_state.value;
        }

        double t0 = monotonic_seconds();
        std::vector<Detection> dets = model.infer(frame, static_cast<float>(conf_thres), IOU_THRES);
        double latency_ms = (monotonic_seconds() - t0) * 1000.0;

        cv::Mat annotated = frame.clone();
        draw_detections(annotated, dets, class_names);
        std::vector<uint8_t> jpg;
        cv::imencode(".jpg", annotated, jpg, {cv::IMWRITE_JPEG_QUALITY, 85});

        std::vector<DetSummary> det_summaries;
        det_summaries.reserve(dets.size());
        for (auto& d : dets) {
            std::string name =
                (d.cls >= 0 && d.cls < static_cast<int>(class_names.size())) ? class_names[d.cls] : "class_" + std::to_string(d.cls);
            det_summaries.push_back({name, round_to(d.conf, 3)});
        }

        bool multi = dets.size() > 1;
        std::optional<std::string> top_name;
        TargetInfo target;
        target.detections = static_cast<int>(dets.size());
        target.multi = multi;
        if (!dets.empty()) {
            const Detection* top = &dets[0];
            for (auto& d : dets) {
                if (d.conf > top->conf) top = &d;
            }
            std::string name =
                (top->cls >= 0 && top->cls < static_cast<int>(class_names.size())) ? class_names[top->cls]
                                                                                    : "class_" + std::to_string(top->cls);
            top_name = name;
            target.found = true;
            target.name = name;
            target.conf = round_to(top->conf, 3);
            // ../setting/cam_sets.yaml's coord_origin: center frame, +x
            // right, +y up - consumed by control/target_distance.cpp.
            double px = (top->x1 + top->x2) / 2.0;
            double py = (top->y1 + top->y2) / 2.0;
            target.x_px = round_to(px - WIDTH / 2.0, 1);
            target.y_px = round_to(HEIGHT / 2.0 - py, 1);
        }

        double now = monotonic_seconds();
        {
            std::lock_guard<std::mutex> lock(state.frame_mutex);
            state.jpg_frame.assign(jpg.begin(), jpg.end());
            state.frame_gen++;
            push_capped(state.infer_times, now, static_cast<size_t>(30));
            state.last_dets = det_summaries;
            state.last_latency_ms = round_to(latency_ms, 1);

            push_capped(streak, multi ? std::optional<std::string>() : top_name,
                        static_cast<size_t>(TARGET_CONFIRM_FRAMES));
            bool confirmed = !multi && top_name.has_value() && streak.size() == TARGET_CONFIRM_FRAMES &&
                              std::all_of(streak.begin(), streak.end(),
                                          [&](const std::optional<std::string>& n) { return n == top_name; });
            target.confirmed = confirmed;
            target.t = now;
            state.target = target;
        }
        state.frame_cv.notify_all();
    }
}

void sampler(SharedState& state, ConfState& conf_state, SysInfoReader& sysinfo_reader, std::deque<HistPoint>& history,
             std::mutex& hist_mutex, size_t hist_maxlen, RecState& rec, const AppPaths& paths,
             const std::vector<std::string>& class_names, int img_size) {
    while (!g_stopping.load()) {
        SampleRow row = build_sample(state, conf_state, sysinfo_reader);
        double now = monotonic_seconds();
        {
            std::lock_guard<std::mutex> lock(hist_mutex);
            push_capped(history, HistPoint{now, row}, hist_maxlen);
        }
        {
            std::lock_guard<std::mutex> lock(rec.mutex);
            if (rec.active) {
                double elapsed = now - rec.t0;
                SampleRow r = row;
                r.elapsed_s = round_to(elapsed, 2);
                rec.rows.push_back(r);
                if (elapsed >= rec.duration) finish_locked(rec, paths, class_names, img_size);
            }
        }
        std::this_thread::sleep_for(std::chrono::duration<double>(SAMPLE_INTERVAL_SEC));
    }
}

// ---------------------------------------------------------------------------
// JSON helpers - hand-rolled since this Jetson image has no jsoncpp-dev
// headers (only the runtime .so); mirrors the manual approach
// control/target_distance.cpp already uses on the consuming side.
// ---------------------------------------------------------------------------

std::string json_num(double v) {
    if (std::isnan(v) || std::isinf(v)) return "null";
    std::ostringstream s;
    s << std::setprecision(10) << v;
    return s.str();
}

std::string json_bool(bool v) { return v ? "true" : "false"; }

std::string json_escape(const std::string& s) {
    std::string out;
    out.reserve(s.size());
    for (char c : s) {
        if (c == '"' || c == '\\') out.push_back('\\');
        out.push_back(c);
    }
    return out;
}

std::string json_str(const std::string& s) { return "\"" + json_escape(s) + "\""; }

std::string row_fields_json(const SampleRow& r) {
    std::ostringstream s;
    s << "\"cap_fps\":" << json_num(r.cap_fps) << ",\"infer_fps\":" << json_num(r.infer_fps)
      << ",\"latency_ms\":" << json_num(r.latency_ms) << ",\"det_count\":" << r.det_count
      << ",\"avg_conf\":" << json_num(r.avg_conf) << ",\"top_conf\":" << json_num(r.top_conf)
      << ",\"conf_threshold\":" << json_num(r.conf_threshold);
    if (r.sys.cpu_pct) s << ",\"cpu_pct\":" << json_num(*r.sys.cpu_pct);
    if (r.sys.mem_pct) s << ",\"mem_pct\":" << json_num(*r.sys.mem_pct);
    if (r.sys.mem_used_mb) s << ",\"mem_used_mb\":" << json_num(*r.sys.mem_used_mb);
    if (r.sys.mem_total_mb) s << ",\"mem_total_mb\":" << json_num(*r.sys.mem_total_mb);
    if (r.sys.clock_mhz) s << ",\"clock_mhz\":" << json_num(*r.sys.clock_mhz);
    if (r.sys.temp_c) s << ",\"temp_c\":" << json_num(*r.sys.temp_c);
    return s.str();
}

// ---------------------------------------------------------------------------
// web page - identical HTML/CSS/JS to yolo_live.py's PAGE, so the front-end
// needs no changes to talk to this binary instead of the Python one.
// ---------------------------------------------------------------------------

const char* PAGE = R"HTMLPAGE(<!doctype html><meta charset=utf-8>
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
)HTMLPAGE";

std::string render_page(double conf_value) {
    std::string page = PAGE;
    auto replace_all = [](std::string& s, const std::string& from, const std::string& to) {
        size_t pos = 0;
        while ((pos = s.find(from, pos)) != std::string::npos) {
            s.replace(pos, from.size(), to);
            pos += to.size();
        }
    };
    std::ostringstream conf_s;
    conf_s << conf_value;
    replace_all(page, "__DEF__", std::to_string(DEFAULT_RECORD_SEC));
    replace_all(page, "__CONF__", conf_s.str());
    std::ostringstream win_s;
    win_s << LIVE_WINDOW_SEC;
    replace_all(page, "__WIN__", win_s.str());
    return page;
}

// ---------------------------------------------------------------------------
// networking helpers - lan_ip()/hostname(), matches yolo_live.py's startup banner
// ---------------------------------------------------------------------------

std::string lan_ip() {
    int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) return "127.0.0.1";
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(80);
    inet_pton(AF_INET, "8.8.8.8", &addr.sin_addr);
    std::string ip = "127.0.0.1";
    if (::connect(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) == 0) {
        sockaddr_in local{};
        socklen_t len = sizeof(local);
        if (getsockname(fd, reinterpret_cast<sockaddr*>(&local), &len) == 0) {
            char buf[INET_ADDRSTRLEN];
            if (inet_ntop(AF_INET, &local.sin_addr, buf, sizeof(buf))) ip = buf;
        }
    }
    ::close(fd);
    return ip;
}

std::string hostname() {
    char buf[256];
    if (gethostname(buf, sizeof(buf)) == 0) return buf;
    return "localhost";
}

}  // namespace

int run() {
    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);

    std::string repo_root = find_repo_root();
    std::string setting_path = repo_root + "/setting/cam_sets.yaml";
    std::string yolo_dir = repo_root + "/YOLO_MODEL";
    std::string onnx_path = yolo_dir + "/0812best.onnx";
    std::string engine_path = yolo_dir + "/0812best.engine";
    std::string classes_path = yolo_dir + "/classes.txt";

    AppPaths paths;
    paths.log_dir = yolo_dir + "/log";
    paths.weights_name = basename_of(onnx_path);

    YamlValue cam_settings = parse_yaml_file(setting_path);
    const YamlValue& wb = cam_settings["wb_correction"];
    cv::Vec3f wb_gains_bgr(static_cast<float>(wb["blue_gain"].as_double()), static_cast<float>(wb["green_gain"].as_double()),
                           static_cast<float>(wb["red_gain"].as_double()));

    SharedState state;
    ConfState conf_state;
    RecState rec;
    std::deque<HistPoint> history;
    std::mutex hist_mutex;
    size_t hist_maxlen = static_cast<size_t>(LIVE_WINDOW_SEC / SAMPLE_INTERVAL_SEC) + 2;
    SysInfoReader sysinfo_reader;

    std::cout << "loading model..." << std::endl;
    std::promise<std::unique_ptr<YoloTrt>> model_promise;
    std::future<std::unique_ptr<YoloTrt>> model_future = model_promise.get_future();
    std::thread load_thread([&] {
        try {
            model_promise.set_value(std::make_unique<YoloTrt>(onnx_path, engine_path, FP16));
        } catch (...) {
            model_promise.set_exception(std::current_exception());
        }
    });

    std::thread grab_thread(grabber, std::ref(state), wb_gains_bgr);

    std::cout << "warming up camera..." << std::endl;
    auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(12);
    while (true) {
        {
            std::lock_guard<std::mutex> lock(state.cap_mutex);
            if (state.cap_count >= 5) break;
        }
        if (std::chrono::steady_clock::now() >= deadline) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    uint64_t cap_count_now = 0;
    {
        std::lock_guard<std::mutex> lock(state.cap_mutex);
        cap_count_now = state.cap_count;
    }
    if (cap_count_now == 0) {
        std::cout << "no frames from the camera - is it connected?" << std::endl;
        g_stopping.store(true);
        state.cap_cv.notify_all();
        grab_thread.join();
        load_thread.join();
        return 1;
    }

    if (model_future.wait_for(std::chrono::seconds(90)) != std::future_status::ready) {
        std::cout << "model did not finish loading in time." << std::endl;
        g_stopping.store(true);
        state.cap_cv.notify_all();
        grab_thread.join();
        load_thread.join();
        return 1;
    }
    std::unique_ptr<YoloTrt> model = model_future.get();
    load_thread.join();

    std::vector<std::string> class_names = load_class_names(classes_path, model->num_classes());
    int img_size = model->input_w();
    std::cout << "model loaded: " << py_list_repr(class_names) << std::endl;

    std::thread infer_thread(inferer, std::ref(state), std::ref(*model), std::ref(conf_state), std::cref(class_names));
    std::thread sampler_thread(sampler, std::ref(state), std::ref(conf_state), std::ref(sysinfo_reader),
                                std::ref(history), std::ref(hist_mutex), hist_maxlen, std::ref(rec), std::cref(paths),
                                std::cref(class_names), img_size);

    HttpServer server(PORT);
    g_server_for_signal = &server;

    server.add_route("/", [&](const HttpRequest&, HttpConnection& conn) {
        double conf_value;
        {
            std::lock_guard<std::mutex> lock(conf_state.mutex);
            conf_value = conf_state.value;
        }
        conn.send(200, "text/html; charset=utf-8", render_page(conf_value));
    });

    server.add_route("/conf", [&](const HttpRequest& req, HttpConnection& conn) {
        std::string val_str;
        {
            std::ostringstream def;
            def << DEFAULT_CONF;
            val_str = def.str();
        }
        auto it = req.query.find("value");
        if (it != req.query.end()) val_str = it->second;
        try {
            double v = std::stod(val_str);
            v = std::clamp(v, 0.0, 1.0);
            std::lock_guard<std::mutex> lock(conf_state.mutex);
            conf_state.value = v;
        } catch (...) {
            // matches yolo_live.py: an unparsable value leaves conf_state unchanged
        }
        double current;
        {
            std::lock_guard<std::mutex> lock(conf_state.mutex);
            current = conf_state.value;
        }
        conn.send(200, "application/json", "{\"ok\":true,\"value\":" + json_num(current) + "}");
    });

    server.add_route("/start", [&](const HttpRequest& req, HttpConnection& conn) {
        std::string sec_str = "30";
        auto it = req.query.find("sec");
        if (it != req.query.end()) sec_str = it->second;
        double sec;
        try {
            sec = std::clamp(std::stod(sec_str), 1.0, 3600.0);
        } catch (...) {
            sec = static_cast<double>(DEFAULT_RECORD_SEC);
        }
        {
            std::lock_guard<std::mutex> lock(rec.mutex);
            rec.active = true;
            rec.rows.clear();
            rec.t0 = monotonic_seconds();
            rec.duration = sec;
            rec.saved.clear();
            rec.started_str = strftime_now("%Y-%m-%d %H:%M:%S");
        }
        std::cout << "recording " << std::fixed << std::setprecision(0) << sec << "s..." << std::endl;
        conn.send(200, "application/json", "{\"ok\":true,\"duration\":" + json_num(sec) + "}");
    });

    server.add_route("/stop", [&](const HttpRequest&, HttpConnection& conn) {
        {
            std::lock_guard<std::mutex> lock(rec.mutex);
            if (rec.active) finish_locked(rec, paths, class_names, img_size);
        }
        conn.send(200, "application/json", "{\"ok\":true}");
    });

    server.add_route("/data", [&](const HttpRequest&, HttpConnection& conn) {
        bool active;
        std::vector<SampleRow> rows;
        double duration, t0;
        std::string saved;
        {
            std::lock_guard<std::mutex> lock(rec.mutex);
            active = rec.active;
            rows = rec.rows;
            duration = rec.duration;
            saved = rec.saved;
            t0 = rec.t0;
        }

        std::ostringstream points;
        double xmax, elapsed;
        bool have_cur = false;
        SampleRow cur;

        if (active && !rows.empty()) {
            for (size_t i = 0; i < rows.size(); ++i) {
                if (i) points << ",";
                points << "{\"t\":" << json_num(rows[i].elapsed_s) << ",\"infer_fps\":" << json_num(rows[i].infer_fps)
                       << "}";
            }
            xmax = duration;
            elapsed = monotonic_seconds() - t0;
            cur = rows.back();
            have_cur = true;
        } else {
            std::vector<HistPoint> hist;
            {
                std::lock_guard<std::mutex> lock(hist_mutex);
                hist.assign(history.begin(), history.end());
            }
            double base = hist.empty() ? 0.0 : hist.front().t;
            for (size_t i = 0; i < hist.size(); ++i) {
                if (i) points << ",";
                points << "{\"t\":" << json_num(round_to(hist[i].t - base, 2))
                       << ",\"infer_fps\":" << json_num(hist[i].row.infer_fps) << "}";
            }
            xmax = LIVE_WINDOW_SEC;
            elapsed = 0.0;
            if (!hist.empty()) {
                cur = hist.back().row;
                have_cur = true;
            }
        }

        std::ostringstream body;
        body << "{\"points\":[" << points.str() << "],\"current\":{" << (have_cur ? row_fields_json(cur) : "")
             << "},\"width\":" << WIDTH << ",\"height\":" << HEIGHT << ",\"recording\":" << json_bool(active)
             << ",\"elapsed\":" << json_num(round_to(elapsed, 1)) << ",\"duration\":" << json_num(duration)
             << ",\"xmax\":" << json_num(xmax) << ",\"saved\":" << json_str(saved) << "}";
        conn.send(200, "application/json", body.str());
    });

    server.add_route("/target", [&](const HttpRequest&, HttpConnection& conn) {
        TargetInfo t;
        {
            std::lock_guard<std::mutex> lock(state.frame_mutex);
            t = state.target;
        }
        double now = monotonic_seconds();
        double age_ms = t.t != 0.0 ? round_to((now - t.t) * 1000.0, 1) : -1.0;

        std::ostringstream body;
        body << "{\"found\":" << json_bool(t.found) << ",\"confirmed\":" << json_bool(t.confirmed);
        if (t.found) {
            body << ",\"name\":" << json_str(t.name) << ",\"conf\":" << json_num(t.conf)
                 << ",\"x_px\":" << json_num(t.x_px) << ",\"y_px\":" << json_num(t.y_px);
        }
        body << ",\"detections\":" << t.detections << ",\"multi\":" << json_bool(t.multi) << ",\"t\":" << json_num(t.t)
             << ",\"age_ms\":" << (t.t != 0.0 ? json_num(age_ms) : "null") << ",\"width\":" << WIDTH
             << ",\"height\":" << HEIGHT << "}";
        conn.send(200, "application/json", body.str());
    });

    server.add_route("/stream", [&](const HttpRequest&, HttpConnection& conn) {
        if (!conn.send_stream_headers("multipart/x-mixed-replace; boundary=FRAME")) return;
        uint64_t last_gen = 0;
        while (!g_stopping.load()) {
            std::vector<uint8_t> jpg;
            {
                std::unique_lock<std::mutex> lock(state.frame_mutex);
                state.frame_cv.wait_for(lock, std::chrono::seconds(1),
                                         [&] { return state.frame_gen != last_gen || g_stopping.load(); });
                if (g_stopping.load()) break;
                if (state.frame_gen == last_gen) continue;
                jpg = state.jpg_frame;
                last_gen = state.frame_gen;
            }
            std::string header =
                "--FRAME\r\nContent-Type: image/jpeg\r\nContent-Length: " + std::to_string(jpg.size()) + "\r\n\r\n";
            if (!conn.write_raw(header)) break;
            if (!conn.write_raw(std::string(jpg.begin(), jpg.end()))) break;
            if (!conn.write_raw("\r\n")) break;
        }
    });

    std::cout << "open this on your phone/PC : http://" << lan_ip() << ":" << PORT << "/" << std::endl;
    std::cout << "(mDNS alt, may not resolve): http://" << hostname() << ".local:" << PORT << "/" << std::endl;
    std::cout << "streaming. drag conf, or set the seconds and press Record." << std::endl;
    std::cout << "ctrl-c to quit." << std::endl;

    server.serve_forever();

    std::cout << "\nstopping..." << std::endl;
    g_stopping.store(true);
    state.cap_cv.notify_all();
    state.frame_cv.notify_all();
    {
        std::lock_guard<std::mutex> lock(rec.mutex);
        if (rec.active) finish_locked(rec, paths, class_names, img_size);
    }
    grab_thread.join();
    infer_thread.join();
    sampler_thread.join();
    std::cout << "camera released. bye." << std::endl;
    return 0;
}

int main() {
    try {
        return run();
    } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return 1;
    }
}
