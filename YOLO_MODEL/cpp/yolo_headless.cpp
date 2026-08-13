// Headless twin of yolo_live.cpp for real flights: same camera capture +
// TensorRT inference + /target JSON endpoint control/target_distance.cpp
// polls, AND the same /stream MJPEG endpoint yolo_live.cpp has - "headless"
// here means no dashboard page/recording-slider UI, NOT no video. The
// laptop-side GCS dashboard (gcs/tools/gcs_bridge.py, dashboard.html) is
// meant to hit this binary's /stream directly over the LAN (see gcs/PLAN.md:
// "화면을 그리는 것은 노트북에서 담당하고, jetson은 json형태의 파일로만 줌" -
// the Jetson doesn't need to host a browsable page for a human sitting at
// it, but the frame bytes themselves still have to exist somewhere a remote
// viewer can pull them from). yolo_live.cpp stays the tool for a human
// sitting at the Jetson itself to open in a browser and manually tune
// (conf slider, recording reports); this binary is what full_mission.sh
// runs during an actual flight.
//
// Every other behavior (settings, /target response shape, /conf, /start,
// /stop, /data, TARGET_CONFIRM_FRAMES stability gate) is unchanged from
// yolo_live.cpp on purpose, so target_distance.cpp and anything else that
// already talks to yolo_live's HTTP API works against this binary
// unmodified - only difference is what's NOT running (the "/" dashboard
// page and its interactive controls).
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
// SETTINGS - mirrors yolo_live.cpp's SETTINGS block; keep these two in sync.
// ---------------------------------------------------------------------------
constexpr int DEFAULT_RECORD_SEC = 30;
constexpr double SAMPLE_INTERVAL_SEC = 0.5;
constexpr double LIVE_WINDOW_SEC = 40;

const std::string LOG_PREFIX = "yolo_log";

// PORT falls back to this if setting/port.yaml has no "yolo_live" key -
// control/target_distance.cpp doesn't care which binary is actually
// serving it, just that it's this port.
int PORT = 8002;

constexpr double DEFAULT_CONF = 0.25;
constexpr float IOU_THRES = 0.45f;
constexpr bool FP16 = false;

constexpr int TARGET_CONFIRM_FRAMES = 5;

// Camera capture settings - fall back to these if setting/cam_sets.yaml's
// "yolo_live" section is missing a key. Overwritten from that section at
// the top of run(), before grabber()/HttpServer read them.
int SENSOR_ID = 0;
int WIDTH = 640, HEIGHT = 480, CAM_FPS = 30;
int WBMODE = 3;
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
// shared state - same shape as yolo_live.cpp's, including the JPEG frame
// fields /stream needs.
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

    std::mutex frame_mutex;  // guards everything below, including jpg_frame
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

// finish_locked(): caller must hold rec.mutex. Mirrors yolo_live.cpp's.
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
// rendering - draws boxes+labels the way yolov5's results.render() would;
// same as yolo_live.cpp's, needed here too since /stream is still served.
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
// threads - grabber() / inferer() / sampler(), mirrors yolo_live.cpp.
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

// Same detection/confirm logic as yolo_live.cpp's inferer(), including the
// draw_detections()+cv::imencode() annotate/encode step - /stream still
// needs a fresh JPEG each cycle here, same as yolo_live.cpp.
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
// JSON helpers - same hand-rolled approach as yolo_live.cpp (no jsoncpp-dev
// headers on this Jetson image, only the runtime .so).
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

    const YamlValue& cam_cfg = cam_settings["yolo_live"];
    SENSOR_ID = static_cast<int>(cam_cfg.get_long_or("sensor_id", SENSOR_ID));
    WIDTH = static_cast<int>(cam_cfg.get_long_or("width", WIDTH));
    HEIGHT = static_cast<int>(cam_cfg.get_long_or("height", HEIGHT));
    CAM_FPS = static_cast<int>(cam_cfg.get_long_or("fps", CAM_FPS));
    WBMODE = static_cast<int>(cam_cfg.get_long_or("wbmode", WBMODE));
    // load_port_settings() assumes an executable under control/ or
    // control/build/ (see control/yaml_settings.cpp) - this binary lives
    // one directory deeper (YOLO_MODEL/cpp/build/), so read port.yaml via
    // the repo_root this file already locates for cam_sets.yaml instead.
    YamlValue port_settings = parse_yaml_file(repo_root + "/setting/port.yaml");
    PORT = static_cast<int>(port_settings.get_long_or("yolo_live", PORT));

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

    // No "/" dashboard page here - see the top-of-file comment. /stream is
    // still served (for a remote GCS dashboard to pull directly), just
    // without the browsable page wrapping it. /conf,/start,/stop,/data are
    // kept because they're handy for headless debugging over curl (e.g.
    // checking infer fps/conf without a browser) and cost nothing extra
    // when nobody calls them.
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
            // matches yolo_live.cpp: an unparsable value leaves conf_state unchanged
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

    std::cout << "headless - no dashboard page. /target on http://127.0.0.1:" << PORT
              << "/target, /stream on http://127.0.0.1:" << PORT << "/stream" << std::endl;
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
