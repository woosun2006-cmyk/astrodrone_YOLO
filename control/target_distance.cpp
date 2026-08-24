// Stage 1 of the target-approach pipeline ("1.표적 관측"): reads the
// YOLO pixel observation and publishes it over loopback UDP for
// FlightMissionApp to combine with Adapter altitude telemetry before
// guidance (stage 2/3: velocity calculation and vehicle command).
//
// Reads YOLO's /target response, preserves the calibrated pixel observation,
// and publishes it over loopback UDP. Vehicle altitude and telemetry
// freshness belong to FlightMissionApp/SafetyMonitor, not this process.
#include <algorithm>
#include <arpa/inet.h>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <netinet/in.h>
#include <stdexcept>
#include <string>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>

#include "yaml_settings.hpp"
#include "pos_calculator.hpp"
#include "target_json.hpp"
#include "target_link.hpp"

namespace {

// Minimal blocking HTTP/1.0 GET over loopback TCP - just enough to read
// yolo_live.py's small flat JSON /target response. No keep-alive, no
// chunked transfer: yolo_live.py's Handler sends Content-Length and closes
// the connection.
std::string http_get(const std::string& host, int port, const std::string& path,
                      double timeout_sec) {
    int fd = ::socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) throw std::runtime_error("create HTTP socket failed");

    timeval tv{};
    tv.tv_sec = static_cast<long>(timeout_sec);
    tv.tv_usec = static_cast<long>((timeout_sec - static_cast<double>(tv.tv_sec)) * 1'000'000);
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<uint16_t>(port));
    if (inet_pton(AF_INET, host.c_str(), &addr.sin_addr) != 1) {
        ::close(fd);
        throw std::runtime_error("invalid YOLO host: " + host);
    }

    if (::connect(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
        ::close(fd);
        throw std::runtime_error("connect to " + host + ":" + std::to_string(port) + " failed");
    }

    std::string request =
        "GET " + path + " HTTP/1.0\r\nHost: " + host + "\r\nConnection: close\r\n\r\n";
    if (::send(fd, request.data(), request.size(), 0) < 0) {
        ::close(fd);
        throw std::runtime_error("send HTTP request failed");
    }

    std::string response;
    char buf[2048];
    ssize_t n;
    while ((n = ::recv(fd, buf, sizeof(buf), 0)) > 0) {
        response.append(buf, static_cast<size_t>(n));
    }
    ::close(fd);

    size_t body_start = response.find("\r\n\r\n");
    if (body_start == std::string::npos) throw std::runtime_error("malformed HTTP response");
    return response.substr(body_start + 4);
}

struct Args {
    std::string yolo_host = "127.0.0.1";
    int yolo_port = 8002;
    int udp_port = 15020;
    int gcs_udp_port = 15022;
    double focal_length_px = 530;
    double focal_x_px = 530;
    double focal_y_px = 530;
    double principal_x_px = 320;
    double principal_y_px = 240;
    double frame_width_px = 640;
    double frame_height_px = 480;
    double target_stale_ms = 400;
    // Paced to setting/rate.yaml's sense_cycle_ms, not control.cpp's
    // control_cycle_ms (MAVLink.yaml) - this loop's natural rate is how
    // often YOLO_MODEL/yolo_live.py is expected to have a fresh detection
    // on hand, not the Pixhawk send rate.
    double sense_cycle_ms = 100;
};

}  // namespace

int run(int argc, char** argv) {
    YamlValue settings = load_mavlink_settings();
    YamlValue track = settings["target_track"];

    Args args;
    args.yolo_host = track["yolo_host"].as_string();
    args.yolo_port = static_cast<int>(track.get_long_or("yolo_port", 8002));
    args.udp_port = static_cast<int>(track.get_long_or("udp_port", 15020));
    args.gcs_udp_port = static_cast<int>(track.get_long_or("gcs_udp_port", 15022));
    args.focal_length_px = track.get_double_or("pixel_focal_length_px", 530);
    args.focal_x_px = track.get_double_or("pixel_focal_length_x_px", args.focal_length_px);
    args.focal_y_px = track.get_double_or("pixel_focal_length_y_px", args.focal_length_px);
    args.principal_x_px = track.get_double_or("pixel_principal_point_x_px", 320);
    args.principal_y_px = track.get_double_or("pixel_principal_point_y_px", 240);
    args.target_stale_ms = track.get_double_or("target_stale_ms", 400);
    // Shared with YOLO_MODEL/yolo_live.py's infer_max_fps - see
    // setting/rate.yaml for why the two live together.
    args.sense_cycle_ms = load_rate_settings().get_double_or("sense_cycle_ms", 100);

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        auto next = [&](const char* name) -> std::string {
            if (i + 1 >= argc) throw std::runtime_error(std::string(name) + " requires a value");
            return argv[++i];
        };
        if (arg == "--yolo-host") {
            args.yolo_host = next("--yolo-host");
        } else if (arg == "--yolo-port") {
            args.yolo_port = std::stoi(next("--yolo-port"));
        } else if (arg == "--udp-port") {
            args.udp_port = std::stoi(next("--udp-port"));
        } else if (arg == "--gcs-udp-port") {
            args.gcs_udp_port = std::stoi(next("--gcs-udp-port"));
        } else if (arg == "--focal-length-px") {
            args.focal_length_px = std::stod(next("--focal-length-px"));
            args.focal_x_px = args.focal_length_px;
            args.focal_y_px = args.focal_length_px;
        } else if (arg == "--focal-x-px") {
            args.focal_x_px = std::stod(next("--focal-x-px"));
        } else if (arg == "--focal-y-px") {
            args.focal_y_px = std::stod(next("--focal-y-px"));
        } else if (arg == "--principal-x-px") {
            args.principal_x_px = std::stod(next("--principal-x-px"));
        } else if (arg == "--principal-y-px") {
            args.principal_y_px = std::stod(next("--principal-y-px"));
        } else if (arg == "--sense-cycle-ms") {
            args.sense_cycle_ms = std::stod(next("--sense-cycle-ms"));
        } else {
            throw std::runtime_error("알 수 없는 target-distance 옵션: " + arg);
        }
    }

    if (args.sense_cycle_ms <= 0) {
        std::cerr << "--sense-cycle-ms must be greater than zero" << std::endl;
        return 2;
    }
    if (args.udp_port == args.gcs_udp_port) {
        std::cerr << "flight and GCS target UDP ports must be different" << std::endl;
        return 2;
    }

    std::cout << "Polling actual YOLO /target at http://" << args.yolo_host << ":"
              << args.yolo_port << "/target." << std::endl;
    std::cout << "Publishing range on UDP 127.0.0.1:" << args.udp_port
              << " and GCS copy on UDP 127.0.0.1:" << args.gcs_udp_port << " every "
              << args.sense_cycle_ms << " ms." << std::endl;

    TargetRangeSender flight_sender(args.udp_port);
    TargetRangeSender gcs_sender(args.gcs_udp_port);
    uint32_t seq = 0;
    double cycle_sec = args.sense_cycle_ms / 1000.0;

    while (true) {
        TargetRangeMsg out{};
        out.seq = seq++;

        bool have_target = false;
        try {
            std::string body = http_get(args.yolo_host, args.yolo_port, "/target", 0.2);
            bool found = target_json::json_bool(body, "found").value_or(false);
            // Handoff stability is owned by FlightMissionApp. Keep parsing
            // neither the legacy confirmation field nor its frame-count
            // semantics here: this process forwards fresh YOLO pixels only.
            double age_ms = target_json::json_number(body, "age_ms").value_or(NAN);
            std::string source = target_json::json_string(body, "source").value_or("");
            std::string target_name = target_json::json_string(body, "name").value_or("");
            double target_conf = target_json::json_number(body, "conf").value_or(NAN);
            double detection_count = target_json::json_number(body, "detections").value_or(0.0);
            double frame_sequence = target_json::json_number(body, "frame_sequence").value_or(-1.0);
            double frame_timestamp_ns = target_json::json_number(body, "frame_timestamp_ns").value_or(-1.0);
            double bbox_x_px = target_json::json_number(body, "bbox_x_px").value_or(0.0);
            double bbox_y_px = target_json::json_number(body, "bbox_y_px").value_or(0.0);
            double bbox_width_px = target_json::json_number(body, "bbox_width_px").value_or(0.0);
            double bbox_height_px = target_json::json_number(body, "bbox_height_px").value_or(0.0);
            args.frame_width_px = target_json::json_number(body, "width").value_or(args.frame_width_px);
            args.frame_height_px = target_json::json_number(body, "height").value_or(args.frame_height_px);
            out.observation_age_ms = std::isfinite(age_ms) ? static_cast<float>(age_ms) : -1.0F;
            out.target_confidence = std::isfinite(target_conf) ? static_cast<float>(target_conf) : -1.0F;
            out.frame_sequence = frame_sequence >= 0 ? static_cast<uint64_t>(frame_sequence) : 0;
            out.frame_timestamp_ns = frame_timestamp_ns >= 0 ? static_cast<int64_t>(frame_timestamp_ns) : -1;
            out.frame_width = static_cast<uint32_t>(std::max(1.0, args.frame_width_px));
            out.frame_height = static_cast<uint32_t>(std::max(1.0, args.frame_height_px));
            out.bbox_x_px = static_cast<float>(bbox_x_px);
            out.bbox_y_px = static_cast<float>(bbox_y_px);
            out.bbox_width_px = static_cast<float>(bbox_width_px);
            out.bbox_height_px = static_cast<float>(bbox_height_px);
            std::snprintf(out.class_name, sizeof(out.class_name), "%s", target_name.c_str());
            std::snprintf(out.source, sizeof(out.source), "%s", source.c_str());
            if (!source.empty()) {
                std::cout << "[target-observation] source=" << source
                          << " frame_sequence=" << std::fixed << std::setprecision(0) << frame_sequence
                          << " frame_timestamp_ns=" << frame_timestamp_ns
                          << " age_ms=" << std::setprecision(1) << age_ms
                          << " name=" << (target_name.empty() ? "-" : target_name)
                          << " conf=" << std::setprecision(3) << target_conf
                          << " detections=" << std::setprecision(0) << detection_count
                          << " found=" << (found ? 1 : 0) << std::endl;
            }
            if (found && !std::isnan(age_ms) && age_ms <= args.target_stale_ms) {
                out.x_px = static_cast<float>(target_json::json_number(body, "x_px").value_or(NAN));
                out.y_px = static_cast<float>(target_json::json_number(body, "y_px").value_or(NAN));
                have_target = std::isfinite(out.x_px) && std::isfinite(out.y_px);
            }
        } catch (const std::exception& e) {
            std::cerr << "YOLO /target poll failed: " << e.what() << std::endl;
        }

        if (have_target) {
            const PixelCalibration calibration{
                args.focal_x_px, args.focal_y_px, args.principal_x_px, args.principal_y_px};
            const auto angles = downward_pixel_to_angles(out.x_px, out.y_px, calibration);
            out.theta_forward_rad = static_cast<float>(angles.forward_rad);
            out.theta_right_rad = static_cast<float>(angles.right_rad);
            out.normalized_x = static_cast<float>(out.x_px / (args.frame_width_px * 0.5));
            out.normalized_y = static_cast<float>(out.y_px / (args.frame_height_px * 0.5));
        }

        out.found = have_target ? 1 : 0;
        // Compatibility metadata only. FlightMissionApp does not use this
        // field as a handoff gate.
        out.confirmed = out.found;
        // `valid` now describes a fresh YOLO pixel observation only. Vehicle
        // altitude, distance, and telemetry freshness are filled/validated by
        // FlightMissionApp from AutopilotState.
        out.valid = have_target ? 1 : 0;

        // Keep the FlightMissionApp and GCS subscribers independent. A UDP
        // datagram is consumed by one socket, so both copies are sent from
        // the same TargetRangeMsg without changing the flight payload.
        flight_sender.send(out);
        gcs_sender.send(out);

        std::cout << std::fixed << std::setprecision(2) << "target="
                  << (out.found ? "yes" : "no ") << " x_px=" << out.x_px << " y_px=" << out.y_px
                  << " valid=" << static_cast<int>(out.valid) << std::endl;
        std::this_thread::sleep_for(std::chrono::duration<double>(cycle_sec));
    }
}

int main(int argc, char** argv) {
    try {
        return run(argc, argv);
    } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return 1;
    }
}
