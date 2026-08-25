#include "preflight_readiness.hpp"

#include <arpa/inet.h>
#include <chrono>
#include <fstream>
#include <map>
#include <netdb.h>
#include <sstream>
#include <sys/socket.h>
#include <type_traits>
#include <unistd.h>

namespace app {
namespace {

std::map<std::string, std::string> read_key_values(const std::string& path) {
    std::ifstream input(path);
    std::map<std::string, std::string> values;
    std::string line;
    while (std::getline(input, line)) {
        const std::size_t separator = line.find('=');
        if (separator == std::string::npos) continue;
        values[line.substr(0, separator)] = line.substr(separator + 1);
    }
    return values;
}

template <typename T>
bool parse_number(const std::map<std::string, std::string>& values,
                 const char* key, T& output) {
    const auto it = values.find(key);
    if (it == values.end()) return false;
    try {
        if constexpr (std::is_same_v<T, std::uint64_t>) {
            output = static_cast<T>(std::stoull(it->second));
        } else if constexpr (std::is_same_v<T, std::int64_t>) {
            output = static_cast<T>(std::stoll(it->second));
        } else {
            output = static_cast<T>(std::stoul(it->second));
        }
        return true;
    } catch (...) {
        return false;
    }
}

bool parse_endpoint(const std::string& endpoint, std::string& host, std::string& port) {
    std::string value = endpoint;
    const std::size_t scheme = value.find("://");
    if (scheme != std::string::npos) value.erase(0, scheme + 3);
    const std::size_t slash = value.find('/');
    if (slash != std::string::npos) value.resize(slash);
    const std::size_t separator = value.rfind(':');
    if (separator == std::string::npos || separator == 0 || separator + 1 >= value.size()) {
        return false;
    }
    host = value.substr(0, separator);
    port = value.substr(separator + 1);
    return true;
}

}  // namespace

CameraFrameReadiness read_camera_frame_readiness(
    const std::string& marker_path, std::chrono::steady_clock::time_point now,
    double stale_timeout_sec) {
    CameraFrameReadiness result;
    const auto values = read_key_values(marker_path);
    result.marker_present = !values.empty();
    if (!result.marker_present) {
        result.reason = "실제 frame 미수신";
        return result;
    }

    const bool parsed = parse_number(values, "frame_sequence", result.frame_sequence) &&
                        parse_number(values, "frame_timestamp_ns", result.frame_timestamp_ns) &&
                        parse_number(values, "monotonic_timestamp_ns", result.monotonic_timestamp_ns) &&
                        parse_number(values, "width", result.frame_width) &&
                        parse_number(values, "height", result.frame_height);
    const auto source = values.find("source");
    if (source != values.end()) result.source = source->second;
    result.valid = parsed && result.frame_sequence > 0 &&
                   result.monotonic_timestamp_ns > 0 && result.frame_width > 0 &&
                   result.frame_height > 0 && !result.source.empty();
    if (!result.valid) {
        result.reason = "frame metadata 오류";
        return result;
    }

    const auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        now.time_since_epoch()).count();
    result.age_sec = static_cast<double>(now_ns - result.monotonic_timestamp_ns) / 1e9;
    result.fresh = result.age_sec >= 0.0 && result.age_sec <= stale_timeout_sec;
    if (!result.fresh) result.reason = "frame stale";
    return result;
}

VisionReadiness evaluate_vision_readiness(bool yolo_process_alive,
                                          bool http_alive,
                                          bool yolo_marker_present,
                                          bool camera_source_ready,
                                          const CameraFrameReadiness& frame) {
    if (!yolo_process_alive) return {false, false, "YOLO 프로세스 미실행"};
    if (!http_alive) return {false, false, "HTTP readiness 실패"};
    if (!yolo_marker_present) return {false, false, "YOLO readiness marker 없음"};
    if (!camera_source_ready) return {false, true, "카메라 topic 미수신"};
    if (!frame.marker_present) return {false, true, "실제 frame 미수신"};
    if (!frame.valid) return {false, true, "frame metadata 오류"};
    if (!frame.fresh) return {false, true, "frame stale"};
    return {true, true, {}};
}

bool http_endpoint_alive(const std::string& endpoint) {
    std::string host;
    std::string port;
    if (!parse_endpoint(endpoint, host, port)) return false;

    addrinfo hints{};
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_family = AF_UNSPEC;
    addrinfo* results = nullptr;
    if (getaddrinfo(host.c_str(), port.c_str(), &hints, &results) != 0) return false;

    bool alive = false;
    for (addrinfo* current = results; current != nullptr && !alive; current = current->ai_next) {
        const int fd = ::socket(current->ai_family, current->ai_socktype, current->ai_protocol);
        if (fd < 0) continue;
        timeval timeout{0, 250000};
        ::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
        ::setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
        if (::connect(fd, current->ai_addr, current->ai_addrlen) == 0) {
            const std::string request = "GET /target HTTP/1.0\r\nHost: " + host +
                                        "\r\nConnection: close\r\n\r\n";
            if (::send(fd, request.data(), request.size(), 0) >= 0) {
                char response[64]{};
                const ssize_t received = ::recv(fd, response, sizeof(response) - 1, 0);
                alive = received >= 5 && std::string(response, static_cast<std::size_t>(received))
                                             .rfind("HTTP/", 0) == 0;
            }
        }
        ::close(fd);
    }
    freeaddrinfo(results);
    return alive;
}

bool yolo_ready_marker_valid(const std::string& marker_path,
                             std::int64_t expected_pid) {
    if (marker_path.empty() || expected_pid <= 0) return false;
    const auto values = read_key_values(marker_path);
    const auto ready = values.find("ready");
    const auto pid = values.find("pid");
    const auto start = values.find("start_monotonic_ns");
    if (ready == values.end() || pid == values.end() || start == values.end() ||
        ready->second != "1") {
        return false;
    }
    try {
        return std::stoll(pid->second) == expected_pid &&
               std::stoll(start->second) > 0;
    } catch (...) {
        return false;
    }
}

}  // namespace app
