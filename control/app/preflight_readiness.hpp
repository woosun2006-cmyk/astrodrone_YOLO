#pragma once

#include <chrono>
#include <cstdint>
#include <string>

namespace app {

struct CameraFrameReadiness {
    bool marker_present = false;
    bool valid = false;
    bool fresh = false;
    std::uint64_t frame_sequence = 0;
    std::int64_t frame_timestamp_ns = -1;
    std::int64_t monotonic_timestamp_ns = -1;
    std::uint32_t frame_width = 0;
    std::uint32_t frame_height = 0;
    double age_sec = -1.0;
    std::string source;
    std::string reason;
};

CameraFrameReadiness read_camera_frame_readiness(
    const std::string& marker_path,
    std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now(),
    double stale_timeout_sec = 1.0);

struct VisionReadiness {
    bool ready = false;
    bool yolo_marker_present = false;
    std::string reason;
};

VisionReadiness evaluate_vision_readiness(bool yolo_process_alive,
                                          bool http_endpoint_alive,
                                          bool yolo_marker_present,
                                          bool camera_source_ready,
                                          const CameraFrameReadiness& frame);

bool http_endpoint_alive(const std::string& endpoint);

}  // namespace app
