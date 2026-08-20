#pragma once

#include <chrono>
#include <cstddef>
#include <string>
#include <vector>

namespace safety {

struct PreflightPolicy {
    std::size_t required_heartbeat_count = 3;
    double required_heartbeat_stable_sec = 2.0;
    double required_health_stable_sec = 1.0;
    std::size_t required_camera_frames = 1;
    bool require_rc_policy = false;
    bool require_vision = false;
    // ArduPilot HEARTBEAT is normally about 1 Hz. Keep its freshness policy
    // separate from the faster GPS/EKF/SYS_STATUS telemetry policy.
    double heartbeat_freshness_sec = 3.0;
    double telemetry_freshness_sec = 1.0;
};

struct PreflightSample {
    using Clock = std::chrono::steady_clock;

    // heartbeat_event is true only when this sample represents a newly
    // received valid autopilot HEARTBEAT. Reusing a stale heartbeat flag must
    // not inflate the consecutive-heartbeat count.
    bool heartbeat_event = false;
    bool valid_autopilot_heartbeat = false;
    bool heartbeat_fresh = false;
    bool gps_ok = false;
    bool ekf_ok = false;
    bool battery_valid = false;
    bool rc_policy_ok = true;
    bool telemetry_fresh = false;
    bool yolo_ready = false;
    bool camera_frame = false;
    Clock::time_point now = Clock::now();
};

class PreflightGate {
public:
    using Clock = PreflightSample::Clock;

    explicit PreflightGate(PreflightPolicy policy = {});

    // Feed samples from either the SITL telemetry adapter or the real serial
    // adapter. This class never reads a socket and never sends MAVLink.
    void observe(const PreflightSample& sample);
    void poll(Clock::time_point now = Clock::now());

    bool ready() const { return ready_; }
    std::size_t consecutive_heartbeats() const { return heartbeat_count_; }
    std::size_t camera_frames() const { return camera_frame_count_; }
    const std::vector<std::string>& reasons() const { return reasons_; }
    const PreflightPolicy& policy() const { return policy_; }

private:
    void recompute(Clock::time_point now);

    PreflightPolicy policy_;
    bool last_heartbeat_valid_ = false;
    bool heartbeat_fresh_ = false;
    bool health_ok_ = false;
    bool rc_policy_ok_ = true;
    bool telemetry_fresh_ = false;
    bool yolo_ready_ = false;
    bool camera_frame_ = false;
    std::size_t heartbeat_count_ = 0;
    std::size_t camera_frame_count_ = 0;
    Clock::time_point heartbeat_stable_since_{};
    Clock::time_point health_stable_since_{};
    Clock::time_point last_heartbeat_at_{};
    Clock::time_point last_camera_frame_at_{};
    bool ready_ = false;
    std::vector<std::string> reasons_;
};

}  // namespace safety
