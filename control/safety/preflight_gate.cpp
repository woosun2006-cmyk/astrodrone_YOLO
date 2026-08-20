#include "preflight_gate.hpp"

#include <algorithm>

namespace safety {

PreflightGate::PreflightGate(PreflightPolicy policy) : policy_(policy) {}

void PreflightGate::observe(const PreflightSample& sample) {
    const auto now = sample.now;

    if (sample.heartbeat_event) {
        if (sample.valid_autopilot_heartbeat) {
            if (!last_heartbeat_valid_ || heartbeat_stable_since_ == Clock::time_point{}) {
                heartbeat_stable_since_ = now;
                heartbeat_count_ = 1;
            } else {
                ++heartbeat_count_;
            }
            last_heartbeat_at_ = now;
            last_heartbeat_valid_ = true;
            heartbeat_fresh_ = true;
        } else {
            heartbeat_count_ = 0;
            heartbeat_stable_since_ = Clock::time_point{};
            last_heartbeat_valid_ = false;
            heartbeat_fresh_ = false;
        }
    } else {
        heartbeat_fresh_ = sample.heartbeat_fresh;
    }

    health_ok_ = sample.gps_ok && sample.ekf_ok && sample.battery_valid;
    rc_policy_ok_ = sample.rc_policy_ok;
    telemetry_fresh_ = sample.telemetry_fresh;
    yolo_ready_ = sample.yolo_ready;

    if (sample.camera_frame) {
        ++camera_frame_count_;
        last_camera_frame_at_ = now;
        camera_frame_ = true;
    }

    if (health_ok_) {
        if (health_stable_since_ == Clock::time_point{}) health_stable_since_ = now;
    } else {
        health_stable_since_ = Clock::time_point{};
    }

    recompute(now);
}

void PreflightGate::poll(Clock::time_point now) {
    if (last_heartbeat_at_ != Clock::time_point{} &&
        std::chrono::duration<double>(now - last_heartbeat_at_).count() >
            policy_.heartbeat_freshness_sec) {
        last_heartbeat_valid_ = false;
        heartbeat_count_ = 0;
        heartbeat_stable_since_ = Clock::time_point{};
        heartbeat_fresh_ = false;
    }
    if (last_camera_frame_at_ != Clock::time_point{} &&
        std::chrono::duration<double>(now - last_camera_frame_at_).count() >
            policy_.telemetry_freshness_sec) {
        camera_frame_ = false;
    }
    recompute(now);
}

void PreflightGate::recompute(Clock::time_point now) {
    reasons_.clear();

    const bool heartbeat_count_ok = heartbeat_count_ >= policy_.required_heartbeat_count;
    const bool heartbeat_time_ok =
        heartbeat_stable_since_ != Clock::time_point{} &&
        std::chrono::duration<double>(now - heartbeat_stable_since_).count() >=
            policy_.required_heartbeat_stable_sec;
    if (!heartbeat_fresh_ || !heartbeat_count_ok || !heartbeat_time_ok) {
        reasons_.emplace_back("autopilot HEARTBEAT stability incomplete");
    }

    const bool health_time_ok =
        health_stable_since_ != Clock::time_point{} &&
        std::chrono::duration<double>(now - health_stable_since_).count() >=
            policy_.required_health_stable_sec;
    if (!health_ok_ || !health_time_ok) reasons_.emplace_back("GPS/EKF/battery stability incomplete");
    if (policy_.require_rc_policy && !rc_policy_ok_) reasons_.emplace_back("RC policy failed");
    if (!telemetry_fresh_) reasons_.emplace_back("telemetry is not fresh");
    if (policy_.require_vision && !yolo_ready_) reasons_.emplace_back("YOLO readiness incomplete");
    if (policy_.require_vision &&
        camera_frame_count_ < policy_.required_camera_frames) {
        reasons_.emplace_back("camera frame stability incomplete");
    }

    ready_ = reasons_.empty();
}

}  // namespace safety
