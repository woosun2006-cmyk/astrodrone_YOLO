#pragma once

#include <chrono>
#include <string>
#include <vector>

#include "../autopilot/vehicle_state.hpp"
#include "../third_party/mavlink/common/mavlink.h"

namespace safety {

struct HealthLimit {
    long min_battery_percent = 20;
    double min_battery_voltage_v = 14.8;
    double max_heartbeat_gap_sec = 3;
    long min_fix_type = 3;
    long min_satellites = 6;
    bool require_prearm_healthy = true;
    bool require_normal_state = true;
    double poll_rate_hz = 5;
    double land_gap_sec = 5;
    double ekf_pos_horiz_variance_max = 1.0;
    double ekf_velocity_variance_max = 1.0;
};

struct SafetyStatus {
    bool healthy = true;
    std::vector<std::string> reasons;

    explicit operator bool() const { return healthy; }
};

using HealthState = autopilot::VehicleState;

class HealthMonitor {
public:
    explicit HealthMonitor(HealthLimit limits = {}) : limits_(limits) {}

    void update(const mavlink_message_t& message,
                std::chrono::steady_clock::time_point now);
    SafetyStatus status(std::chrono::steady_clock::time_point now) const;

    const HealthState& state() const { return state_; }
    const HealthLimit& limits() const { return limits_; }

private:
    HealthState state_;
    HealthLimit limits_;
};

// Compatibility functions used by the current control.cpp. They are pure
// telemetry/state operations and never send MAVLink commands.
std::vector<std::string> evaluate_health_breach(
    const HealthState& state, const HealthLimit& limits,
    std::chrono::steady_clock::time_point now);
void apply_health_message(const mavlink_message_t& message, HealthState& state,
                          std::chrono::steady_clock::time_point now);

}  // namespace safety
