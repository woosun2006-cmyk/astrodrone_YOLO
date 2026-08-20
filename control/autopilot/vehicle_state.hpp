#pragma once

#include <chrono>
#include <cstdint>

namespace autopilot {

// Telemetry snapshot shared by safety and mission layers. Each telemetry
// group carries its own update timestamp so freshness checks do not depend on
// the rate of unrelated MAVLink messages.
struct VehicleState {
    using Clock = std::chrono::steady_clock;

    Clock::time_point last_heartbeat{};
    Clock::time_point battery_updated_at{};
    Clock::time_point altitude_updated_at{};
    Clock::time_point gps_updated_at{};
    Clock::time_point ekf_updated_at{};
    Clock::time_point mode_updated_at{};

    bool have_battery = false;
    bool battery_valid = false;
    int battery_percent = -1;
    double battery_voltage_v = -1;

    bool have_altitude = false;
    double altitude_m = 0;

    bool have_gps = false;
    uint8_t fix_type = 0;
    uint8_t satellites = 255;  // 255 == unknown, per MAVLink convention
    bool have_position = false;
    double lat = 0;
    double lon = 0;

    bool have_sys_status = false;
    bool prearm_healthy = true;
    uint8_t system_status = 3;  // MAV_STATE_STANDBY

    uint32_t custom_mode = 0;
    bool have_armed = false;
    bool armed = false;

    bool have_ekf = false;
    double ekf_pos_horiz_variance = 0;
    double ekf_velocity_variance = 0;
};

}  // namespace autopilot
