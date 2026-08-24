#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <string>

namespace autopilot {

// Single MAVLink telemetry snapshot owned by AutopilotMavlinkAdapter.
struct AutopilotState {
    using Clock = std::chrono::steady_clock;

    Clock::time_point last_heartbeat{};
    // Trace-only heartbeat provenance. These fields do not participate in
    // freshness or safety decisions.
    Clock::time_point last_heartbeat_observed_at{};
    Clock::time_point last_heartbeat_state_updated_at{};
    uint8_t last_heartbeat_mav_seq = 0;
    uint64_t heartbeat_observation_count = 0;
    Clock::time_point battery_updated_at{};
    Clock::time_point altitude_updated_at{};
    Clock::time_point local_position_updated_at{};
    Clock::time_point gps_updated_at{};
    Clock::time_point ekf_updated_at{};
    Clock::time_point mode_updated_at{};
    Clock::time_point rc_updated_at{};
    Clock::time_point attitude_updated_at{};
    Clock::time_point status_text_updated_at{};
    Clock::time_point command_ack_updated_at{};
    Clock::time_point raw_imu_updated_at{};
    Clock::time_point highres_imu_updated_at{};
    Clock::time_point vibration_updated_at{};
    Clock::time_point home_updated_at{};
    Clock::time_point mission_updated_at{};

    bool have_battery = false;
    bool battery_valid = false;
    int battery_percent = -1;
    double battery_voltage_v = -1;
    double battery_current_a = -1;

    bool have_altitude = false;
    double altitude_m = 0;

    bool have_local_position = false;
    double local_x_m = 0;
    double local_y_m = 0;
    double local_z_m = 0;
    double local_vx_mps = 0;
    double local_vy_mps = 0;
    double local_vz_mps = 0;

    bool have_gps = false;
    uint8_t fix_type = 0;
    uint8_t satellites = 255;
    bool have_position = false;
    double lat = 0;
    double lon = 0;

    bool have_sys_status = false;
    uint32_t sensor_present_bits = 0;
    uint32_t sensor_enabled_bits = 0;
    uint32_t sensor_health_bits = 0;
    bool prearm_healthy = true;
    uint8_t system_status = 3;

    uint32_t custom_mode = 0;
    uint8_t base_mode = 0;
    uint8_t heartbeat_system_id = 0;
    uint8_t heartbeat_component_id = 0;
    bool have_armed = false;
    bool armed = false;

    bool have_ekf = false;
    uint16_t ekf_flags = 0;
    double ekf_pos_horiz_variance = 0;
    double ekf_pos_vert_variance = 0;
    double ekf_velocity_variance = 0;

    bool have_rc = false;
    std::array<uint16_t, 18> rc_channels{};
    uint8_t rc_channel_count = 0;
    uint8_t rc_rssi = 255;

    bool have_attitude = false;
    double roll_rad = 0;
    double pitch_rad = 0;
    double yaw_rad = 0;
    double roll_rate_rps = 0;
    double pitch_rate_rps = 0;
    double yaw_rate_rps = 0;

    bool have_status_text = false;
    uint8_t status_text_severity = 0;
    std::string last_status_text;

    bool have_command_ack = false;
    uint16_t last_command_ack_command = 0;
    uint8_t last_command_ack_result = 0;
    int32_t last_command_ack_param2 = 0;

    bool have_raw_imu = false;
    uint32_t raw_imu_samples = 0;
    uint8_t raw_imu_id = 0;
    int16_t raw_imu_xacc = 0;
    int16_t raw_imu_yacc = 0;
    int16_t raw_imu_zacc = 0;

    bool have_highres_imu = false;
    uint32_t highres_imu_samples = 0;
    double highres_xacc = 0;
    double highres_yacc = 0;
    double highres_zacc = 0;

    bool have_vibration = false;
    uint32_t vibration_samples = 0;
    double vibration_x = 0;
    double vibration_y = 0;
    double vibration_z = 0;
    uint32_t vibration_clipping_0 = 0;
    uint32_t vibration_clipping_1 = 0;
    uint32_t vibration_clipping_2 = 0;

    bool have_home_position = false;
    double home_lat = 0;
    double home_lon = 0;
    double home_altitude_m = 0;
    double home_x_m = 0;
    double home_y_m = 0;
    double home_z_m = 0;

    bool have_mission_current = false;
    uint16_t mission_current_seq = 0;
    uint16_t mission_total = 0;
    uint8_t mission_state = 0;
    uint8_t mission_mode = 0;
};

}  // namespace autopilot
