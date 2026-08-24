#pragma once

#include <cstdint>
#include <fstream>
#include <string>
#include <vector>

#include "../autopilot/autopilot_mavlink_adapter.hpp"

namespace mission {

struct MissionSetupConfig {
    double waypoint_lat = -35.363172;
    double waypoint_lon = 149.165237;
    double altitude_m = 5.0;
    double timeout_sec = 20.0;
    // Enabled only by the SITL launcher. Real runtime keeps this opt-in.
    bool require_raw_imu_stable = false;
    uint32_t raw_imu_min_samples = 5;
};

// Owns the mission upload/AUTO/ARM handshake for the C++ onboard control
// process. Every outbound mission packet goes through the adapter facade,
// while this class only handles MAVLink protocol responses and read-back checks.
class MissionSetup {
public:
    explicit MissionSetup(autopilot::AutopilotMavlinkAdapter& vehicle,
                          MissionSetupConfig config = {});

    // Throws on a rejected ACK, protocol timeout, or read-back mismatch.
    // Returns only after AUTO and ARM have been confirmed by HEARTBEAT.
    bool run();

private:
    void ensure_disarmed_and_loiter();
    void upload(double home_lat, double home_lon);
    void verify_readback(double home_lat, double home_lon);
    void set_current_and_start();
    void wait_raw_imu_stable(const char* phase);

    void wait_ack(const char* phase);
    uint16_t wait_request_seq(const char* phase);
    uint16_t wait_mission_count(const char* phase);
    mavlink_mission_item_int_t wait_mission_item(uint16_t expected_seq,
                                                 const char* phase);
    void wait_mode(uint32_t expected_mode, const char* phase);
    void wait_armed(const char* phase);
    bool receive_diagnostic(const std::vector<uint32_t>& message_ids,
                            mavlink_message_t& message, double timeout_sec,
                            const char* phase);
    void record_diagnostic_message(const mavlink_message_t& message,
                                   const char* phase);
    void write_diagnostic(const std::string& line);
    std::string arm_diagnostic_summary();

    autopilot::AutopilotMavlinkAdapter& vehicle_;
    MissionSetupConfig config_;
    std::ofstream diagnostic_log_;
    bool arm_ack_seen_ = false;
    uint8_t arm_ack_result_ = MAV_RESULT_ENUM_END;
    int32_t arm_ack_result_param2_ = 0;
    bool heartbeat_seen_ = false;
    uint32_t heartbeat_mode_ = 0;
    uint8_t heartbeat_base_mode_ = 0;
    bool heartbeat_armed_ = false;
    bool gps_seen_ = false;
    uint8_t gps_fix_type_ = 0;
    uint8_t gps_satellites_ = 0;
    bool ekf_seen_ = false;
    uint16_t ekf_flags_ = 0;
    float ekf_velocity_variance_ = 0.0F;
    float ekf_pos_horiz_variance_ = 0.0F;
    float ekf_pos_vert_variance_ = 0.0F;
    bool sys_status_seen_ = false;
    uint32_t sensor_health_bits_ = 0;
    uint16_t battery_voltage_mv_ = 0;
    int8_t battery_remaining_ = -1;
    bool rc_seen_ = false;
    uint32_t rc_time_boot_ms_ = 0;
    uint8_t rc_channel_count_ = 0;
    uint8_t rc_rssi_ = 255;
    bool home_seen_ = false;
    bool raw_imu_seen_ = false;
    uint32_t raw_imu_samples_ = 0;
    uint8_t raw_imu_last_id_ = 0;
    int16_t raw_imu_last_xacc_ = 0;
    int16_t raw_imu_last_yacc_ = 0;
    int16_t raw_imu_last_zacc_ = 0;
    bool highres_imu_seen_ = false;
    uint32_t highres_imu_samples_ = 0;
    float highres_imu_last_xacc_ = 0.0F;
    float highres_imu_last_yacc_ = 0.0F;
    float highres_imu_last_zacc_ = 0.0F;
    bool vibration_seen_ = false;
    uint32_t vibration_samples_ = 0;
    float vibration_last_x_ = 0.0F;
    float vibration_last_y_ = 0.0F;
    float vibration_last_z_ = 0.0F;
    uint32_t vibration_last_clipping_0_ = 0;
    uint32_t vibration_last_clipping_1_ = 0;
    uint32_t vibration_last_clipping_2_ = 0;
    bool mission_count_seen_ = false;
    uint16_t mission_count_ = 0;
    bool mission_current_seen_ = false;
    uint16_t mission_current_ = 0;
    bool mission_ack_seen_ = false;
    uint8_t mission_ack_type_ = MAV_MISSION_TYPE_MISSION;
    std::vector<std::string> arm_status_texts_;
};

}  // namespace mission
