#include "mission_setup.hpp"

#include <algorithm>
#include <cmath>
#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>

#include "../drone_lib.hpp"

namespace mission {
namespace {

constexpr uint8_t kMissionType = MAV_MISSION_TYPE_MISSION;

const char* mav_result_name(uint8_t result) {
    switch (result) {
        case MAV_RESULT_ACCEPTED: return "ACCEPTED";
        case MAV_RESULT_TEMPORARILY_REJECTED: return "TEMPORARILY_REJECTED";
        case MAV_RESULT_DENIED: return "DENIED";
        case MAV_RESULT_UNSUPPORTED: return "UNSUPPORTED";
        case MAV_RESULT_FAILED: return "FAILED";
        case MAV_RESULT_IN_PROGRESS: return "IN_PROGRESS";
        case MAV_RESULT_CANCELLED: return "CANCELLED";
        default: return "UNKNOWN";
    }
}

std::string trim_status_text(const mavlink_statustext_t& status) {
    std::string text(status.text, sizeof(status.text));
    const auto nul = text.find('\0');
    if (nul != std::string::npos) text.resize(nul);
    return text;
}

bool vehicle_message(const autopilot::AutopilotMavlinkAdapter& vehicle,
                     const mavlink_message_t& message) {
    return message.sysid == vehicle.target_system();
}

void require_sent(const safety::CommandDecision& decision, const char* phase) {
    if (!decision) {
        throw std::runtime_error(std::string("mission setup command blocked: ") + phase +
                                 " (" + safety::gate_block_reason_name(decision.block_reason) + ")");
    }
}

}  // namespace

MissionSetup::MissionSetup(autopilot::AutopilotMavlinkAdapter& vehicle,
                           MissionSetupConfig config)
    : vehicle_(vehicle), config_(config) {
    const char* require_imu = std::getenv("SITL_REQUIRE_RAW_IMU_STABLE");
    if (require_imu != nullptr && std::string(require_imu) == "1") {
        config_.require_raw_imu_stable = true;
    }
    const char* min_samples = std::getenv("SITL_RAW_IMU_MIN_SAMPLES");
    if (config_.require_raw_imu_stable && min_samples != nullptr && *min_samples != '\0') {
        errno = 0;
        char* end = nullptr;
        const unsigned long parsed = std::strtoul(min_samples, &end, 10);
        if (errno != 0 || end == min_samples || *end != '\0' || parsed == 0 ||
            parsed > std::numeric_limits<uint32_t>::max()) {
            throw std::runtime_error("mission setup: invalid SITL_RAW_IMU_MIN_SAMPLES");
        }
        config_.raw_imu_min_samples = static_cast<uint32_t>(parsed);
    }
    const char* path = std::getenv("ARM_DIAGNOSTIC_LOG");
    if (path != nullptr && *path != '\0') diagnostic_log_.open(path, std::ios::out | std::ios::trunc);
    write_diagnostic("ARM_GATE_DIAGNOSTICS_BEGIN");
    {
        std::ostringstream line;
        line << "ARM_GATE_CONFIG require_raw_imu_stable="
             << (config_.require_raw_imu_stable ? "true" : "false")
             << " raw_imu_min_samples=" << config_.raw_imu_min_samples;
        write_diagnostic(line.str());
    }
}

bool MissionSetup::run() {
    ensure_disarmed_and_loiter();

    mavlink_message_t message{};
    if (!receive_diagnostic({MAVLINK_MSG_ID_GPS_RAW_INT}, message, config_.timeout_sec,
                            "mission setup GPS") ||
        !vehicle_message(vehicle_, message)) {
        throw std::runtime_error("mission setup: 현재 GPS 위치를 받지 못했습니다");
    }
    mavlink_gps_raw_int_t gps{};
    mavlink_msg_gps_raw_int_decode(&message, &gps);
    if (gps.lat == 0 || gps.lon == 0) {
        throw std::runtime_error("mission setup: 현재 GPS 위치가 유효하지 않습니다");
    }

    const double home_lat = static_cast<double>(gps.lat) / 1e7;
    const double home_lon = static_cast<double>(gps.lon) / 1e7;
    upload(home_lat, home_lon);
    verify_readback(home_lat, home_lon);
    set_current_and_start();
    return true;
}

void MissionSetup::ensure_disarmed_and_loiter() {
    mavlink_message_t message{};
    if (!receive_diagnostic({MAVLINK_MSG_ID_HEARTBEAT}, message, config_.timeout_sec,
                            "setup heartbeat") ||
        !vehicle_message(vehicle_, message)) {
        throw std::runtime_error("mission setup: 초기 HEARTBEAT를 받지 못했습니다");
    }
    mavlink_heartbeat_t heartbeat{};
    mavlink_msg_heartbeat_decode(&message, &heartbeat);
    if (is_armed_from_heartbeat(heartbeat)) {
        throw std::runtime_error("mission setup: 기존 ARM 상태에서는 미션을 교체하지 않습니다");
    }
    if (heartbeat.custom_mode != copter_mode_mapping().at("LOITER")) {
        require_sent(vehicle_.set_mode("LOITER"), "LOITER");
        wait_mode(copter_mode_mapping().at("LOITER"), "LOITER 확인");
    }
}

void MissionSetup::upload(double home_lat, double home_lon) {
    require_sent(vehicle_.send_mission_clear(), "MISSION_CLEAR_ALL");
    wait_ack("MISSION_CLEAR_ALL");

    require_sent(vehicle_.send_mission_count(3), "MISSION_COUNT");
    for (uint16_t expected = 0; expected < 3; ++expected) {
        const uint16_t seq = wait_request_seq("MISSION_REQUEST");
        if (seq != expected) {
            throw std::runtime_error("mission setup: 요청된 mission sequence가 예상과 다릅니다");
        }

        const double lat = expected == 2 ? config_.waypoint_lat : home_lat;
        const double lon = expected == 2 ? config_.waypoint_lon : home_lon;
        const uint16_t command = expected == 1 ? MAV_CMD_NAV_TAKEOFF : MAV_CMD_NAV_WAYPOINT;
        const float altitude = expected == 1 ? static_cast<float>(config_.altitude_m)
                                             : (expected == 2 ? static_cast<float>(config_.altitude_m)
                                                              : 0.0F);
        require_sent(vehicle_.send_mission_item(
                         expected, command, MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                         static_cast<int32_t>(std::llround(lat * 1e7)),
                         static_cast<int32_t>(std::llround(lon * 1e7)), altitude),
                     "MISSION_ITEM_INT");
    }
    wait_ack("MISSION upload");
}

void MissionSetup::verify_readback(double home_lat, double home_lon) {
    require_sent(vehicle_.send_mission_request_list(), "MISSION_REQUEST_LIST");
    if (wait_mission_count("MISSION_COUNT read-back") != 3) {
        throw std::runtime_error("mission setup: read-back mission count가 3이 아닙니다");
    }

    const int32_t home_x = static_cast<int32_t>(std::llround(home_lat * 1e7));
    const int32_t home_y = static_cast<int32_t>(std::llround(home_lon * 1e7));
    const int32_t waypoint_x = static_cast<int32_t>(std::llround(config_.waypoint_lat * 1e7));
    const int32_t waypoint_y = static_cast<int32_t>(std::llround(config_.waypoint_lon * 1e7));
    for (uint16_t seq = 0; seq < 3; ++seq) {
        require_sent(vehicle_.send_mission_request_item(seq), "MISSION_REQUEST_INT");
        const mavlink_mission_item_int_t item = wait_mission_item(seq, "MISSION_ITEM_INT read-back");
        const uint16_t expected_command = seq == 1 ? MAV_CMD_NAV_TAKEOFF : MAV_CMD_NAV_WAYPOINT;
        const int32_t expected_x = seq == 2 ? waypoint_x : home_x;
        const int32_t expected_y = seq == 2 ? waypoint_y : home_y;
        if (item.command != expected_command || item.x != expected_x || item.y != expected_y) {
            throw std::runtime_error("mission setup: read-back mission item이 업로드 내용과 다릅니다");
        }
    }
}

void MissionSetup::set_current_and_start() {
    require_sent(vehicle_.send_mission_set_current(0), "MISSION_SET_CURRENT");
    mavlink_message_t message{};
        if (!receive_diagnostic({MAVLINK_MSG_ID_MISSION_CURRENT}, message, config_.timeout_sec,
                                "mission current") ||
        !vehicle_message(vehicle_, message)) {
        throw std::runtime_error("mission setup: mission current 확인 timeout");
    }
    mavlink_mission_current_t current{};
    mavlink_msg_mission_current_decode(&message, &current);
    if (current.seq != 0) throw std::runtime_error("mission setup: mission current가 seq 0이 아닙니다");

    require_sent(vehicle_.set_mode("AUTO"), "AUTO");
    wait_mode(copter_mode_mapping().at("AUTO"), "AUTO 확인");
    if (config_.require_raw_imu_stable) {
        wait_raw_imu_stable("ARM 전 raw IMU 안정성");
    }
    require_sent(vehicle_.arm_disarm(true), "ARM");
    wait_armed("ARM 확인");
}

void MissionSetup::wait_raw_imu_stable(const char* phase) {
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::duration<double>(config_.timeout_sec);
    const uint32_t samples_at_start = raw_imu_samples_;
    while (std::chrono::steady_clock::now() < deadline) {
        mavlink_message_t message{};
        if (!receive_diagnostic({MAVLINK_MSG_ID_RAW_IMU, MAVLINK_MSG_ID_HIGHRES_IMU,
                                 MAVLINK_MSG_ID_VIBRATION}, message, 1.0, phase) ||
            !vehicle_message(vehicle_, message)) {
            continue;
        }
        if (message.msgid == MAVLINK_MSG_ID_VIBRATION &&
            (vibration_last_clipping_0_ != 0 || vibration_last_clipping_1_ != 0 ||
             vibration_last_clipping_2_ != 0)) {
            throw std::runtime_error("mission setup: ARM 전 VIBRATION clipping 감지");
        }
        const bool prearm_ok = !vehicle_.state().have_sys_status || vehicle_.state().prearm_healthy;
        if (raw_imu_samples_ - samples_at_start >= config_.raw_imu_min_samples && prearm_ok) {
            std::ostringstream line;
            line << phase << " PASS raw_imu_samples=" << raw_imu_samples_
                 << " vibration_samples=" << vibration_samples_
                 << " required=" << config_.raw_imu_min_samples
                 << " prearm_healthy=" << (prearm_ok ? "true" : "false")
                 << " clipping=" << vibration_last_clipping_0_ << ","
                 << vibration_last_clipping_1_ << "," << vibration_last_clipping_2_;
            write_diagnostic(line.str());
            return;
        }
    }
    std::ostringstream line;
    line << phase << " FAIL raw_imu_samples=" << raw_imu_samples_
         << " required=" << config_.raw_imu_min_samples
         << " prearm_healthy="
         << ((!vehicle_.state().have_sys_status || vehicle_.state().prearm_healthy) ? "true" : "false");
    write_diagnostic(line.str());
    throw std::runtime_error("mission setup: ARM 전 raw IMU 안정성 확인 timeout");
}

void MissionSetup::wait_ack(const char* phase) {
    mavlink_message_t message{};
    if (!receive_diagnostic({MAVLINK_MSG_ID_MISSION_ACK}, message, config_.timeout_sec,
                            phase) ||
        !vehicle_message(vehicle_, message)) {
        throw std::runtime_error(std::string("mission setup: ") + phase + " ACK timeout");
    }
    mavlink_mission_ack_t ack{};
    mavlink_msg_mission_ack_decode(&message, &ack);
    if (ack.type != MAV_MISSION_ACCEPTED) {
        throw std::runtime_error(std::string("mission setup: ") + phase + " rejected");
    }
}

uint16_t MissionSetup::wait_request_seq(const char* phase) {
    mavlink_message_t message{};
    if (!receive_diagnostic({MAVLINK_MSG_ID_MISSION_REQUEST_INT, MAVLINK_MSG_ID_MISSION_REQUEST},
                            message, config_.timeout_sec, phase) ||
        !vehicle_message(vehicle_, message)) {
        throw std::runtime_error(std::string("mission setup: ") + phase + " timeout");
    }
    if (message.msgid == MAVLINK_MSG_ID_MISSION_REQUEST_INT) {
        mavlink_mission_request_int_t request{};
        mavlink_msg_mission_request_int_decode(&message, &request);
        if (request.mission_type != kMissionType) throw std::runtime_error("mission setup: mission type mismatch");
        return request.seq;
    }
    mavlink_mission_request_t request{};
    mavlink_msg_mission_request_decode(&message, &request);
    if (request.mission_type != kMissionType) throw std::runtime_error("mission setup: mission type mismatch");
    return request.seq;
}

uint16_t MissionSetup::wait_mission_count(const char* phase) {
    mavlink_message_t message{};
    if (!receive_diagnostic({MAVLINK_MSG_ID_MISSION_COUNT}, message, config_.timeout_sec,
                            phase) ||
        !vehicle_message(vehicle_, message)) {
        throw std::runtime_error(std::string("mission setup: ") + phase + " timeout");
    }
    mavlink_mission_count_t count{};
    mavlink_msg_mission_count_decode(&message, &count);
    if (count.mission_type != kMissionType) throw std::runtime_error("mission setup: mission type mismatch");
    return count.count;
}

mavlink_mission_item_int_t MissionSetup::wait_mission_item(uint16_t expected_seq,
                                                            const char* phase) {
    mavlink_message_t message{};
    if (!receive_diagnostic({MAVLINK_MSG_ID_MISSION_ITEM_INT}, message, config_.timeout_sec,
                            phase) ||
        !vehicle_message(vehicle_, message)) {
        throw std::runtime_error(std::string("mission setup: ") + phase + " timeout");
    }
    mavlink_mission_item_int_t item{};
    mavlink_msg_mission_item_int_decode(&message, &item);
    if (item.seq != expected_seq || item.mission_type != kMissionType) {
        throw std::runtime_error(std::string("mission setup: ") + phase + " sequence/type mismatch");
    }
    return item;
}

void MissionSetup::wait_mode(uint32_t expected_mode, const char* phase) {
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::duration<double>(config_.timeout_sec);
    while (std::chrono::steady_clock::now() < deadline) {
        mavlink_message_t message{};
        if (!receive_diagnostic({MAVLINK_MSG_ID_HEARTBEAT}, message, 1.0, phase) ||
            !vehicle_message(vehicle_, message)) continue;
        mavlink_heartbeat_t heartbeat{};
        mavlink_msg_heartbeat_decode(&message, &heartbeat);
        if (heartbeat.custom_mode == expected_mode && !is_armed_from_heartbeat(heartbeat)) {
            return;
        }
        if (expected_mode == copter_mode_mapping().at("AUTO") &&
            heartbeat.custom_mode == expected_mode) return;
    }
    throw std::runtime_error(std::string("mission setup: ") + phase + " timeout");
}

void MissionSetup::wait_armed(const char* phase) {
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::duration<double>(config_.timeout_sec);
    auto failure_deadline = deadline;
    while (std::chrono::steady_clock::now() < deadline) {
        mavlink_message_t message{};
        if (!receive_diagnostic({MAVLINK_MSG_ID_COMMAND_ACK, MAVLINK_MSG_ID_HEARTBEAT,
                                 MAVLINK_MSG_ID_STATUSTEXT, MAVLINK_MSG_ID_GPS_RAW_INT,
                                 MAVLINK_MSG_ID_EKF_STATUS_REPORT, MAVLINK_MSG_ID_SYS_STATUS,
                                 MAVLINK_MSG_ID_RC_CHANNELS, MAVLINK_MSG_ID_HOME_POSITION,
                                 MAVLINK_MSG_ID_RAW_IMU, MAVLINK_MSG_ID_HIGHRES_IMU,
                                 MAVLINK_MSG_ID_VIBRATION,
                                 MAVLINK_MSG_ID_MISSION_COUNT, MAVLINK_MSG_ID_MISSION_CURRENT,
                                 MAVLINK_MSG_ID_MISSION_ACK},
                                message, 1.0, phase) || !vehicle_message(vehicle_, message)) {
            continue;
        }
        if (message.msgid == MAVLINK_MSG_ID_COMMAND_ACK) {
            mavlink_command_ack_t ack{};
            mavlink_msg_command_ack_decode(&message, &ack);
            if (ack.command == MAV_CMD_COMPONENT_ARM_DISARM) {
                arm_ack_seen_ = true;
                arm_ack_result_ = ack.result;
                arm_ack_result_param2_ = ack.result_param2;
                if (ack.result != MAV_RESULT_ACCEPTED && ack.result != MAV_RESULT_IN_PROGRESS) {
                    failure_deadline = std::min(
                        failure_deadline,
                        std::chrono::steady_clock::now() + std::chrono::duration<double>(1.0));
                }
            }
        }
        if (message.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
            mavlink_heartbeat_t heartbeat{};
            if (is_valid_ardupilot_heartbeat(message, &heartbeat)) {
                heartbeat_seen_ = true;
                heartbeat_mode_ = heartbeat.custom_mode;
                heartbeat_base_mode_ = heartbeat.base_mode;
                heartbeat_armed_ = is_armed_from_heartbeat(heartbeat);
                if (heartbeat_armed_) return;
            }
        }
        if (std::chrono::steady_clock::now() >= failure_deadline) break;
    }
    const std::string summary = arm_diagnostic_summary();
    std::cerr << summary << std::endl;
    throw std::runtime_error(std::string("mission setup: ") + phase +
                             " timeout; " + summary);
}

bool MissionSetup::receive_diagnostic(const std::vector<uint32_t>& message_ids,
                                      mavlink_message_t& message, double timeout_sec,
                                      const char* phase) {
    const bool received = vehicle_.recv_match(message_ids, message, timeout_sec);
    if (received && vehicle_message(vehicle_, message)) {
        write_diagnostic(std::string(phase) + " SOURCE sysid=" +
                         std::to_string(message.sysid) + " compid=" +
                         std::to_string(message.compid));
        record_diagnostic_message(message, phase);
    }
    return received;
}

void MissionSetup::write_diagnostic(const std::string& line) {
    if (diagnostic_log_.is_open()) {
        diagnostic_log_ << line << '\n';
        diagnostic_log_.flush();
    }
}

void MissionSetup::record_diagnostic_message(const mavlink_message_t& message,
                                             const char* phase) {
    if (message.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
        mavlink_heartbeat_t hb{};
        if (!is_valid_ardupilot_heartbeat(message, &hb)) return;
        heartbeat_seen_ = true;
        heartbeat_mode_ = hb.custom_mode;
        heartbeat_base_mode_ = hb.base_mode;
        heartbeat_armed_ = is_armed_from_heartbeat(hb);
        std::ostringstream line;
        line << phase << " HEARTBEAT mode=" << hb.custom_mode
             << " base_mode=" << static_cast<int>(hb.base_mode)
             << " armed=" << (heartbeat_armed_ ? "true" : "false");
        write_diagnostic(line.str());
        return;
    }
    if (message.msgid == MAVLINK_MSG_ID_STATUSTEXT) {
        mavlink_statustext_t status{};
        mavlink_msg_statustext_decode(&message, &status);
        const std::string text = trim_status_text(status);
        arm_status_texts_.push_back(text);
        write_diagnostic(std::string(phase) + " STATUSTEXT severity=" +
                         std::to_string(status.severity) + " text=" + text);
        return;
    }
    if (message.msgid == MAVLINK_MSG_ID_COMMAND_ACK) {
        mavlink_command_ack_t ack{};
        mavlink_msg_command_ack_decode(&message, &ack);
        if (ack.command == MAV_CMD_COMPONENT_ARM_DISARM) {
            std::ostringstream line;
            line << phase << " ARM_ACK result=" << mav_result_name(ack.result) << "("
                 << static_cast<int>(ack.result) << ") result_param1=unavailable"
                 << " result_param2=" << ack.result_param2;
            write_diagnostic(line.str());
        }
        return;
    }
    if (message.msgid == MAVLINK_MSG_ID_GPS_RAW_INT) {
        mavlink_gps_raw_int_t gps{};
        mavlink_msg_gps_raw_int_decode(&message, &gps);
        gps_seen_ = true; gps_fix_type_ = gps.fix_type; gps_satellites_ = gps.satellites_visible;
        write_diagnostic(std::string(phase) + " GPS_RAW_INT fix_type=" +
                         std::to_string(gps.fix_type) + " satellites=" +
                         std::to_string(gps.satellites_visible));
        return;
    }
    if (message.msgid == MAVLINK_MSG_ID_EKF_STATUS_REPORT) {
        mavlink_ekf_status_report_t ekf{};
        mavlink_msg_ekf_status_report_decode(&message, &ekf);
        ekf_seen_ = true; ekf_flags_ = ekf.flags;
        ekf_velocity_variance_ = ekf.velocity_variance;
        ekf_pos_horiz_variance_ = ekf.pos_horiz_variance;
        ekf_pos_vert_variance_ = ekf.pos_vert_variance;
        std::ostringstream line;
        line << phase << " EKF_STATUS_REPORT flags=" << ekf.flags
             << " velocity_variance=" << ekf.velocity_variance
             << " pos_horiz_variance=" << ekf.pos_horiz_variance
             << " pos_vert_variance=" << ekf.pos_vert_variance;
        write_diagnostic(line.str());
        return;
    }
    if (message.msgid == MAVLINK_MSG_ID_SYS_STATUS) {
        mavlink_sys_status_t status{};
        mavlink_msg_sys_status_decode(&message, &status);
        sys_status_seen_ = true; sensor_health_bits_ = status.onboard_control_sensors_health;
        battery_voltage_mv_ = status.voltage_battery;
        battery_remaining_ = status.battery_remaining;
        std::ostringstream line;
        line << phase << " SYS_STATUS health_bits=0x" << std::hex << sensor_health_bits_
             << std::dec << " prearm_bit="
             << ((sensor_health_bits_ & MAV_SYS_STATUS_PREARM_CHECK) ? "healthy" : "unhealthy")
             << " battery_voltage_v=" << (battery_voltage_mv_ / 1000.0)
             << " battery_remaining=" << static_cast<int>(battery_remaining_);
        write_diagnostic(line.str());
        return;
    }
    if (message.msgid == MAVLINK_MSG_ID_RC_CHANNELS) {
        mavlink_rc_channels_t rc{};
        mavlink_msg_rc_channels_decode(&message, &rc);
        rc_seen_ = true; rc_time_boot_ms_ = rc.time_boot_ms;
        rc_channel_count_ = rc.chancount; rc_rssi_ = rc.rssi;
        write_diagnostic(std::string(phase) + " RC_CHANNELS time_boot_ms=" +
                         std::to_string(rc.time_boot_ms) + " channel_count=" +
                         std::to_string(rc.chancount) + " rssi=" + std::to_string(rc.rssi));
        return;
    }
    if (message.msgid == MAVLINK_MSG_ID_HOME_POSITION) {
        home_seen_ = true;
        write_diagnostic(std::string(phase) + " HOME_POSITION prepared=true");
        return;
    }
    if (message.msgid == MAVLINK_MSG_ID_RAW_IMU) {
        mavlink_raw_imu_t imu{};
        mavlink_msg_raw_imu_decode(&message, &imu);
        raw_imu_seen_ = true;
        ++raw_imu_samples_;
        raw_imu_last_id_ = imu.id;
        raw_imu_last_xacc_ = imu.xacc;
        raw_imu_last_yacc_ = imu.yacc;
        raw_imu_last_zacc_ = imu.zacc;
        std::ostringstream line;
        line << phase << " RAW_IMU samples=" << raw_imu_samples_
             << " id=" << static_cast<int>(imu.id)
             << " xacc=" << imu.xacc << " yacc=" << imu.yacc
             << " zacc=" << imu.zacc;
        write_diagnostic(line.str());
        return;
    }
    if (message.msgid == MAVLINK_MSG_ID_HIGHRES_IMU) {
        mavlink_highres_imu_t imu{};
        mavlink_msg_highres_imu_decode(&message, &imu);
        highres_imu_seen_ = true;
        ++highres_imu_samples_;
        highres_imu_last_xacc_ = imu.xacc;
        highres_imu_last_yacc_ = imu.yacc;
        highres_imu_last_zacc_ = imu.zacc;
        std::ostringstream line;
        line << phase << " HIGHRES_IMU samples=" << highres_imu_samples_
             << " xacc=" << imu.xacc << " yacc=" << imu.yacc
             << " zacc=" << imu.zacc;
        write_diagnostic(line.str());
        return;
    }
    if (message.msgid == MAVLINK_MSG_ID_VIBRATION) {
        mavlink_vibration_t vibration{};
        mavlink_msg_vibration_decode(&message, &vibration);
        vibration_seen_ = true;
        ++vibration_samples_;
        vibration_last_x_ = vibration.vibration_x;
        vibration_last_y_ = vibration.vibration_y;
        vibration_last_z_ = vibration.vibration_z;
        vibration_last_clipping_0_ = vibration.clipping_0;
        vibration_last_clipping_1_ = vibration.clipping_1;
        vibration_last_clipping_2_ = vibration.clipping_2;
        std::ostringstream line;
        line << phase << " VIBRATION samples=" << vibration_samples_
             << " x=" << vibration.vibration_x << " y=" << vibration.vibration_y
             << " z=" << vibration.vibration_z
             << " clipping=" << vibration.clipping_0 << ","
             << vibration.clipping_1 << "," << vibration.clipping_2;
        write_diagnostic(line.str());
        return;
    }
    if (message.msgid == MAVLINK_MSG_ID_MISSION_COUNT) {
        mavlink_mission_count_t count{};
        mavlink_msg_mission_count_decode(&message, &count);
        mission_count_seen_ = true; mission_count_ = count.count;
        write_diagnostic(std::string(phase) + " MISSION_COUNT count=" +
                         std::to_string(count.count));
        return;
    }
    if (message.msgid == MAVLINK_MSG_ID_MISSION_CURRENT) {
        mavlink_mission_current_t current{};
        mavlink_msg_mission_current_decode(&message, &current);
        mission_current_seen_ = true; mission_current_ = current.seq;
        write_diagnostic(std::string(phase) + " MISSION_CURRENT seq=" +
                         std::to_string(current.seq));
        return;
    }
    if (message.msgid == MAVLINK_MSG_ID_MISSION_ACK) {
        mavlink_mission_ack_t ack{};
        mavlink_msg_mission_ack_decode(&message, &ack);
        mission_ack_seen_ = true; mission_ack_type_ = ack.type;
        write_diagnostic(std::string(phase) + " MISSION_ACK type=" +
                         std::to_string(ack.type));
    }
}

std::string MissionSetup::arm_diagnostic_summary() {
    std::ostringstream summary;
    summary << "ARM_DIAGNOSTIC_SUMMARY ack_seen=" << (arm_ack_seen_ ? "true" : "false");
    if (arm_ack_seen_) {
        summary << " result=" << mav_result_name(arm_ack_result_) << "("
                << static_cast<int>(arm_ack_result_) << ") result_param1=unavailable"
                << " result_param2=" << arm_ack_result_param2_;
    }
    summary << " heartbeat=" << (heartbeat_seen_ ? "seen" : "missing")
            << " mode=" << heartbeat_mode_ << " base_mode="
            << static_cast<int>(heartbeat_base_mode_) << " armed="
            << (heartbeat_armed_ ? "true" : "false")
            << " gps=" << (gps_seen_ ? "seen" : "missing") << " fix="
            << static_cast<int>(gps_fix_type_) << " sats="
            << static_cast<int>(gps_satellites_)
            << " ekf=" << (ekf_seen_ ? "seen" : "missing") << " flags=" << ekf_flags_
            << " ekf_vel_var=" << ekf_velocity_variance_
            << " ekf_pos_var=" << ekf_pos_horiz_variance_
            << " sys_status=" << (sys_status_seen_ ? "seen" : "missing")
            << " sensor_health=0x" << std::hex << sensor_health_bits_ << std::dec
            << " prearm_bit="
            << ((sensor_health_bits_ & MAV_SYS_STATUS_PREARM_CHECK) ? "healthy" : "unhealthy")
            << " battery_v=" << (battery_voltage_mv_ / 1000.0)
            << " battery_remaining=" << static_cast<int>(battery_remaining_)
            << " rc=" << (rc_seen_ ? "seen" : "missing")
            << " rc_time_boot_ms=" << rc_time_boot_ms_
            << " rc_channels=" << static_cast<int>(rc_channel_count_)
            << " rc_rssi=" << static_cast<int>(rc_rssi_)
            << " home_position=" << (home_seen_ ? "ready" : "missing")
            << " raw_imu=" << (raw_imu_seen_ ? "seen" : "missing")
            << " raw_imu_samples=" << raw_imu_samples_
            << " raw_imu_last_id=" << static_cast<int>(raw_imu_last_id_)
            << " raw_imu_last_acc=" << raw_imu_last_xacc_ << ","
            << raw_imu_last_yacc_ << "," << raw_imu_last_zacc_
            << " highres_imu=" << (highres_imu_seen_ ? "seen" : "missing")
            << " highres_imu_samples=" << highres_imu_samples_
            << " highres_imu_last_acc=" << highres_imu_last_xacc_ << ","
            << highres_imu_last_yacc_ << "," << highres_imu_last_zacc_
            << " vibration=" << (vibration_seen_ ? "seen" : "missing")
            << " vibration_samples=" << vibration_samples_
            << " vibration_last=" << vibration_last_x_ << ","
            << vibration_last_y_ << "," << vibration_last_z_
            << " vibration_clipping=" << vibration_last_clipping_0_ << ","
            << vibration_last_clipping_1_ << "," << vibration_last_clipping_2_
            << " mission_count=" << (mission_count_seen_ ? std::to_string(mission_count_) : "missing")
            << " mission_current=" << (mission_current_seen_ ? std::to_string(mission_current_) : "missing")
            << " mission_ack=" << (mission_ack_seen_ ? std::to_string(mission_ack_type_) : "missing");
    if (!arm_status_texts_.empty()) {
        summary << " statustext=";
        for (size_t i = 0; i < arm_status_texts_.size(); ++i) {
            if (i != 0) summary << " | ";
            summary << arm_status_texts_[i];
        }
    }
    const std::string result = summary.str();
    write_diagnostic(result);
    return result;
}

}  // namespace mission
