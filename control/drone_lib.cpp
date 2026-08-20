#include "drone_lib.hpp"

#include <algorithm>
#include <chrono>
#include <iostream>
#include <stdexcept>

namespace {
// Identity we present to the flight controller as the sender of our
// commands -- 255/0 is pymavlink's default GCS-like source address.
constexpr uint8_t kSourceSystem = 255;
constexpr uint8_t kSourceComponent = 0;
}  // namespace

MavConnection::MavConnection(std::unique_ptr<Transport> transport)
    : transport_(std::move(transport)) {}

bool is_valid_ardupilot_heartbeat(const mavlink_message_t& message,
                                  mavlink_heartbeat_t* decoded) {
    if (message.msgid != MAVLINK_MSG_ID_HEARTBEAT || message.sysid != 1 ||
        message.compid != MAV_COMP_ID_AUTOPILOT1) {
        return false;
    }

    mavlink_heartbeat_t heartbeat{};
    mavlink_msg_heartbeat_decode(&message, &heartbeat);
    if (heartbeat.autopilot != MAV_AUTOPILOT_ARDUPILOTMEGA) return false;
    if (decoded != nullptr) *decoded = heartbeat;
    return true;
}

bool MavConnection::recv_match(const std::vector<uint32_t>& msg_ids, mavlink_message_t& out,
                                double timeout_sec) {
    using clock = std::chrono::steady_clock;
    auto deadline = clock::now() + std::chrono::duration<double>(timeout_sec);

    auto wanted = [&msg_ids](uint32_t msgid) {
        return msg_ids.empty() ||
               std::find(msg_ids.begin(), msg_ids.end(), msgid) != msg_ids.end();
    };

    // Keep unmatched messages in the queue. A single transport read commonly
    // contains telemetry, heartbeat, and status packets together; dropping a
    // packet here can make the health watchdog falsely report a lost link.
    for (auto it = pending_messages_.begin(); it != pending_messages_.end(); ++it) {
        if (wanted(it->msgid)) {
            out = *it;
            pending_messages_.erase(it);
            return true;
        }
    }

    uint8_t buf[512];
    while (true) {
        auto now = clock::now();
        if (now >= deadline) return false;
        int remaining_ms = static_cast<int>(
            std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now).count());
        remaining_ms = std::max(remaining_ms, 0);
        bool found = false;
        size_t n = transport_->read_bytes(buf, sizeof(buf), remaining_ms);
        for (size_t i = 0; i < n; ++i) {
            mavlink_message_t msg;
            mavlink_status_t status;
            if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status)) {
                if (target_system_ == 0 && is_valid_ardupilot_heartbeat(msg)) {
                    target_system_ = msg.sysid;
                    target_component_ = msg.compid;
                }

                if (!found && wanted(msg.msgid)) {
                    out = msg;
                    found = true;
                } else {
                    // Preserve every complete frame, including frames before
                    // the selected one. A later recv_match() may use a
                    // different message filter.
                    pending_messages_.push_back(msg);
                }
            }
        }
        if (found) return true;
    }
}

bool MavConnection::wait_heartbeat(double timeout_sec, mavlink_heartbeat_t* out) {
    using clock = std::chrono::steady_clock;
    const auto deadline = clock::now() + std::chrono::duration<double>(timeout_sec);

    while (clock::now() < deadline) {
        const double remaining =
            std::chrono::duration<double>(deadline - clock::now()).count();
        mavlink_message_t msg;
        if (!recv_match({MAVLINK_MSG_ID_HEARTBEAT}, msg, remaining)) return false;

        mavlink_heartbeat_t heartbeat{};
        if (!is_valid_ardupilot_heartbeat(msg, &heartbeat)) continue;
        if (msg.sysid != target_system_ || msg.compid != target_component_) continue;

        if (out) *out = heartbeat;
        return true;
    }
    return false;
}

void MavConnection::send(const mavlink_message_t& msg) {
    if (target_system_ == 0 || target_component_ == 0) {
        throw std::runtime_error("MAVLink target is unknown; wait for an autopilot heartbeat first");
    }
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    transport_->write_bytes(buf, len);
}

std::unique_ptr<MavConnection> open_connection(const std::string& address, int baud) {
    return std::make_unique<MavConnection>(open_transport(address, baud));
}

const std::map<std::string, uint32_t>& copter_mode_mapping() {
    // ArduCopter Mode::Number, in ArduPilot/ArduCopter/mode.h.
    static const std::map<std::string, uint32_t> mapping = {
        {"STABILIZE", 0},   {"ACRO", 1},         {"ALT_HOLD", 2},   {"AUTO", 3},
        {"GUIDED", 4},      {"LOITER", 5},       {"RTL", 6},        {"CIRCLE", 7},
        {"LAND", 9},        {"DRIFT", 11},       {"SPORT", 13},     {"FLIP", 14},
        {"AUTOTUNE", 15},   {"POSHOLD", 16},     {"BRAKE", 17},     {"THROW", 18},
        {"AVOID_ADSB", 19}, {"GUIDED_NOGPS", 20}, {"SMART_RTL", 21}, {"FLOWHOLD", 22},
        {"FOLLOW", 23},     {"ZIGZAG", 24},      {"SYSTEMID", 25},  {"AUTOROTATE", 26},
        {"AUTO_RTL", 27},
    };
    return mapping;
}

bool is_armed_from_heartbeat(const mavlink_heartbeat_t& hb) {
    return (hb.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0;
}

namespace drone {

namespace {
std::unique_ptr<MavConnection> g_master;
}

YamlValue load_mavlink_settings() { return ::load_mavlink_settings(); }

YamlValue load_safety_settings() { return ::load_safety_settings(); }

YamlValue load_port_settings() { return ::load_port_settings(); }

YamlValue load_rate_settings() { return ::load_rate_settings(); }

MavConnection& connect(const std::string& address, double heartbeat_timeout) {
    g_master = open_connection(address);
    std::cout << "MAVLink heartbeat 대기 중: " << address << " (" << heartbeat_timeout
              << "초 제한)" << std::endl;
    if (!g_master->wait_heartbeat(heartbeat_timeout)) {
        throw std::runtime_error(
            "MAVLink heartbeat를 받지 못했습니다. "
            "Mission Planner SITL의 MAVLink UDP 출력을 Jetson 192.168.0.196:14561로 추가하세요.");
    }
    std::cout << "연결됨. system = " << static_cast<int>(g_master->target_system()) << std::endl;
    return *g_master;
}

MavConnection& require_connection() {
    if (!g_master) {
        throw std::runtime_error("드론 연결이 없습니다. 먼저 connect(address)를 호출하세요.");
    }
    return *g_master;
}

bool set_mode(MavConnection& vehicle, const std::string& mode) {
    const auto& mapping = copter_mode_mapping();
    auto it = mapping.find(mode);
    if (it == mapping.end()) {
        std::cout << "지원하지 않는 모드입니다: " << mode << std::endl;
        return false;
    }

    mavlink_message_t msg;
    mavlink_msg_set_mode_pack(kSourceSystem, kSourceComponent, &msg, vehicle.target_system(),
                               MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, it->second);
    vehicle.send(msg);
    return true;
}

void arm_disarm(MavConnection& vehicle, bool arm) {
    mavlink_message_t msg;
    mavlink_msg_command_long_pack(kSourceSystem, kSourceComponent, &msg, vehicle.target_system(),
                                   vehicle.target_component(), MAV_CMD_COMPONENT_ARM_DISARM, 0,
                                   arm ? 1 : 0, 0, 0, 0, 0, 0, 0);
    vehicle.send(msg);
}

void takeoff(MavConnection& vehicle, double altitude) {
    mavlink_message_t msg;
    mavlink_msg_command_long_pack(kSourceSystem, kSourceComponent, &msg, vehicle.target_system(),
                                   vehicle.target_component(), MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0,
                                   0, 0, static_cast<float>(altitude));
    vehicle.send(msg);
}

void send_velocity(MavConnection& vehicle, double vx, double vy, double vz, double yaw_rate) {
    mavlink_message_t msg;
    // Mask selects velocity (vx,vy,vz) and yaw_rate only, ignoring
    // position/acceleration/yaw fields. Bit 11 (YAW_RATE_IGNORE) must stay
    // 0 here - it was previously 1, which silently dropped every yaw_rate
    // this function was ever called with.
    const uint16_t type_mask = 0b0000011111000111;
    mavlink_msg_set_position_target_local_ned_pack(
        kSourceSystem, kSourceComponent, &msg, 0, vehicle.target_system(),
        vehicle.target_component(), MAV_FRAME_LOCAL_NED, type_mask, 0, 0, 0,
        static_cast<float>(vx), static_cast<float>(vy), static_cast<float>(vz), 0, 0, 0, 0,
        static_cast<float>(yaw_rate));
    vehicle.send(msg);
}

void send_velocity_body(MavConnection& vehicle, double vx, double vy, double vz,
                        double yaw_rate) {
    mavlink_message_t msg;
    // Same mask/fix as send_velocity() above - see its comment.
    const uint16_t type_mask = 0b0000011111000111;
    mavlink_msg_set_position_target_local_ned_pack(
        kSourceSystem, kSourceComponent, &msg, 0, vehicle.target_system(),
        vehicle.target_component(), MAV_FRAME_BODY_OFFSET_NED, type_mask, 0, 0, 0,
        static_cast<float>(vx), static_cast<float>(vy), static_cast<float>(vz), 0, 0, 0, 0,
        static_cast<float>(yaw_rate));
    vehicle.send(msg);
}

void land(MavConnection& vehicle) {
    mavlink_message_t msg;
    mavlink_msg_command_long_pack(kSourceSystem, kSourceComponent, &msg, vehicle.target_system(),
                                   vehicle.target_component(), MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0,
                                   0, 0);
    vehicle.send(msg);
}

bool set_mode(const std::string& mode) { return set_mode(require_connection(), mode); }

void arm_disarm(bool arm) { arm_disarm(require_connection(), arm); }

void takeoff(double altitude) { takeoff(require_connection(), altitude); }

void send_velocity(double vx, double vy, double vz, double yaw_rate) {
    send_velocity(require_connection(), vx, vy, vz, yaw_rate);
}

void send_velocity_body(double vx, double vy, double vz, double yaw_rate) {
    send_velocity_body(require_connection(), vx, vy, vz, yaw_rate);
}

void land() { land(require_connection()); }

}  // namespace drone
