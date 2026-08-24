#include "drone_lib.hpp"

#include <array>
#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <mutex>
#include <stdexcept>

namespace {
// Identity we present to the flight controller as the sender of our
// commands -- 255/0 is pymavlink's default GCS-like source address.
constexpr uint8_t kSourceSystem = 255;
constexpr uint8_t kSourceComponent = 0;
}  // namespace

namespace {

std::string input_trace_path() {
    const char* path = std::getenv("MAVLINK_INPUT_TRACE_LOG");
    return path == nullptr ? std::string{} : std::string(path);
}

std::string input_trace_wall_ms() {
    const auto now = std::chrono::system_clock::now();
    return std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(
                              now.time_since_epoch())
                              .count());
}

long long input_trace_monotonic_ms() {
    const auto now = std::chrono::steady_clock::now();
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               now.time_since_epoch())
        .count();
}

const char* traced_message_name(uint32_t message_id) {
    switch (message_id) {
        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: return "GLOBAL_POSITION_INT";
        case MAVLINK_MSG_ID_LOCAL_POSITION_NED: return "LOCAL_POSITION_NED";
        case MAVLINK_MSG_ID_HEARTBEAT: return "HEARTBEAT";
        case MAVLINK_MSG_ID_SYS_STATUS: return "SYS_STATUS";
        case MAVLINK_MSG_ID_ATTITUDE: return "ATTITUDE";
        default: return nullptr;
    }
}

}  // namespace

class MavConnection::InputTrace {
public:
    InputTrace() : stream_(input_trace_path(), std::ios::out | std::ios::app) {}

    void transport_read(std::size_t bytes, int timeout_ms,
                        long long read_start_mono_ms,
                        long long read_end_mono_ms) {
        if (!stream_.is_open()) return;
        std::lock_guard<std::mutex> lock(mutex_);
        stream_ << "source=MAVLINK_RAW_RX mono_ms=" << read_end_mono_ms
                << " wall_ms=" << input_trace_wall_ms()
                << " event=MAVLINK_RAW_RX read_counter=" << ++read_counter_
                << " bytes=" << bytes
                << " timeout_ms=" << timeout_ms
                << " raw_bytes=" << bytes
                << " read_start_mono_ms=" << read_start_mono_ms
                << " read_end_mono_ms=" << read_end_mono_ms
                << " read_duration_ms=" << (read_end_mono_ms - read_start_mono_ms)
                << " transport_read_unit=stream_chunk\n";
        stream_.flush();
    }

    void transport_timeout(int timeout_ms, long long timeout_mono_ms) {
        if (!stream_.is_open()) return;
        std::lock_guard<std::mutex> lock(mutex_);
        stream_ << "source=MAVLINK_RAW_RX mono_ms=" << timeout_mono_ms
                << " wall_ms=" << input_trace_wall_ms()
                << " event=MAVLINK_RAW_TIMEOUT timeout_ms=" << timeout_ms
                << " raw_bytes=0"
                << " transport_read_unit=stream_chunk\n";
        stream_.flush();
    }

    void message_parsed(const mavlink_message_t& message,
                        long long parse_start_mono_ms,
                        long long parse_end_mono_ms) {
        const char* name = traced_message_name(message.msgid);
        if (name == nullptr) name = "MAVLINK_UNKNOWN";
        if (!stream_.is_open()) return;
        std::lock_guard<std::mutex> lock(mutex_);
        const uint64_t counter = ++message_counters_[message.msgid];
        stream_ << "source=MAVLINK_PARSE mono_ms=" << parse_end_mono_ms
                << " wall_ms=" << input_trace_wall_ms()
                << " event=MAVLINK_PARSE_COMPLETE message=" << name
                << " message_id=" << message.msgid
                << " mav_seq=" << static_cast<unsigned>(message.seq)
                << " system=" << static_cast<unsigned>(message.sysid)
                << " component=" << static_cast<unsigned>(message.compid)
                << " parse_start_mono_ms=" << parse_start_mono_ms
                << " parse_end_mono_ms=" << parse_end_mono_ms
                << " parse_duration_ms=" << (parse_end_mono_ms - parse_start_mono_ms)
                << " message_counter=" << counter << '\n';
        stream_.flush();
    }

private:
    std::ofstream stream_;
    std::mutex mutex_;
    uint64_t read_counter_ = 0;
    std::map<uint32_t, uint64_t> message_counters_;
};

MavConnection::MavConnection(std::unique_ptr<Transport> transport)
    : transport_(std::move(transport)), input_trace_(std::make_unique<InputTrace>()) {}

MavConnection::~MavConnection() = default;

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

    // MAVProxy can batch multiple MAVLink frames in one UDP datagram. A
    // 512-byte recvfrom buffer truncates such datagrams, which can preserve
    // GPS/EKF frames while dropping a later HEARTBEAT and falsely trip the
    // heartbeat safety gate. Keep the full UDP datagram for the parser.
    std::array<uint8_t, 64 * 1024> buf{};
    bool found = false;
    auto process_bytes = [&](const uint8_t* bytes, size_t count,
                             long long parse_start_mono_ms) {
        for (size_t i = 0; i < count; ++i) {
            mavlink_message_t msg;
            mavlink_status_t status{};
            if (mavlink_parse_char(MAVLINK_COMM_0, bytes[i], &msg, &status)) {
                if (input_trace_) {
                    input_trace_->message_parsed(
                        msg, parse_start_mono_ms, input_trace_monotonic_ms());
                }
                if (target_system_ == 0 && is_valid_ardupilot_heartbeat(msg)) {
                    target_system_ = msg.sysid;
                    target_component_ = msg.compid;
                }

                if (message_observer_) message_observer_(msg);

                if (!found && wanted(msg.msgid)) {
                    out = msg;
                    found = true;
                } else {
                    // Preserve complete frames for callers that use a
                    // different filter, but bound the queue. Adapter state
                    // has already consumed every frame through the observer;
                    // retaining an unbounded telemetry backlog can starve
                    // fresh UDP reads and make freshness look stale.
                    constexpr std::size_t kMaxPendingMessages = 1024;
                    if (pending_messages_.size() >= kMaxPendingMessages) {
                        pending_messages_.pop_front();
                    }
                    pending_messages_.push_back(msg);
                }
            }
        }
    };

    // The adapter's observer is the authoritative state consumer. Drain a
    // bounded number of already-readable datagrams before consulting old
    // filtered messages, so a high-rate telemetry stream cannot leave the
    // AutopilotState timestamps behind while recv_match() returns backlog.
    if (message_observer_) {
        constexpr int kMaxImmediateDatagrams = 64;
        for (int i = 0; i < kMaxImmediateDatagrams; ++i) {
            const auto read_start = clock::now();
            const size_t n = transport_->read_bytes(buf.data(), buf.size(), 0);
            const auto read_end = clock::now();
            if (n == 0) break;
            if (input_trace_) {
                input_trace_->transport_read(
                    n, 0,
                    std::chrono::duration_cast<std::chrono::milliseconds>(
                        read_start.time_since_epoch()).count(),
                    std::chrono::duration_cast<std::chrono::milliseconds>(
                        read_end.time_since_epoch()).count());
            }
            process_bytes(
                buf.data(), n,
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    read_end.time_since_epoch()).count());
        }
        if (found) return true;
    }

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

    while (true) {
        auto now = clock::now();
        if (now >= deadline) return false;
        int remaining_ms = static_cast<int>(
            std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now).count());
        remaining_ms = std::max(remaining_ms, 0);
        const auto read_start = clock::now();
        size_t n = transport_->read_bytes(buf.data(), buf.size(), remaining_ms);
        const auto read_end = clock::now();
        if (n > 0) {
            if (input_trace_) {
                input_trace_->transport_read(
                    n, remaining_ms,
                    std::chrono::duration_cast<std::chrono::milliseconds>(
                        read_start.time_since_epoch()).count(),
                    std::chrono::duration_cast<std::chrono::milliseconds>(
                        read_end.time_since_epoch()).count());
            }
        } else if (remaining_ms >= 1000) {
            if (input_trace_) {
                input_trace_->transport_timeout(
                    remaining_ms,
                    std::chrono::duration_cast<std::chrono::milliseconds>(
                        read_end.time_since_epoch()).count());
            }
        }
        found = false;
        process_bytes(
            buf.data(), n,
            std::chrono::duration_cast<std::chrono::milliseconds>(
                read_end.time_since_epoch()).count());
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
    ++tx_packet_count_;
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
