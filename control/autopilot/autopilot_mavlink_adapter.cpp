#include "autopilot_mavlink_adapter.hpp"

#include <arpa/inet.h>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <mutex>
#include <stdexcept>
#include <sys/socket.h>
#include <unistd.h>

namespace autopilot {
namespace {

std::string wall_clock_ms() {
    const auto now = std::chrono::system_clock::now();
    return std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(
                              now.time_since_epoch())
                              .count());
}

long long monotonic_ms() {
    const auto now = std::chrono::steady_clock::now();
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               now.time_since_epoch())
        .count();
}

std::string trace_path_for(const app::RuntimeConfig& config) {
    const char* explicit_path = config.role == app::TransportRole::CommandOwner
                                    ? std::getenv("ADAPTER_TELEMETRY_TRACE_LOG")
                                    : std::getenv("TARGET_DISTANCE_TELEMETRY_TRACE_LOG");
    if (explicit_path != nullptr && *explicit_path != '\0') return explicit_path;

    const char* directory = std::getenv("LOG_DIR");
    if (directory == nullptr || *directory == '\0') return {};
    return std::string(directory) +
           (config.role == app::TransportRole::CommandOwner
                ? "/adapter-telemetry-trace.log"
                : "/telemetry-subscriber-trace.log");
}

std::string input_trace_path_for(const app::RuntimeConfig& config) {
    const char* explicit_path = config.role == app::TransportRole::CommandOwner
                                    ? std::getenv("ADAPTER_INPUT_TRACE_LOG")
                                    : std::getenv("TARGET_DISTANCE_INPUT_TRACE_LOG");
    if (explicit_path != nullptr && *explicit_path != '\0') return explicit_path;

    const char* directory = std::getenv("LOG_DIR");
    if (directory == nullptr || *directory == '\0') return {};
    return std::string(directory) +
           (config.role == app::TransportRole::CommandOwner
                ? "/adapter-input-trace.log"
                : "/target-distance-input-trace.log");
}

const char* input_message_name(uint32_t message_id) {
    switch (message_id) {
        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: return "GLOBAL_POSITION_INT";
        case MAVLINK_MSG_ID_LOCAL_POSITION_NED: return "LOCAL_POSITION_NED";
        case MAVLINK_MSG_ID_HEARTBEAT: return "HEARTBEAT";
        case MAVLINK_MSG_ID_SYS_STATUS: return "SYS_STATUS";
        case MAVLINK_MSG_ID_ATTITUDE: return "ATTITUDE";
        default: return "MAVLINK_UNKNOWN";
    }
}

MavConnection& require_connection(const std::unique_ptr<MavConnection>& connection) {
    if (!connection) {
        throw std::invalid_argument("AutopilotMavlinkAdapter requires a connection");
    }
    return *connection;
}

safety::CommandDecision technical_failure(const safety::CommandRequest& request) {
    safety::CommandDecision decision;
    decision.type = request.type;
    decision.evaluated_at = safety::CommandRequest::Clock::now();
    decision.block_reason = safety::GateBlockReason::InvalidRequest;
    decision.mission_operation_name = request.mission_operation_name;
    return decision;
}

}  // namespace

class AutopilotMavlinkAdapter::TelemetryTrace {
public:
    explicit TelemetryTrace(const std::string& path)
        : stream_(path, std::ios::out | std::ios::app) {}

    void log(const std::string& line) {
        if (!stream_.is_open()) return;
        std::lock_guard<std::mutex> lock(mutex_);
        stream_ << "source=ADAPTER mono_ms=" << monotonic_ms()
                << " wall_ms=" << wall_clock_ms() << ' ' << line << '\n';
        stream_.flush();
    }

private:
    std::ofstream stream_;
    std::mutex mutex_;
};

class AutopilotMavlinkAdapter::TelemetryFanout {
public:
    TelemetryFanout(const std::vector<std::string>& endpoints,
                    std::shared_ptr<TelemetryTrace> trace)
        : trace_(std::move(trace)) {
        for (const auto& endpoint : endpoints) {
            if (endpoint.empty()) continue;
            if (endpoint.rfind("udp:127.0.0.1:", 0) != 0) {
                throw std::runtime_error(
                    "adapter telemetry fan-out requires udp:127.0.0.1:<port>");
            }
            const auto port_text = endpoint.substr(std::string("udp:127.0.0.1:").size());
            const int port = std::stoi(port_text);
            if (port < 1 || port > 65535) {
                throw std::runtime_error("invalid telemetry fan-out port");
            }

            const int fd = ::socket(AF_INET, SOCK_DGRAM | SOCK_CLOEXEC, 0);
            if (fd < 0) throw std::runtime_error("create telemetry fan-out socket failed");
            sockaddr_in address{};
            address.sin_family = AF_INET;
            address.sin_port = htons(static_cast<uint16_t>(port));
            inet_pton(AF_INET, "127.0.0.1", &address.sin_addr);
            destinations_.push_back({fd, address});
            if (trace_) {
                trace_->log("event=fanout_endpoint_ready endpoint=" + endpoint +
                            " bind_mode=send_only");
            }
        }
    }

    ~TelemetryFanout() {
        for (const auto& destination : destinations_) ::close(destination.first);
    }

    void publish(const mavlink_message_t& message) const {
        if (destinations_.empty()) return;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        const uint16_t length = mavlink_msg_to_send_buffer(buffer, &message);
        const bool altitude_message = message.msgid == MAVLINK_MSG_ID_ALTITUDE ||
                                      message.msgid == MAVLINK_MSG_ID_GLOBAL_POSITION_INT;
        const char* message_name = message.msgid == MAVLINK_MSG_ID_ALTITUDE
                                       ? "ALTITUDE"
                                       : "GLOBAL_POSITION_INT";
        static uint64_t altitude_send_counter = 0;
        const uint64_t counter = altitude_message ? ++altitude_send_counter : 0;
        for (const auto& destination : destinations_) {
            const ssize_t sent = ::sendto(
                destination.first, buffer, length, MSG_DONTWAIT,
                reinterpret_cast<const sockaddr*>(&destination.second),
                sizeof(destination.second));
            if (trace_ && altitude_message) {
                trace_->log(
                    "event=fanout_send endpoint=udp:127.0.0.1:" +
                    std::to_string(ntohs(destination.second.sin_port)) +
                    " message=" + message_name +
                    " mav_seq=" + std::to_string(message.seq) +
                    " altitude_send_counter=" + std::to_string(counter) +
                    " bytes=" + std::to_string(length) +
                    " send_ok=" +
                    (sent == static_cast<ssize_t>(length) ? "1" : "0"));
            }
        }
    }

private:
    std::vector<std::pair<int, sockaddr_in>> destinations_;
    std::shared_ptr<TelemetryTrace> trace_;
};

AutopilotMavlinkAdapter::AutopilotMavlinkAdapter(
    std::unique_ptr<MavConnection> connection, app::RuntimeConfig config,
    ApprovalSink approval_sink, DecisionSink decision_sink,
    ModeRequestSink mode_request_sink)
    : connection_(std::move(connection)),
      config_(std::move(config)),
      telemetry_trace_(std::make_shared<TelemetryTrace>(trace_path_for(config_))),
      input_trace_(std::make_shared<TelemetryTrace>(input_trace_path_for(config_))),
      telemetry_fanout_(std::make_unique<TelemetryFanout>(
          config_.telemetry_fanout_endpoints, telemetry_trace_)),
      approval_sink_(std::move(approval_sink)),
      decision_sink_(std::move(decision_sink)),
      mode_request_sink_(std::move(mode_request_sink)) {
    require_connection(connection_);
    if (telemetry_trace_) {
        telemetry_trace_->log(
            "event=adapter_start role=" +
            std::string(config_.role == app::TransportRole::CommandOwner
                            ? "command_owner"
                            : "telemetry_subscriber") +
            " endpoint=" + config_.endpoint);
    }
    connection_->set_message_observer([this](const mavlink_message_t& message) {
        observe_message(message);
        telemetry_fanout_->publish(message);
    });
}

AutopilotMavlinkAdapter::AutopilotMavlinkAdapter(
    std::unique_ptr<Transport> transport, app::RuntimeConfig config,
    ApprovalSink approval_sink, DecisionSink decision_sink,
    ModeRequestSink mode_request_sink)
    : AutopilotMavlinkAdapter(
          std::make_unique<MavConnection>(std::move(transport)), std::move(config),
          std::move(approval_sink), std::move(decision_sink),
          std::move(mode_request_sink)) {}

AutopilotMavlinkAdapter::~AutopilotMavlinkAdapter() = default;

void AutopilotMavlinkAdapter::observe_message(const mavlink_message_t& message,
                                              Clock::time_point now) {
    if (message.msgid != MAVLINK_MSG_ID_HEARTBEAT &&
        connection_->target_system() != 0 &&
        message.sysid != connection_->target_system()) {
        return;
    }
    const bool valid_heartbeat = is_valid_ardupilot_heartbeat(message);
    if (input_trace_) {
        input_trace_->log(
            "source=ADAPTER_MESSAGE event=ADAPTER_MESSAGE_OBSERVED message=" +
            std::string(input_message_name(message.msgid)) +
            " message_id=" + std::to_string(message.msgid) +
            " mav_seq=" + std::to_string(static_cast<unsigned>(message.seq)) +
            " system=" + std::to_string(static_cast<unsigned>(message.sysid)) +
            " component=" + std::to_string(static_cast<unsigned>(message.compid)));
    }
    if (config_.role == app::TransportRole::TelemetrySubscriber && telemetry_trace_ &&
        (message.msgid == MAVLINK_MSG_ID_ALTITUDE ||
         message.msgid == MAVLINK_MSG_ID_GLOBAL_POSITION_INT)) {
        static uint64_t altitude_receive_counter = 0;
        telemetry_trace_->log(
            "event=adapter_receive message=" +
            std::string(message.msgid == MAVLINK_MSG_ID_ALTITUDE
                            ? "ALTITUDE"
                            : "GLOBAL_POSITION_INT") +
            " mav_seq=" + std::to_string(message.seq) +
            " altitude_receive_counter=" +
            std::to_string(++altitude_receive_counter));
    }
    const auto state_update_start = Clock::now();
    update_state(message, now);
    const auto state_update_end = Clock::now();
    if (valid_heartbeat) {
        state_.last_heartbeat_observed_at = now;
        state_.last_heartbeat_state_updated_at = state_update_end;
        state_.last_heartbeat_mav_seq = message.seq;
        ++state_.heartbeat_observation_count;
        if (input_trace_) {
            const auto update_start_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                state_update_start.time_since_epoch()).count();
            const auto update_end_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                state_update_end.time_since_epoch()).count();
            input_trace_->log(
                "source=ADAPTER_HEARTBEAT event=ADAPTER_HEARTBEAT "
                "heartbeat_observed_mono_ms=" +
                std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(
                    now.time_since_epoch()).count()) +
                " heartbeat_state_updated_mono_ms=" + std::to_string(update_end_ms) +
                " state_update_start_mono_ms=" + std::to_string(update_start_ms) +
                " state_update_end_mono_ms=" + std::to_string(update_end_ms) +
                " state_update_duration_ms=" + std::to_string(update_end_ms - update_start_ms) +
                " last_heartbeat_age_ms=0 "
                "mav_seq=" + std::to_string(static_cast<unsigned>(message.seq)) +
                " system=" + std::to_string(static_cast<unsigned>(message.sysid)) +
                " component=" + std::to_string(static_cast<unsigned>(message.compid)) +
                " heartbeat_observation_count=" +
                std::to_string(state_.heartbeat_observation_count));
        }
    }
}

void AutopilotMavlinkAdapter::update_state(const mavlink_message_t& message,
                                           Clock::time_point now) {
    switch (message.msgid) {
        case MAVLINK_MSG_ID_SYS_STATUS: {
            mavlink_sys_status_t value{};
            mavlink_msg_sys_status_decode(&message, &value);
            state_.sensor_present_bits = value.onboard_control_sensors_present;
            state_.sensor_enabled_bits = value.onboard_control_sensors_enabled;
            state_.sensor_health_bits = value.onboard_control_sensors_health;
            state_.battery_percent = value.battery_remaining;
            state_.battery_current_a = value.current_battery < 0 ? -1 : value.current_battery / 100.0;
            state_.battery_voltage_v = (value.voltage_battery == 0 || value.voltage_battery == 65535)
                                           ? -1 : value.voltage_battery / 1000.0;
            state_.battery_valid = value.voltage_battery > 0 && value.voltage_battery != 65535 &&
                                   value.battery_remaining >= 0;
            state_.prearm_healthy =
                (value.onboard_control_sensors_health & MAV_SYS_STATUS_PREARM_CHECK) != 0;
            state_.have_battery = true;
            state_.have_sys_status = true;
            state_.battery_updated_at = now;
            break;
        }
        case MAVLINK_MSG_ID_BATTERY_STATUS: {
            mavlink_battery_status_t value{};
            mavlink_msg_battery_status_decode(&message, &value);
            state_.battery_percent = value.battery_remaining;
            state_.battery_voltage_v = (value.voltages[0] == 0 || value.voltages[0] == UINT16_MAX)
                                           ? -1 : value.voltages[0] / 1000.0;
            state_.battery_current_a = value.current_battery < 0 ? -1 : value.current_battery / 100.0;
            state_.battery_valid = value.voltages[0] > 0 && value.voltages[0] != UINT16_MAX &&
                                   value.battery_remaining >= 0;
            state_.have_battery = true;
            state_.battery_updated_at = now;
            break;
        }
        case MAVLINK_MSG_ID_ALTITUDE: {
            mavlink_altitude_t value{};
            mavlink_msg_altitude_decode(&message, &value);
            state_.altitude_m = value.altitude_relative;
            state_.have_altitude = true;
            state_.altitude_updated_at = now;
            break;
        }
        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
            mavlink_global_position_int_t value{};
            mavlink_msg_global_position_int_decode(&message, &value);
            state_.altitude_m = static_cast<double>(value.relative_alt) / 1000.0;
            state_.have_altitude = true;
            state_.altitude_updated_at = now;
            break;
        }
        case MAVLINK_MSG_ID_LOCAL_POSITION_NED: {
            mavlink_local_position_ned_t value{};
            mavlink_msg_local_position_ned_decode(&message, &value);
            state_.local_x_m = value.x;
            state_.local_y_m = value.y;
            state_.local_z_m = value.z;
            state_.local_vx_mps = value.vx;
            state_.local_vy_mps = value.vy;
            state_.local_vz_mps = value.vz;
            state_.have_local_position = true;
            state_.local_position_updated_at = now;
            break;
        }
        case MAVLINK_MSG_ID_GPS_RAW_INT: {
            mavlink_gps_raw_int_t value{};
            mavlink_msg_gps_raw_int_decode(&message, &value);
            state_.fix_type = value.fix_type;
            state_.satellites = value.satellites_visible;
            state_.have_gps = true;
            state_.gps_updated_at = now;
            if (value.fix_type >= 2) {
                state_.lat = value.lat / 1e7;
                state_.lon = value.lon / 1e7;
                state_.have_position = true;
            }
            break;
        }
        case MAVLINK_MSG_ID_EKF_STATUS_REPORT: {
            mavlink_ekf_status_report_t value{};
            mavlink_msg_ekf_status_report_decode(&message, &value);
            state_.ekf_flags = value.flags;
            state_.ekf_pos_horiz_variance = value.pos_horiz_variance;
            state_.ekf_pos_vert_variance = value.pos_vert_variance;
            state_.ekf_velocity_variance = value.velocity_variance;
            state_.have_ekf = true;
            state_.ekf_updated_at = now;
            break;
        }
        case MAVLINK_MSG_ID_HEARTBEAT: {
            mavlink_heartbeat_t value{};
            if (!is_valid_ardupilot_heartbeat(message, &value)) return;
            state_.system_status = value.system_status;
            state_.custom_mode = value.custom_mode;
            state_.base_mode = value.base_mode;
            state_.heartbeat_system_id = message.sysid;
            state_.heartbeat_component_id = message.compid;
            state_.armed = (value.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0;
            state_.have_armed = true;
            state_.last_heartbeat = now;
            state_.mode_updated_at = now;
            break;
        }
        case MAVLINK_MSG_ID_RC_CHANNELS: {
            mavlink_rc_channels_t value{};
            mavlink_msg_rc_channels_decode(&message, &value);
            const uint16_t channels[18] = {
                value.chan1_raw, value.chan2_raw, value.chan3_raw, value.chan4_raw,
                value.chan5_raw, value.chan6_raw, value.chan7_raw, value.chan8_raw,
                value.chan9_raw, value.chan10_raw, value.chan11_raw, value.chan12_raw,
                value.chan13_raw, value.chan14_raw, value.chan15_raw, value.chan16_raw,
                value.chan17_raw, value.chan18_raw};
            std::copy(std::begin(channels), std::end(channels), state_.rc_channels.begin());
            state_.rc_channel_count = value.chancount;
            state_.rc_rssi = value.rssi;
            state_.have_rc = value.chancount > 0;
            state_.rc_updated_at = now;
            break;
        }
        case MAVLINK_MSG_ID_ATTITUDE: {
            mavlink_attitude_t value{};
            mavlink_msg_attitude_decode(&message, &value);
            state_.roll_rad = value.roll;
            state_.pitch_rad = value.pitch;
            state_.yaw_rad = value.yaw;
            state_.roll_rate_rps = value.rollspeed;
            state_.pitch_rate_rps = value.pitchspeed;
            state_.yaw_rate_rps = value.yawspeed;
            state_.have_attitude = true;
            state_.attitude_updated_at = now;
            break;
        }
        case MAVLINK_MSG_ID_STATUSTEXT: {
            mavlink_statustext_t value{};
            mavlink_msg_statustext_decode(&message, &value);
            state_.status_text_severity = value.severity;
            state_.last_status_text.assign(value.text, sizeof(value.text));
            const auto nul = state_.last_status_text.find('\0');
            if (nul != std::string::npos) state_.last_status_text.resize(nul);
            state_.have_status_text = true;
            state_.status_text_updated_at = now;
            break;
        }
        case MAVLINK_MSG_ID_COMMAND_ACK: {
            mavlink_command_ack_t value{};
            mavlink_msg_command_ack_decode(&message, &value);
            state_.last_command_ack_command = value.command;
            state_.last_command_ack_result = value.result;
            state_.last_command_ack_param2 = value.result_param2;
            state_.have_command_ack = true;
            state_.command_ack_updated_at = now;
            break;
        }
        case MAVLINK_MSG_ID_RAW_IMU: {
            mavlink_raw_imu_t value{};
            mavlink_msg_raw_imu_decode(&message, &value);
            state_.raw_imu_id = value.id;
            state_.raw_imu_xacc = value.xacc;
            state_.raw_imu_yacc = value.yacc;
            state_.raw_imu_zacc = value.zacc;
            state_.have_raw_imu = true;
            ++state_.raw_imu_samples;
            state_.raw_imu_updated_at = now;
            break;
        }
        case MAVLINK_MSG_ID_HIGHRES_IMU: {
            mavlink_highres_imu_t value{};
            mavlink_msg_highres_imu_decode(&message, &value);
            state_.highres_xacc = value.xacc;
            state_.highres_yacc = value.yacc;
            state_.highres_zacc = value.zacc;
            state_.have_highres_imu = true;
            ++state_.highres_imu_samples;
            state_.highres_imu_updated_at = now;
            break;
        }
        case MAVLINK_MSG_ID_VIBRATION: {
            mavlink_vibration_t value{};
            mavlink_msg_vibration_decode(&message, &value);
            state_.vibration_x = value.vibration_x;
            state_.vibration_y = value.vibration_y;
            state_.vibration_z = value.vibration_z;
            state_.vibration_clipping_0 = value.clipping_0;
            state_.vibration_clipping_1 = value.clipping_1;
            state_.vibration_clipping_2 = value.clipping_2;
            state_.have_vibration = true;
            ++state_.vibration_samples;
            state_.vibration_updated_at = now;
            break;
        }
        case MAVLINK_MSG_ID_HOME_POSITION: {
            mavlink_home_position_t value{};
            mavlink_msg_home_position_decode(&message, &value);
            state_.home_lat = value.latitude / 1e7;
            state_.home_lon = value.longitude / 1e7;
            state_.home_altitude_m = value.altitude / 1000.0;
            state_.home_x_m = value.x;
            state_.home_y_m = value.y;
            state_.home_z_m = value.z;
            state_.have_home_position = true;
            state_.home_updated_at = now;
            break;
        }
        case MAVLINK_MSG_ID_MISSION_CURRENT: {
            mavlink_mission_current_t value{};
            mavlink_msg_mission_current_decode(&message, &value);
            state_.mission_current_seq = value.seq;
            state_.mission_total = value.total;
            state_.mission_state = value.mission_state;
            state_.mission_mode = value.mission_mode;
            state_.have_mission_current = true;
            state_.mission_updated_at = now;
            break;
        }
        default:
            break;
    }
}

bool AutopilotMavlinkAdapter::recv_match(const std::vector<uint32_t>& msg_ids,
                                         mavlink_message_t& out, double timeout_sec) {
    const auto process_start = Clock::now();
    const bool received = connection_->recv_match(msg_ids, out, timeout_sec);
    const auto process_end = Clock::now();
    if (received && input_trace_) {
        const uint64_t counter = ++input_dequeue_counters_[out.msgid];
        const auto process_start_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            process_start.time_since_epoch()).count();
        const auto process_end_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            process_end.time_since_epoch()).count();
        input_trace_->log(
            "source=ADAPTER_RECV_MATCH event=ADAPTER_RECV_MATCH message=" +
            std::string(input_message_name(out.msgid)) +
            " message_id=" + std::to_string(out.msgid) +
            " mav_seq=" + std::to_string(static_cast<unsigned>(out.seq)) +
            " system=" + std::to_string(static_cast<unsigned>(out.sysid)) +
            " component=" + std::to_string(static_cast<unsigned>(out.compid)) +
            " dequeue_counter=" + std::to_string(counter) +
            " pending_queue_len=" + std::to_string(connection_->pending_message_count()) +
            " timeout_sec=" + std::to_string(timeout_sec) +
            " process_start_mono_ms=" + std::to_string(process_start_ms) +
            " process_end_mono_ms=" + std::to_string(process_end_ms) +
            " process_duration_ms=" + std::to_string(process_end_ms - process_start_ms));
    } else if (input_trace_) {
        const auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            process_end.time_since_epoch()).count();
        input_trace_->log(
            "source=ADAPTER_RECV_MATCH event=ADAPTER_RECV_MATCH_TIMEOUT "
            "pending_queue_len=" + std::to_string(connection_->pending_message_count()) +
            " timeout_sec=" + std::to_string(timeout_sec) +
            " process_end_mono_ms=" + std::to_string(now_ms));
    }
    return received;
}

bool AutopilotMavlinkAdapter::poll(double timeout_sec) {
    mavlink_message_t message{};
    const auto poll_start = Clock::now();
    const bool received = recv_match({}, message, timeout_sec);
    const auto poll_end = Clock::now();
    if (input_trace_) {
        const auto start_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            poll_start.time_since_epoch()).count();
        const auto end_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            poll_end.time_since_epoch()).count();
        const auto heartbeat_age_ms = state_.last_heartbeat == Clock::time_point{}
                                          ? -1LL
                                          : std::chrono::duration_cast<std::chrono::milliseconds>(
                                                poll_end - state_.last_heartbeat).count();
        input_trace_->log(
            "source=ADAPTER_POLL event=ADAPTER_POLL "
            "poll_start_mono_ms=" + std::to_string(start_ms) +
            " poll_end_mono_ms=" + std::to_string(end_ms) +
            " poll_duration_ms=" + std::to_string(end_ms - start_ms) +
            " received=" + (received ? "1" : "0") +
            " pending_queue_len=" + std::to_string(connection_->pending_message_count()) +
            " heartbeat_age_ms=" + std::to_string(heartbeat_age_ms));
    }
    return received;
}

bool AutopilotMavlinkAdapter::wait_heartbeat(double timeout_sec,
                                             mavlink_heartbeat_t* out) {
    const auto deadline = Clock::now() + std::chrono::duration<double>(timeout_sec);
    while (Clock::now() < deadline) {
        const double remaining =
            std::chrono::duration<double>(deadline - Clock::now()).count();
        mavlink_message_t message{};
        if (!recv_match({MAVLINK_MSG_ID_HEARTBEAT}, message, remaining)) return false;

        mavlink_heartbeat_t heartbeat{};
        if (!is_valid_ardupilot_heartbeat(message, &heartbeat)) continue;
        if (out) *out = heartbeat;
        return true;
    }
    return false;
}

bool AutopilotMavlinkAdapter::request_message_interval(uint32_t message_id,
                                                       double frequency_hz) {
    if (message_id == 0 || !std::isfinite(frequency_hz) || frequency_hz <= 0.0 ||
        target_system() == 0 || target_component() == 0) {
        return false;
    }
    const int64_t interval_us = static_cast<int64_t>(1'000'000 / frequency_hz);
    if (interval_us <= 0) return false;

    mavlink_message_t message{};
    mavlink_msg_command_long_pack(
        255, 0, &message, target_system(), target_component(),
        MAV_CMD_SET_MESSAGE_INTERVAL, 0, static_cast<float>(message_id),
        static_cast<float>(interval_us), 0, 0, 0, 0, 0);
    connection_->send(message);
    return true;
}

bool AutopilotMavlinkAdapter::technical_request_is_valid(
    const safety::CommandRequest& request) const {
    if (target_system() == 0 || target_component() == 0) return false;
    if (request.type == safety::CommandType::SetMode && request.mode.empty()) return false;
    if (request.type == safety::CommandType::Takeoff &&
        (!std::isfinite(request.altitude_m) || request.altitude_m < 0.0)) return false;
    if (request.type == safety::CommandType::VelocitySetpoint &&
        (!std::isfinite(request.vx) || !std::isfinite(request.vy) ||
         !std::isfinite(request.vz) || !std::isfinite(request.yaw_rate))) return false;
    if (request.type == safety::CommandType::MissionProtocol) {
        if (request.mission_operation_name.empty()) return false;
        if (request.mission_operation == safety::MissionOperation::Count &&
            request.mission_count_value == 0) return false;
        if (request.mission_operation == safety::MissionOperation::Item &&
            (!std::isfinite(request.mission_altitude_m) || request.mission_command == 0)) {
            return false;
        }
    }
    return true;
}

safety::CommandDecision AutopilotMavlinkAdapter::send_request(
    const safety::CommandRequest& request) {
    safety::CommandDecision decision = approval_sink_
        ? approval_sink_(request)
        : safety::CommandDecision{};
    if (!approval_sink_) {
        decision.type = request.type;
        decision.allowed = true;
        decision.evaluated_at = safety::CommandRequest::Clock::now();
        decision.mission_operation_name = request.mission_operation_name;
    }
    decision.type = request.type;
    decision.mission_operation_name = request.mission_operation_name;

    bool mode_request_started = false;
    if (decision.allowed && !technical_request_is_valid(request)) {
        decision = technical_failure(request);
    } else if (decision.allowed) {
        if (request.type == safety::CommandType::SetMode && mode_request_sink_) {
            mode_request_sink_(request, false);
            mode_request_started = true;
        }
        decision.sent = serialize_and_send(request);
    }

    if (decision_sink_) decision_sink_(decision);
    if (mode_request_started && mode_request_sink_) {
        mode_request_sink_(request, decision.sent);
    }
    return decision;
}

bool AutopilotMavlinkAdapter::serialize_and_send(
    const safety::CommandRequest& request) {
    switch (request.type) {
        case safety::CommandType::SetMode:
            return drone::set_mode(*connection_, request.mode);
        case safety::CommandType::ArmDisarm:
            drone::arm_disarm(*connection_, request.arm);
            return true;
        case safety::CommandType::Takeoff:
            drone::takeoff(*connection_, request.altitude_m);
            return true;
        case safety::CommandType::VelocitySetpoint:
            if (request.body_frame) {
                drone::send_velocity_body(*connection_, request.vx, request.vy,
                                           request.vz, request.yaw_rate);
            } else {
                drone::send_velocity(*connection_, request.vx, request.vy,
                                     request.vz, request.yaw_rate);
            }
            return true;
        case safety::CommandType::Land:
            drone::land(*connection_);
            return true;
        case safety::CommandType::MissionProtocol:
            return send_mission_protocol(request);
    }
    return false;
}

bool AutopilotMavlinkAdapter::send_mission_protocol(
    const safety::CommandRequest& request) {
    constexpr uint8_t source_system = 255;
    constexpr uint8_t source_component = 0;
    mavlink_message_t message{};
    const uint8_t target_system_value = target_system();
    const uint8_t target_component_value = target_component();
    switch (request.mission_operation) {
        case safety::MissionOperation::Clear:
            mavlink_msg_mission_clear_all_pack(
                source_system, source_component, &message, target_system_value,
                target_component_value, MAV_MISSION_TYPE_MISSION);
            break;
        case safety::MissionOperation::Count:
            mavlink_msg_mission_count_pack(
                source_system, source_component, &message, target_system_value,
                target_component_value, request.mission_count_value,
                MAV_MISSION_TYPE_MISSION, 0);
            break;
        case safety::MissionOperation::Item:
            mavlink_msg_mission_item_int_pack(
                source_system, source_component, &message, target_system_value,
                target_component_value, request.mission_seq, request.mission_frame,
                request.mission_command, request.mission_current, 1, 0, 0, 0,
                0, request.mission_x, request.mission_y,
                request.mission_altitude_m, MAV_MISSION_TYPE_MISSION);
            break;
        case safety::MissionOperation::RequestList:
            mavlink_msg_mission_request_list_pack(
                source_system, source_component, &message, target_system_value,
                target_component_value, MAV_MISSION_TYPE_MISSION);
            break;
        case safety::MissionOperation::RequestItem:
            mavlink_msg_mission_request_int_pack(
                source_system, source_component, &message, target_system_value,
                target_component_value, request.mission_seq,
                MAV_MISSION_TYPE_MISSION);
            break;
        case safety::MissionOperation::SetCurrent:
            mavlink_msg_mission_set_current_pack(
                source_system, source_component, &message, target_system_value,
                target_component_value, request.mission_seq);
            break;
    }
    connection_->send(message);
    return true;
}

safety::CommandDecision AutopilotMavlinkAdapter::set_mode(const std::string& mode) {
    const auto now = safety::CommandRequest::Clock::now();
    return send_request(safety::CommandRequest::set_mode(
        mode, now, now + std::chrono::seconds(5)));
}

safety::CommandDecision AutopilotMavlinkAdapter::arm_disarm(bool arm) {
    return send_request(safety::CommandRequest::arm_disarm(arm));
}

safety::CommandDecision AutopilotMavlinkAdapter::takeoff(double altitude_m) {
    return send_request(safety::CommandRequest::takeoff(altitude_m));
}

safety::CommandDecision AutopilotMavlinkAdapter::send_velocity(
    double vx, double vy, double vz, double yaw_rate) {
    return send_request(safety::CommandRequest::velocity_local(vx, vy, vz, yaw_rate));
}

safety::CommandDecision AutopilotMavlinkAdapter::send_velocity_body(
    double vx, double vy, double vz, double yaw_rate) {
    return send_request(safety::CommandRequest::velocity_body(vx, vy, vz, yaw_rate));
}

safety::CommandDecision AutopilotMavlinkAdapter::send_zero_velocity() {
    return send_request(safety::CommandRequest::zero_velocity());
}

safety::CommandDecision AutopilotMavlinkAdapter::land() {
    return send_request(safety::CommandRequest::land());
}

safety::CommandDecision AutopilotMavlinkAdapter::send_mission_clear() {
    return send_request(safety::CommandRequest::mission_clear());
}

safety::CommandDecision AutopilotMavlinkAdapter::send_mission_count(uint16_t count) {
    return send_request(safety::CommandRequest::mission_count(count));
}

safety::CommandDecision AutopilotMavlinkAdapter::send_mission_item(
    uint16_t seq, uint16_t command, uint8_t frame, int32_t x, int32_t y,
    float altitude, uint8_t current) {
    return send_request(safety::CommandRequest::mission_item(
        seq, command, frame, x, y, altitude, current));
}

safety::CommandDecision AutopilotMavlinkAdapter::send_mission_request_list() {
    return send_request(safety::CommandRequest::mission_request_list());
}

safety::CommandDecision AutopilotMavlinkAdapter::send_mission_request_item(uint16_t seq) {
    return send_request(safety::CommandRequest::mission_request_item(seq));
}

safety::CommandDecision AutopilotMavlinkAdapter::send_mission_set_current(uint16_t seq) {
    return send_request(safety::CommandRequest::mission_set_current(seq));
}

}  // namespace autopilot
