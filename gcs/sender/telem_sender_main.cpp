// Read-only onboard telemetry publisher.
//
// The external MAVLink router owns Pixhawk serial. This process subscribes to
// its loopback UDP telemetry output, builds a small JSON snapshot, and sends
// that snapshot to the laptop using the existing FEC transport. It never
// calls MavConnection::send(), requests message intervals, or opens a serial
// device.

#include <unistd.h>
#include <limits.h>

#include <algorithm>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "drone_lib.hpp"
#include "target_link.hpp"
#include "fec_encoder.hpp"
#include "udp_sender.hpp"
#include "yaml_settings.hpp"

namespace {

volatile std::sig_atomic_t g_stop = 0;
void on_signal(int) { g_stop = 1; }

using Clock = std::chrono::steady_clock;

struct Args {
    std::string telemetry_endpoint;
    double duration_sec = 0.0;
};

struct Seen {
    bool value = false;
    Clock::time_point at{};
};

struct VehicleState {
    std::string mode = "UNKNOWN";
    bool armed = false;
    bool heartbeat_valid = false;
    bool gps_valid = false;
    bool ekf_valid = false;
    bool battery_valid = false;
    int gps_fix = 0;
    int gps_satellites = 0;
    int battery_percent = -1;
    double battery_voltage = 0.0;
    double altitude_m = 0.0;
    double actual_vx = 0.0;
    double actual_vy = 0.0;
    double actual_vz = 0.0;
    bool actual_velocity_valid = false;
    Seen heartbeat;
    Seen gps;
    Seen ekf;
    Seen battery;
    Seen altitude;
};

YamlValue load_gcs_yaml() {
    char exe_path[PATH_MAX];
    ssize_t len = readlink("/proc/self/exe", exe_path, sizeof(exe_path) - 1);
    std::string exe_dir = ".";
    if (len != -1) {
        exe_path[len] = '\0';
        std::string full(exe_path);
        exe_dir = full.substr(0, full.find_last_of('/'));
    }
    std::vector<std::string> candidates;
    if (const char* settings_dir = std::getenv("ASTRODRONE_SETTINGS_DIR");
        settings_dir && *settings_dir) {
        candidates.emplace_back(std::string(settings_dir) + "/gcs.yaml");
    }
    if (const char* repo = std::getenv("ASTRODRONE_REPO"); repo && *repo) {
        candidates.emplace_back(std::string(repo) + "/setting/gcs.yaml");
    }
    const std::string relative_candidates[] = {
        exe_dir + "/../../setting/gcs.yaml",
        exe_dir + "/../setting/gcs.yaml",
    };
    for (const auto& path : relative_candidates) candidates.push_back(path);
    for (const auto& path : candidates) {
        std::ifstream probe(path);
        if (probe.good()) return parse_yaml_file(path);
    }
    throw std::runtime_error("gcs.yaml not found relative to telem_sender");
}

bool is_loopback_udp(const std::string& endpoint) {
    return endpoint.rfind("udp:127.0.0.1:", 0) == 0;
}

Args parse_args(int argc, char** argv) {
    Args args;
    const char* env_endpoint = std::getenv("MAVPROXY_TELEMETRY_ENDPOINT");
    args.telemetry_endpoint = env_endpoint && *env_endpoint
                                  ? env_endpoint
                                  : "udp:127.0.0.1:14551";
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if ((arg == "--telemetry-endpoint" || arg == "--connect") && i + 1 < argc) {
            args.telemetry_endpoint = argv[++i];
        } else if (arg == "--duration" && i + 1 < argc) {
            args.duration_sec = std::stod(argv[++i]);
        } else if (arg == "--help") {
            std::cout << "사용법: telem_sender [--telemetry-endpoint udp:127.0.0.1:14551] [--duration 초]\n";
            std::exit(0);
        } else {
            throw std::runtime_error("알 수 없는 옵션: " + arg);
        }
    }
    if (!is_loopback_udp(args.telemetry_endpoint)) {
        throw std::runtime_error(
            "telem_sender는 외부 router의 loopback UDP telemetry만 허용합니다: " +
            args.telemetry_endpoint);
    }
    return args;
}

double age_ms(const Seen& seen, Clock::time_point now) {
    if (!seen.value) return -1.0;
    return std::chrono::duration<double, std::milli>(now - seen.at).count();
}

void mark(Seen& seen, Clock::time_point now) {
    seen.value = true;
    seen.at = now;
}

bool is_vehicle_message(const mavlink_message_t& message) {
    return message.sysid == 1 && message.compid == MAV_COMP_ID_AUTOPILOT1;
}

void consume_mavlink(const mavlink_message_t& message, VehicleState& state,
                     Clock::time_point now) {
    if (!is_vehicle_message(message)) return;

    switch (message.msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT: {
            mavlink_heartbeat_t heartbeat{};
            if (!is_valid_ardupilot_heartbeat(message, &heartbeat)) return;
            for (const auto& entry : copter_mode_mapping()) {
                if (entry.second == heartbeat.custom_mode) {
                    state.mode = entry.first;
                    break;
                }
            }
            state.armed = is_armed_from_heartbeat(heartbeat);
            state.heartbeat_valid = true;
            mark(state.heartbeat, now);
            break;
        }
        case MAVLINK_MSG_ID_GPS_RAW_INT: {
            mavlink_gps_raw_int_t gps{};
            mavlink_msg_gps_raw_int_decode(&message, &gps);
            state.gps_fix = gps.fix_type;
            state.gps_satellites = gps.satellites_visible == UINT8_MAX
                                        ? 0
                                        : gps.satellites_visible;
            state.gps_valid = gps.fix_type >= 3;
            mark(state.gps, now);
            break;
        }
        case MAVLINK_MSG_ID_EKF_STATUS_REPORT: {
            mavlink_ekf_status_report_t ekf{};
            mavlink_msg_ekf_status_report_decode(&message, &ekf);
            const uint16_t required = ESTIMATOR_ATTITUDE | ESTIMATOR_VELOCITY_HORIZ |
                                      ESTIMATOR_POS_HORIZ_REL;
            state.ekf_valid = (ekf.flags & required) == required;
            mark(state.ekf, now);
            break;
        }
        case MAVLINK_MSG_ID_SYS_STATUS: {
            mavlink_sys_status_t sys{};
            mavlink_msg_sys_status_decode(&message, &sys);
            state.battery_voltage = sys.voltage_battery == UINT16_MAX
                                        ? 0.0
                                        : static_cast<double>(sys.voltage_battery) / 1000.0;
            state.battery_percent = sys.battery_remaining == INT8_MAX
                                        ? -1
                                        : static_cast<int>(sys.battery_remaining);
            state.battery_valid = state.battery_voltage > 0.0 &&
                                  state.battery_percent >= 0;
            mark(state.battery, now);
            break;
        }
        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
            mavlink_global_position_int_t position{};
            mavlink_msg_global_position_int_decode(&message, &position);
            state.altitude_m = static_cast<double>(position.relative_alt) / 1000.0;
            state.actual_vx = static_cast<double>(position.vx) / 100.0;
            state.actual_vy = static_cast<double>(position.vy) / 100.0;
            state.actual_vz = static_cast<double>(position.vz) / 100.0;
            state.actual_velocity_valid = true;
            mark(state.altitude, now);
            break;
        }
        case MAVLINK_MSG_ID_LOCAL_POSITION_NED: {
            mavlink_local_position_ned_t position{};
            mavlink_msg_local_position_ned_decode(&message, &position);
            state.altitude_m = std::max(0.0, -static_cast<double>(position.z));
            state.actual_vx = position.vx;
            state.actual_vy = position.vy;
            state.actual_vz = position.vz;
            state.actual_velocity_valid = true;
            mark(state.altitude, now);
            break;
        }
        default:
            break;
    }
}

std::string json_escape(const std::string& value) {
    std::ostringstream out;
    for (const unsigned char ch : value) {
        if (ch == '\\') out << "\\\\";
        else if (ch == '"') out << "\\\"";
        else if (ch == '\n') out << "\\n";
        else out << ch;
    }
    return out.str();
}

std::string nullable_float(float value) {
    if (!std::isfinite(value)) return "null";
    std::ostringstream out;
    out << std::fixed << std::setprecision(3) << value;
    return out.str();
}

std::string make_json(uint32_t seq, const VehicleState& vehicle,
                      const TargetRangeMsg& target, bool have_target,
                      const GcsCommandStateMsg& command, bool have_command,
                      Clock::time_point now) {
    const auto unix_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    const double heartbeat_age = age_ms(vehicle.heartbeat, now);
    const bool telemetry_fresh = heartbeat_age >= 0.0 && heartbeat_age <= 3000.0;
    const bool target_found = have_target && target.found != 0;
    const bool target_confirmed = target_found && target.confirmed != 0;
    const std::string state_name = have_command && command.state[0] != '\0'
                                       ? command.state
                                       : vehicle.mode;
    std::ostringstream out;
    out << "{\"protocol\":\"astrodrone-gcs-v1\",\"seq\":" << seq
        << ",\"timestamp_ms\":" << unix_ms
        << ",\"state\":\"" << json_escape(state_name)
        << "\",\"mode\":\"" << json_escape(vehicle.mode)
        << "\",\"armed\":" << (vehicle.armed ? "true" : "false")
        << ",\"altitude_m\":" << vehicle.altitude_m
        << ",\"heartbeat_age_ms\":" << heartbeat_age
        << ",\"gps_fix\":" << vehicle.gps_fix
        << ",\"gps_satellites\":" << vehicle.gps_satellites
        << ",\"ekf_healthy\":" << (vehicle.ekf_valid ? "true" : "false")
        << ",\"battery_voltage_v\":" << vehicle.battery_voltage
        << ",\"battery_percent\":" << vehicle.battery_percent
        << ",\"telemetry_fresh\":" << (telemetry_fresh ? "true" : "false")
        << ",\"target_found\":" << (target_found ? "true" : "false")
        << ",\"target_confirmed\":" << (target_confirmed ? "true" : "false")
        << ",\"target_x_px\":" << (target_found ? std::to_string(target.x_px) : "null")
        << ",\"target_y_px\":" << (target_found ? std::to_string(target.y_px) : "null")
        << ",\"bbox_x_px\":" << (target_found ? std::to_string(target.bbox_x_px) : "null")
        << ",\"bbox_y_px\":" << (target_found ? std::to_string(target.bbox_y_px) : "null")
        << ",\"bbox_width_px\":" << (target_found ? std::to_string(target.bbox_width_px) : "null")
        << ",\"bbox_height_px\":" << (target_found ? std::to_string(target.bbox_height_px) : "null")
        << ",\"target_width_px\":" << (target_found ? std::to_string(target.bbox_width_px) : "null")
        << ",\"target_height_px\":" << (target_found ? std::to_string(target.bbox_height_px) : "null")
        << ",\"target_confidence\":" << (target_found ? nullable_float(target.target_confidence) : "null")
        << ",\"target_class\":\"" << (target_found ? json_escape(target.class_name) : "") << "\""
        << ",\"target_distance_m\":" << (target_found ? std::to_string(target.distance_m) : "null")
        << ",\"frame_sequence\":" << (target_found ? std::to_string(target.frame_sequence) : "null")
        << ",\"frame_timestamp_ns\":" << (target_found ? std::to_string(target.frame_timestamp_ns) : "null")
        << ",\"frame_width\":" << (target_found ? std::to_string(target.frame_width) : "null")
        << ",\"frame_height\":" << (target_found ? std::to_string(target.frame_height) : "null")
        << ",\"frame_source\":\"" << (target_found ? json_escape(target.source) : "") << "\""
        // Command setpoints come only from the local non-MAVLink command-state
        // publisher. They are never inferred from telemetry or re-sent to the
        // vehicle.
        << ",\"vx\":" << (have_command ? std::to_string(command.vx) : "null")
        << ",\"vy\":" << (have_command ? std::to_string(command.vy) : "null")
        << ",\"vz\":" << (have_command ? std::to_string(command.vz) : "null")
        << ",\"command_path_angle_rad\":"
        << (have_command ? std::to_string(command.path_angle_rad) : "null")
        << ",\"command_allowed\":"
        << (have_command && command.allowed ? "true" : "false")
        << ",\"command_state\":\""
        << (have_command ? json_escape(command.state) : "") << "\""
        << ",\"command_safety_reason\":\""
        << (have_command ? json_escape(command.safety_reason) : "") << "\""
        << ",\"actual_vx\":" << (vehicle.actual_velocity_valid ? std::to_string(vehicle.actual_vx) : "null")
        << ",\"actual_vy\":" << (vehicle.actual_velocity_valid ? std::to_string(vehicle.actual_vy) : "null")
        << ",\"actual_vz\":" << (vehicle.actual_velocity_valid ? std::to_string(vehicle.actual_vz) : "null")
        << ",\"safety_reason\":\""
        << (telemetry_fresh ? "" : "heartbeat_stale") << "\"}";
    return out.str();
}

}  // namespace

int main(int argc, char** argv) {
    try {
        std::signal(SIGINT, on_signal);
        std::signal(SIGTERM, on_signal);
        const Args args = parse_args(argc, argv);
        const YamlValue cfg = load_gcs_yaml();
        const YamlValue& telem = cfg["telemetry"];
        const char* env_dest_host = std::getenv("GCS_TELEMETRY_HOST");
        const char* env_dest_port = std::getenv("GCS_TELEMETRY_PORT");
        const std::string dest_host = env_dest_host && *env_dest_host
                                          ? env_dest_host
                                          : telem.get_string_or("dest_host", "127.0.0.1");
        const uint16_t dest_port = static_cast<uint16_t>(
            env_dest_port && *env_dest_port
                ? std::stoul(env_dest_port)
                : telem.get_long_or("dest_port", 15550));
        const uint8_t lanes = static_cast<uint8_t>(telem.get_long_or("lanes", 3));
        const uint8_t group_size = static_cast<uint8_t>(telem.get_long_or("group_size", 4));
        const int rate_hz = static_cast<int>(telem.get_long_or("rate_hz", 6));
        const int target_udp_port = static_cast<int>(
            telem.get_long_or("target_udp_port", 15020));

        auto mav = open_connection(args.telemetry_endpoint);
        gcs::UdpSender udp(dest_host, dest_port);
        gcs::FecEncoder encoder(lanes, group_size,
            [&udp](const uint8_t* data, size_t len) { udp.send(data, len); });
        TargetRangeReceiver target_rx(target_udp_port);
        GcsCommandStateReceiver command_rx(static_cast<int>(
            telem.get_long_or("command_state_udp_port", 15021)));
        VehicleState vehicle;
        TargetRangeMsg target{};
        GcsCommandStateMsg command{};
        bool have_target = false;
        bool have_command = false;
        uint32_t sequence = 0;
        const auto start = Clock::now();
        const auto period = std::chrono::milliseconds(std::max(1, 1000 / rate_hz));

        std::cout << "telem_sender: UDP telemetry=" << args.telemetry_endpoint
                  << " destination=" << dest_host << ":" << dest_port
                  << " rate=" << rate_hz << "Hz (read-only)" << std::endl;
        while (!g_stop && (args.duration_sec <= 0.0 ||
                           std::chrono::duration<double>(Clock::now() - start).count() < args.duration_sec)) {
            const auto tick = Clock::now();
            mavlink_message_t message{};
            while (mav->recv_match({}, message, 0.001)) {
                consume_mavlink(message, vehicle, Clock::now());
            }
            if (target_rx.poll(target)) have_target = true;
            if (command_rx.poll(command)) have_command = true;
            const std::string payload = make_json(sequence++, vehicle, target, have_target,
                                                  command, have_command, Clock::now());
            if (payload.size() > gcs::kMaxPayload) {
                throw std::runtime_error("telemetry JSON exceeds the FEC payload limit");
            }
            encoder.submit(reinterpret_cast<const uint8_t*>(payload.data()),
                           static_cast<uint16_t>(payload.size()));
            std::this_thread::sleep_until(tick + period);
        }
        encoder.flush();
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "telem_sender: " << error.what() << std::endl;
        return 1;
    }
}
