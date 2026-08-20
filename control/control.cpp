#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits.h>
#include <map>
#include <sstream>
#include <sys/stat.h>
#include <thread>
#include <unistd.h>
#include <vector>

#include "drone_lib.hpp"
#include "autopilot/command_sender.hpp"
#include "autopilot/real_flight_guard.hpp"
#include "autopilot/runtime_transport.hpp"
#include "logging/command_audit_logger.hpp"
#include "mission/diagonal_approach_guidance.hpp"
#include "mission/pixel_guidance.hpp"
#include "mission/target_centered_hold.hpp"
#include "pos_calculator.hpp"
#include "safety/control_authority.hpp"
#include "safety/flight_state.hpp"
#include "safety/health_monitor.hpp"
#include "safety/preflight_gate.hpp"
#include "target_link.hpp"

namespace {

using HealthLimit = safety::HealthLimit;
using HealthState = safety::HealthState;
using safety::apply_health_message;
using safety::evaluate_health_breach;
using TelemetryObserver = std::function<void(const mavlink_message_t*)>;

struct PreflightMessageDiagnostic {
    std::size_t count = 0;
    std::chrono::steady_clock::time_point first_at{};
    std::chrono::steady_clock::time_point last_at{};
    uint8_t last_sysid = 0;
    uint8_t last_compid = 0;
};

const char* preflight_message_name(uint32_t message_id) {
    switch (message_id) {
        case MAVLINK_MSG_ID_HEARTBEAT: return "HEARTBEAT";
        case MAVLINK_MSG_ID_GPS_RAW_INT: return "GPS_RAW_INT";
        case MAVLINK_MSG_ID_EKF_STATUS_REPORT: return "EKF_STATUS_REPORT";
        case MAVLINK_MSG_ID_SYS_STATUS: return "SYS_STATUS";
        default: return "OTHER";
    }
}

// safety.yaml stores booleans as the bare strings "true"/"false" --
// YamlValue has no bool accessor of its own.
bool get_bool_or(const YamlValue& parent, const std::string& key, bool default_value) {
    try {
        return parent[key].as_string() == "true";
    } catch (const std::exception&) {
        return default_value;
    }
}

// Telemetry configuration policy: MAV_CMD_SET_MESSAGE_INTERVAL is not a
// vehicle-affecting flight command and remains outside CommandGate.
void request_message_interval(MavConnection& connection, uint32_t message_id, double frequency_hz) {
    int64_t interval_us = static_cast<int64_t>(1'000'000 / frequency_hz);
    mavlink_message_t msg;
    mavlink_msg_command_long_pack(255, 0, &msg, connection.target_system(),
                                   connection.target_component(), MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                                   static_cast<float>(message_id), static_cast<float>(interval_us),
                                   0, 0, 0, 0, 0);
    connection.send(msg);
}

std::string mode_string(uint32_t custom_mode) {
    for (const auto& entry : copter_mode_mapping()) {
        if (entry.second == custom_mode) return entry.first;
    }
    return "Mode(" + std::to_string(custom_mode) + ")";
}

void write_marker_file_from_env(const char* env_name, const char* label) {
    const char* path = std::getenv(env_name);
    if (path == nullptr || *path == '\0') return;

    std::ofstream marker(path, std::ios::trunc);
    if (!marker) {
        throw std::runtime_error(std::string("could not create ") + label +
                                 " marker: " + path);
    }
    marker << label << "\n";
}

void wait_for_marker_file_from_env(const char* env_name, const char* label,
                                   double timeout_sec = 120.0) {
    const char* path = std::getenv(env_name);
    if (path == nullptr || *path == '\0') return;

    const auto start = std::chrono::steady_clock::now();
    while (access(path, F_OK) != 0) {
        const double elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - start).count();
        if (elapsed >= timeout_sec) {
            throw std::runtime_error(std::string("timed out waiting for ") + label +
                                     " marker: " + path);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

// The startup sequence (set_mode/arm_disarm/takeoff below, before
// approach_target()'s own health loop takes over) used to be "send the
// command, sleep a fixed amount, hope it worked" - no check that the mode
// actually switched, that the vehicle actually armed, or that the Pixhawk
// was even still there to receive the command. This waits on live
// HEARTBEAT data instead:
//   - if is_ready is given, it polls HEARTBEAT until is_ready(hb) is true
//     (mode switched / armed) or timeout_sec elapses, and throws on
//     timeout - "sent" is not "worked".
//   - if is_ready is empty, it just waits out timeout_sec (e.g. takeoff's
//     climb, where no single HEARTBEAT field means "done") while still
//     watching that heartbeats keep arriving.
// Either way, a heartbeat gap past max_heartbeat_gap_sec throws
// immediately: past that point we can no longer tell whether the vehicle
// is even still listening, so waiting out the rest of the timeout
// wouldn't tell us anything.
void wait_for_heartbeat(MavConnection& vehicle, double timeout_sec, double max_heartbeat_gap_sec,
                         const std::string& what,
                         const std::function<bool(const mavlink_heartbeat_t&)>& is_ready = nullptr,
                         const TelemetryObserver& observe = nullptr) {
    auto start = std::chrono::steady_clock::now();
    auto last_heartbeat = start;
    while (true) {
        if (observe) observe(nullptr);
        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(now - start).count();
        if (elapsed >= timeout_sec) {
            if (!is_ready) return;
            throw std::runtime_error(what + ": " + std::to_string(timeout_sec) +
                                      "초 내에 확인되지 않음");
        }
        double heartbeat_gap = std::chrono::duration<double>(now - last_heartbeat).count();
        if (heartbeat_gap > max_heartbeat_gap_sec) {
            throw std::runtime_error(what + ": heartbeat " + std::to_string(max_heartbeat_gap_sec) +
                                      "초 이상 끊김 - 픽스호크 응답 없음");
        }
        mavlink_message_t msg;
        if (vehicle.recv_match({MAVLINK_MSG_ID_HEARTBEAT, MAVLINK_MSG_ID_STATUSTEXT}, msg, 0.2)) {
            if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
                mavlink_heartbeat_t hb{};
                if (!is_valid_ardupilot_heartbeat(msg, &hb)) continue;
                if (observe) observe(&msg);
                last_heartbeat = now;
                if (is_ready && is_ready(hb)) return;
                continue;
            }
            if (observe) observe(&msg);
            continue;
        }
    }
}

bool env_flag_enabled(const char* name) {
    const char* value = std::getenv(name);
    return value != nullptr &&
           (std::string(value) == "1" || std::string(value) == "true" ||
            std::string(value) == "TRUE");
}

// Preflight is deliberately a read-only telemetry phase. It does not send a
// vehicle command; the only command sent before it is the existing telemetry
// interval request. Vehicle-affecting commands stay blocked by CommandGate
// until this function returns successfully.
void wait_for_preflight(
    MavConnection& vehicle, const HealthLimit& health_limit,
    safety::PreflightGate& preflight, safety::HealthMonitor& health_monitor,
    const TelemetryObserver& observe_telemetry, double timeout_sec = 30.0) {
    using Clock = std::chrono::steady_clock;
    const auto start = Clock::now();
    std::map<uint32_t, PreflightMessageDiagnostic> received_messages;
    const auto print_diagnostic_summary = [&]() {
        for (uint32_t message_id : {MAVLINK_MSG_ID_HEARTBEAT, MAVLINK_MSG_ID_GPS_RAW_INT,
                                    MAVLINK_MSG_ID_EKF_STATUS_REPORT, MAVLINK_MSG_ID_SYS_STATUS}) {
            const auto it = received_messages.find(message_id);
            if (it == received_messages.end()) {
                std::cerr << "[PREFLIGHT_DIAG] missing msgid=" << message_id
                          << " name=" << preflight_message_name(message_id) << std::endl;
                continue;
            }
            const auto& diagnostic = it->second;
            const double age_sec = std::chrono::duration<double>(
                Clock::now() - diagnostic.last_at).count();
            std::cerr << "[PREFLIGHT_DIAG] summary msgid=" << message_id
                      << " name=" << preflight_message_name(message_id)
                      << " count=" << diagnostic.count
                      << " last_age_sec=" << std::fixed << std::setprecision(3) << age_sec
                      << " last_sysid=" << static_cast<int>(diagnostic.last_sysid)
                      << " last_compid=" << static_cast<int>(diagnostic.last_compid)
                      << std::endl;
        }
        const auto& health = health_monitor.state();
        const auto age_ms = [&](Clock::time_point updated_at) {
            if (updated_at == Clock::time_point{}) return -1.0;
            return std::chrono::duration<double, std::milli>(
                       Clock::now() - updated_at).count();
        };
        std::cerr << "[PREFLIGHT_DIAG] health"
                  << " gps_have=" << health.have_gps
                  << " gps_fix=" << static_cast<int>(health.fix_type)
                  << " gps_sats=" << static_cast<int>(health.satellites)
                  << " gps_age_ms=" << age_ms(health.gps_updated_at)
                  << " ekf_have=" << health.have_ekf
                  << " ekf_pos_var=" << health.ekf_pos_horiz_variance
                  << " ekf_vel_var=" << health.ekf_velocity_variance
                  << " ekf_age_ms=" << age_ms(health.ekf_updated_at)
                  << " battery_have=" << health.have_battery
                  << " battery_valid=" << health.battery_valid
                  << " battery_voltage_v=" << health.battery_voltage_v
                  << " battery_percent=" << health.battery_percent
                  << " battery_age_ms=" << age_ms(health.battery_updated_at)
                  << " heartbeat_age_ms=" << age_ms(health.last_heartbeat)
                  << std::endl;
    };
    std::cerr << "[PREFLIGHT_DIAG] control telemetry receive loop started; "
              << "required=HEARTBEAT(0),GPS_RAW_INT(24),EKF_STATUS_REPORT(193),"
              << "SYS_STATUS(1)" << std::endl;
    const bool require_rc = env_flag_enabled("REQUIRE_RC_PREFLIGHT");
    const bool require_vision = env_flag_enabled("REQUIRE_VISION_PREFLIGHT");
    const char* yolo_ready_path = std::getenv("YOLO_READY_FILE");
    const bool yolo_marker_ready =
        yolo_ready_path != nullptr && *yolo_ready_path != '\0' &&
        access(yolo_ready_path, F_OK) == 0;
    const char* camera_ready_path = std::getenv("CAMERA_FRAME_READY_FILE");
    const bool camera_marker_ready =
        camera_ready_path != nullptr && *camera_ready_path != '\0' &&
        access(camera_ready_path, F_OK) == 0;
    bool rc_seen = false;

    safety::PreflightSample sample;
    sample.heartbeat_event = true;
    sample.valid_autopilot_heartbeat = true;  // initial wait_heartbeat() already validated it
    sample.heartbeat_fresh = true;
    sample.rc_policy_ok = !require_rc;
    sample.yolo_ready = !require_vision || yolo_marker_ready;
    sample.camera_frame = require_vision && camera_marker_ready;
    sample.now = start;
    preflight.observe(sample);

    while (!preflight.ready()) {
        const auto now = Clock::now();
        if (std::chrono::duration<double>(now - start).count() >= timeout_sec) {
            print_diagnostic_summary();
            std::string reason = "preflight timeout";
            for (const auto& item : preflight.reasons()) reason += "; " + item;
            throw std::runtime_error(reason);
        }

        sample = safety::PreflightSample{};
        sample.now = now;
        sample.rc_policy_ok = !require_rc || rc_seen;
        sample.yolo_ready = !require_vision || yolo_marker_ready;
        sample.camera_frame = require_vision && camera_marker_ready;
        mavlink_message_t message{};
        if (vehicle.recv_match({MAVLINK_MSG_ID_HEARTBEAT, MAVLINK_MSG_ID_SYS_STATUS,
                                MAVLINK_MSG_ID_GPS_RAW_INT,
                                MAVLINK_MSG_ID_EKF_STATUS_REPORT, MAVLINK_MSG_ID_RC_CHANNELS,
                                MAVLINK_MSG_ID_STATUSTEXT},
                               message, 0.2)) {
            auto& diagnostic = received_messages[message.msgid];
            ++diagnostic.count;
            if (diagnostic.first_at == Clock::time_point{}) diagnostic.first_at = now;
            diagnostic.last_at = now;
            diagnostic.last_sysid = message.sysid;
            diagnostic.last_compid = message.compid;
            if (diagnostic.count == 1 || diagnostic.count % 10 == 0) {
                std::cerr << "[PREFLIGHT_DIAG] received msgid=" << message.msgid
                          << " name=" << preflight_message_name(message.msgid)
                          << " sysid=" << static_cast<int>(message.sysid)
                          << " compid=" << static_cast<int>(message.compid)
                          << " count=" << diagnostic.count << std::endl;
            }
            if (observe_telemetry) observe_telemetry(&message);
            if (message.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
                mavlink_heartbeat_t heartbeat{};
                if (!is_valid_ardupilot_heartbeat(message, &heartbeat)) continue;
                sample.heartbeat_event = true;
                sample.valid_autopilot_heartbeat = true;
            } else if (message.msgid == MAVLINK_MSG_ID_RC_CHANNELS) {
                mavlink_rc_channels_t channels{};
                mavlink_msg_rc_channels_decode(&message, &channels);
                rc_seen = channels.chancount > 0;
                sample.rc_policy_ok = !require_rc || rc_seen;
            }
            health_monitor.update(message, now);
        }

        const auto& state = health_monitor.state();
        sample.gps_ok = state.have_gps && state.fix_type >= health_limit.min_fix_type &&
                        (state.satellites == 255 || state.satellites >= health_limit.min_satellites);
        sample.ekf_ok = state.have_ekf &&
                       state.ekf_pos_horiz_variance <= health_limit.ekf_pos_horiz_variance_max &&
                       state.ekf_velocity_variance <= health_limit.ekf_velocity_variance_max;
        sample.battery_valid = state.battery_valid &&
                               state.battery_voltage_v >= health_limit.min_battery_voltage_v &&
                               state.battery_percent >= health_limit.min_battery_percent;
        const auto fresh = [&](Clock::time_point updated_at) {
            return updated_at != Clock::time_point{} &&
                   std::chrono::duration<double>(now - updated_at).count() <=
                       health_limit.max_heartbeat_gap_sec;
        };
        sample.heartbeat_fresh = fresh(state.last_heartbeat);
        sample.telemetry_fresh = fresh(state.gps_updated_at) &&
                                 fresh(state.ekf_updated_at) && fresh(state.battery_updated_at);
        sample.rc_policy_ok = !require_rc || rc_seen;
        preflight.observe(sample);
        preflight.poll(now);
    }

    print_diagnostic_summary();
}

// Equirectangular approximation, accurate to well under 1% over the
// few-hundred-meter distances a single approach_target() run covers -
// good enough for a verification/logging number, not for actual guidance
// (nothing here feeds a velocity command).
double gps_distance_m(double lat1, double lon1, double lat2, double lon2) {
    constexpr double kEarthRadiusM = 6371000.0;
    constexpr double kDegToRad = M_PI / 180.0;
    double mean_lat = (lat1 + lat2) * 0.5 * kDegToRad;
    double dx = (lon2 - lon1) * kDegToRad * std::cos(mean_lat) * kEarthRadiusM;
    double dy = (lat2 - lat1) * kDegToRad * kEarthRadiusM;
    return std::hypot(dx, dy);
}

// Store controller CSV logs under <repo_root>/logs. The executable normally
// lives in <repo_root>/build, so this remains stable regardless of the shell's
// current working directory.
std::string resolve_log_dir() {
    if (const char* configured = std::getenv("LOG_DIR"); configured != nullptr && *configured != '\0') {
        struct stat configured_stat;
        if (stat(configured, &configured_stat) == 0 && S_ISDIR(configured_stat.st_mode)) {
            return configured;
        }
        if (mkdir(configured, 0755) == 0) return configured;
    }
    char exe_path[PATH_MAX];
    ssize_t len = readlink("/proc/self/exe", exe_path, sizeof(exe_path) - 1);
    std::string exe_dir = ".";
    if (len != -1) {
        exe_path[len] = '\0';
        std::string full(exe_path);
        exe_dir = full.substr(0, full.find_last_of('/'));
    }
    std::string log_dir = exe_dir + "/../logs";
    struct stat st;
    if (stat(log_dir.c_str(), &st) == 0 && S_ISDIR(st.st_mode)) {
        return log_dir;
    }
    if (mkdir(log_dir.c_str(), 0755) == 0) return log_dir;
    return exe_dir;
}

std::string csv_quote(const std::string& s) {
    std::string out = "\"";
    for (char c : s) {
        if (c == '"') out += "\"\"";
        else out += c;
    }
    out += '"';
    return out;
}

// Per-tick flight log, one row per control-loop iteration plus one row on
// every notable event (LOITER enter/exit, altitude-limit start/stop,
// unexpected disarm). Written as CSV to logs/flight_<timestamp>.csv
// so a post-flight "why did it do that" question has something to look at
// besides the raw mav.tlog binary. Flushed every row on purpose: if the
// vehicle disarms unexpectedly, the interesting data is what got logged in
// the seconds right before that, and an unflushed buffer would lose it.
class FlightLogger {
public:
    explicit FlightLogger(const std::string& dir) {
        std::time_t now = std::time(nullptr);
        char stamp[32];
        std::strftime(stamp, sizeof(stamp), "%Y%m%d_%H%M%S", std::localtime(&now));
        path_ = dir + "/flight_" + stamp + ".csv";
        file_.open(path_);
        if (file_) {
            file_ << "timestamp,elapsed_s,mode,armed,tracking,dist_m,vx,yaw_rate,vz,alt_m,alt_status,"
                     "battery_percent,battery_voltage_v,gps_fix_type,gps_satellites,prearm_healthy,"
                     "system_status,emergency_active,emergency_reason,event,gps_lat,gps_lon,gps_moved_m,"
                     "u_px,v_px,normalized_x,normalized_y,body_forward_m,body_right_m,"
                     "observation_age_ms,vy,theta_forward_rad,theta_right_rad,"
                     "filtered_theta_forward_rad,filtered_theta_right_rad,raw_vx,raw_vy,"
                     "filtered_vx,filtered_vy,raw_vz,filtered_vz,theta_error_rad,descent_margin,"
                     "horizontal_speed_mps,desired_path_angle_rad,vz_by_path_mps,vz_by_safety_mps,"
                     "commanded_path_angle_rad,acceleration_mps2,jerk_mps3,angular_safe,"
                     "descent_allowed,safety_override\n";
            file_.flush();
            std::cout << "Flight log: " << path_ << std::endl;
        } else {
            std::cerr << "Warning: could not open flight log at " << path_ << " - continuing without it."
                      << std::endl;
        }
    }

    // Compatibility overload for event rows that do not have a target
    // observation. Keep those existing call sites semantically unchanged;
    // the diagnostic columns use zero/default values for such rows.
    void log(double elapsed_s, const std::string& mode, bool armed, bool tracking, double dist_m, double vx,
             double yaw_rate, double vz, double alt_m, const std::string& alt_status, int battery_percent,
             double battery_voltage_v, int gps_fix_type, int gps_satellites, bool prearm_healthy,
             int system_status, bool emergency_active, const std::string& emergency_reason,
             const std::string& event, double gps_lat, double gps_lon, double gps_moved_m) {
        log(elapsed_s, mode, armed, tracking, dist_m, vx, yaw_rate, vz, alt_m, alt_status,
            battery_percent, battery_voltage_v, gps_fix_type, gps_satellites, prearm_healthy,
            system_status, emergency_active, emergency_reason, event, gps_lat, gps_lon,
            gps_moved_m, TargetRangeMsg{}, 0.0);
    }

    void log(double elapsed_s, const std::string& mode, bool armed, bool tracking, double dist_m, double vx,
              double yaw_rate, double vz, double alt_m, const std::string& alt_status, int battery_percent,
              double battery_voltage_v, int gps_fix_type, int gps_satellites, bool prearm_healthy,
              int system_status, bool emergency_active, const std::string& emergency_reason,
              const std::string& event, double gps_lat, double gps_lon, double gps_moved_m,
              const TargetRangeMsg& observation, double vy,
              const mission::GuidanceDiagnostics& guidance = {}) {
        if (!file_) return;
        std::time_t now = std::time(nullptr);
        char timestamp[32];
        std::strftime(timestamp, sizeof(timestamp), "%Y-%m-%d %H:%M:%S", std::localtime(&now));
        file_ << timestamp << ',' << std::fixed << std::setprecision(3) << elapsed_s << ',' << mode << ','
              << (armed ? 1 : 0) << ',' << (tracking ? 1 : 0) << ',' << std::setprecision(2) << dist_m << ','
              << vx << ',' << yaw_rate << ',' << vz << ',' << alt_m << ',' << alt_status << ','
              << battery_percent << ',' << battery_voltage_v << ',' << gps_fix_type << ',' << gps_satellites
              << ',' << (prearm_healthy ? 1 : 0) << ',' << system_status << ','
              << (emergency_active ? 1 : 0) << ',' << csv_quote(emergency_reason) << ',' << csv_quote(event)
              << ',' << std::setprecision(7) << gps_lat << ',' << gps_lon << ',' << std::setprecision(2)
              << gps_moved_m << ',' << observation.x_px << ',' << observation.y_px << ','
              << observation.normalized_x << ',' << observation.normalized_y << ','
              << observation.body_forward_m << ',' << observation.body_right_m << ','
              << observation.observation_age_ms << ',' << vy << ','
              << guidance.theta_forward_rad << ',' << guidance.theta_right_rad << ','
              << guidance.filtered_theta_forward_rad << ',' << guidance.filtered_theta_right_rad
              << ',' << guidance.raw_vx << ',' << guidance.raw_vy << ','
              << guidance.filtered_vx << ',' << guidance.filtered_vy << ','
              << guidance.raw_vz << ',' << guidance.filtered_vz << ','
              << guidance.theta_error_rad << ',' << guidance.descent_margin << ','
              << guidance.horizontal_speed_mps << ',' << guidance.desired_path_angle_rad << ','
              << guidance.vz_by_path_mps << ',' << guidance.vz_by_safety_mps << ','
              << guidance.commanded_path_angle_rad << ','
              << guidance.acceleration_mps2 << ',' << guidance.jerk_mps3 << ','
              << (guidance.angular_safe ? 1 : 0) << ','
              << (guidance.descent_allowed ? 1 : 0) << ','
              << csv_quote(guidance.safety_override) << '\n';
        file_.flush();
    }

private:
    std::ofstream file_;
    std::string path_;
};

struct AltitudeLimit {
    double soft_limit_m = 4.0;
    double hard_limit_m = 5.0;
    double descent_speed_mps = 0.5;
    double recovery_margin_m = 0.2;
};

enum class ApproachOutcome { kLanded, kHandedBack, kHeld };

// Stage 2+3 of the target-approach pipeline, plus this program's safety
// layers: an altitude ceiling (setting/safety.yaml's altitude_limit), a
// vehicle-health watch (battery/heartbeat/gps/prearm) that hands off to
// LOITER on a breach, and an unexpected-disarm watch that requests LAND
// immediately (see the HEARTBEAT case below). control.cpp is the only
// process that ever sends MAVLink commands to the vehicle, so all of this
// lives here instead of in a separate emergency process - see
// setting/safety.yaml's top comment.
ApproachOutcome approach_target(const YamlValue& track, const AltitudeLimit& alt_limit,
                                const HealthLimit& health_limit,
                                autopilot::CommandSender& commands,
                                const TelemetryObserver& observe_telemetry, double duration_sec,
                                const std::string& resume_mode = "",
                                safety::FlightStateMachine* flight_state = nullptr,
                                bool allow_mavlink_writes = true) {
    int udp_port = static_cast<int>(track.get_long_or("udp_port", 15020));
    double cycle_ms = track.get_double_or("control_cycle_ms", 50);
    double max_horizontal_speed = track.get_double_or("max_forward_speed", 0.5);
    double max_lateral_speed = track.get_double_or("max_lateral_speed", max_horizontal_speed);
    const char* simulation_profile = std::getenv("SIMULATION_PROFILE");
    std::string guidance_mode = "centered_descent";
    try {
        guidance_mode = track["guidance_mode"].as_string();
    } catch (const std::exception&) {
        // Older settings files retain the centered_descent baseline.
    }
    if (guidance_mode != "centered_descent" && guidance_mode != "diagonal_approach") {
        throw std::runtime_error("guidance_mode must be centered_descent or diagonal_approach");
    }
    double center_tolerance_px = track.get_double_or("center_tolerance_px", 25.0);
    double center_hold_sec = track.get_double_or("center_hold_sec", 1.0);
    double vertical_descent_speed = track.get_double_or(
        "vertical_descent_speed", track.get_double_or("approach_descent_speed", 0.2));
    double max_descent_speed = track.get_double_or(
        "guidance_max_descent_speed_mps", vertical_descent_speed);
    double stop_distance = track.get_double_or(
        "stop_distance", track.get_double_or("approach_stop_distance", 2.0));
    double diagonal_stop_distance = track.get_double_or(
        "diagonal_approach_stop_distance_m", 1.0);
    mission::PixelGuidanceConfig guidance_config;
    guidance_config.calibration.fx_px =
        track.get_double_or("pixel_focal_length_x_px",
                            track.get_double_or("pixel_focal_length_px", 530.0));
    guidance_config.calibration.fy_px =
        track.get_double_or("pixel_focal_length_y_px",
                            track.get_double_or("pixel_focal_length_px", 530.0));
    guidance_config.calibration.cx_px =
        track.get_double_or("pixel_principal_point_x_px", 320.0);
    guidance_config.calibration.cy_px =
        track.get_double_or("pixel_principal_point_y_px", 240.0);
    guidance_config.gain = track.get_double_or("guidance_gain", 0.8);
    guidance_config.max_forward_speed = max_horizontal_speed;
    guidance_config.max_lateral_speed = max_lateral_speed;
    guidance_config.max_descent_speed_mps = max_descent_speed;
    guidance_config.desired_path_angle_rad =
        track.get_double_or("guidance_desired_path_angle_rad", 0.0);
    guidance_config.horizontal_speed_deadband_mps =
        track.get_double_or("guidance_horizontal_speed_deadband_mps", 0.01);
    guidance_config.max_vector_speed = track.get_double_or(
        "guidance_max_vector_speed_mps", std::min(max_horizontal_speed, max_lateral_speed));
    guidance_config.filter_alpha = track.get_double_or("guidance_filter_alpha", 0.25);
    guidance_config.max_acceleration_mps2 =
        track.get_double_or("guidance_max_acceleration_mps2", 1.0);
    guidance_config.max_jerk_mps3 = track.get_double_or("guidance_max_jerk_mps3", 5.0);
    guidance_config.velocity_deadband_rad =
        track.get_double_or("guidance_velocity_deadband_rad", 0.01);
    guidance_config.velocity_hysteresis_rad =
        track.get_double_or("guidance_velocity_hysteresis_rad", 0.005);
    guidance_config.theta_safe_threshold_rad =
        track.get_double_or("guidance_theta_safe_threshold_rad", 0.0872664626);

    mission::DiagonalApproachGuidanceConfig diagonal_config;
    diagonal_config.calibration = guidance_config.calibration;
    diagonal_config.gain = guidance_config.gain;
    diagonal_config.distance_gain = track.get_double_or(
        "diagonal_approach_distance_gain", 0.15);
    diagonal_config.max_forward_speed = max_horizontal_speed;
    diagonal_config.max_lateral_speed = max_lateral_speed;
    diagonal_config.max_vector_speed = guidance_config.max_vector_speed;
    diagonal_config.max_descent_speed_mps = guidance_config.max_descent_speed_mps;
    diagonal_config.desired_path_angle_rad = guidance_config.desired_path_angle_rad;
    diagonal_config.stop_distance_m = diagonal_stop_distance;
    diagonal_config.horizontal_speed_deadband_mps =
        guidance_config.horizontal_speed_deadband_mps;
    diagonal_config.filter_alpha = guidance_config.filter_alpha;
    diagonal_config.max_acceleration_mps2 = guidance_config.max_acceleration_mps2;
    diagonal_config.max_jerk_mps3 = guidance_config.max_jerk_mps3;
    diagonal_config.velocity_deadband_rad = guidance_config.velocity_deadband_rad;
    diagonal_config.velocity_hysteresis_rad = guidance_config.velocity_hysteresis_rad;
    diagonal_config.safe_fov_angle_rad = track.get_double_or(
        "diagonal_approach_safe_fov_angle_rad", 0.6);
    // For diagonal approach, the safe-FOV boundary is the descent margin.
    // The tighter centered-descent theta threshold must not turn ordinary
    // in-FOV pixel offset into a near-zero approach speed.
    diagonal_config.theta_safe_threshold_rad = diagonal_config.safe_fov_angle_rad;

    auto validation_override = [&](const char* name, double& target, double minimum) {
        const char* value = std::getenv(name);
        if (!value) return;
        char* end = nullptr;
        const double parsed = std::strtod(value, &end);
        if (end != value && *end == '\0' && std::isfinite(parsed) && parsed >= minimum) {
            target = parsed;
            std::cout << "[validation] " << name << "=" << target << std::endl;
        }
    };
    const bool simulation_validation_profile =
        simulation_profile != nullptr &&
        (std::string(simulation_profile) == "control-validation" ||
         std::string(simulation_profile) == "path-angle-validation" ||
         std::string(simulation_profile) == "diagonal-approach-validation");
    if (simulation_validation_profile) {
        validation_override("GUIDANCE_MAX_FORWARD_SPEED_MPS", max_horizontal_speed, 0.0);
        validation_override("GUIDANCE_MAX_LATERAL_SPEED_MPS", max_lateral_speed, 0.0);
        validation_override("GUIDANCE_FILTER_ALPHA", guidance_config.filter_alpha, 0.0);
        validation_override("GUIDANCE_MAX_ACCELERATION_MPS2",
                            guidance_config.max_acceleration_mps2, 0.0);
        validation_override("GUIDANCE_MAX_JERK_MPS3", guidance_config.max_jerk_mps3, 0.0);
        validation_override("GUIDANCE_VELOCITY_DEADBAND_RAD",
                            guidance_config.velocity_deadband_rad, 0.0);
        validation_override("GUIDANCE_VELOCITY_HYSTERESIS_RAD",
                            guidance_config.velocity_hysteresis_rad, 0.0);
        validation_override("GUIDANCE_THETA_SAFE_THRESHOLD_RAD",
                            guidance_config.theta_safe_threshold_rad, 0.0);
        validation_override("GUIDANCE_MAX_VECTOR_SPEED_MPS",
                            guidance_config.max_vector_speed, 0.0);
        validation_override("GUIDANCE_MAX_DESCENT_SPEED_MPS",
                            guidance_config.max_descent_speed_mps, 0.0);
        validation_override("GUIDANCE_DESIRED_PATH_ANGLE_RAD",
                            guidance_config.desired_path_angle_rad, 0.0);
    }
    guidance_config.max_forward_speed = max_horizontal_speed;
    guidance_config.max_lateral_speed = max_lateral_speed;
    // The diagonal instance is created from the final, possibly overridden
    // validation values so the opt-in simulation profile cannot silently use
    // the production speed/path limits.
    diagonal_config.max_forward_speed = max_horizontal_speed;
    diagonal_config.max_lateral_speed = max_lateral_speed;
    diagonal_config.max_vector_speed = guidance_config.max_vector_speed;
    diagonal_config.max_descent_speed_mps = guidance_config.max_descent_speed_mps;
    diagonal_config.desired_path_angle_rad = guidance_config.desired_path_angle_rad;
    diagonal_config.filter_alpha = guidance_config.filter_alpha;
    diagonal_config.max_acceleration_mps2 = guidance_config.max_acceleration_mps2;
    diagonal_config.max_jerk_mps3 = guidance_config.max_jerk_mps3;
    diagonal_config.velocity_deadband_rad = guidance_config.velocity_deadband_rad;
    diagonal_config.velocity_hysteresis_rad = guidance_config.velocity_hysteresis_rad;
    diagonal_config.theta_safe_threshold_rad = diagonal_config.safe_fov_angle_rad;
    double link_stale_ms = track.get_double_or("link_stale_ms", 500);
    // Continuous target loss past this many seconds escalates straight to
    // LAND, the same way heartbeat loss past land_gap_sec does below -
    // otherwise the vehicle just sits hovering in GUIDED on zero velocity
    // forever, waiting for a target that may never come back.
    double target_lost_land_sec = track.get_double_or("target_lost_land_sec", 10.0);

    // Altitude-limit correction: proportional-with-floor, same shape as
    // the guidance gains above - scales with overshoot past hard_limit_m,
    // but never below kAltMinDescent (a barely-over-the-limit reading still
    // gets pushed down) and never above descent_speed_mps (stays gentle).
    constexpr double kAltGain = 0.5;        // 1/s
    constexpr double kAltMinDescent = 0.1;  // m/s
    constexpr double kMinLoiterHoldSec = 3.0;
    // Small enough not to meaningfully disrupt the cycle_ms pacing below,
    // long enough to opportunistically pick up whatever SYS_STATUS/
    // GPS_RAW_INT/HEARTBEAT bytes are already sitting in the socket buffer.
    // This is the cost of control.cpp reading its own vehicle-health
    // telemetry instead of relaying it from a separate process - it can't
    // be removed without losing battery/gps/heartbeat monitoring entirely.
    constexpr double kHealthPollTimeoutSec = 0.005;
    mission::PixelGuidance guidance(guidance_config);
    mission::DiagonalApproachGuidance diagonal_guidance(diagonal_config);
    const bool use_diagonal_approach = guidance_mode == "diagonal_approach";
    std::cout << "[GUIDANCE] mode=" << guidance_mode << std::endl;

    MavConnection& vehicle = drone::require_connection();
    if (allow_mavlink_writes) {
        request_message_interval(vehicle, MAVLINK_MSG_ID_SYS_STATUS, health_limit.poll_rate_hz);
        request_message_interval(vehicle, MAVLINK_MSG_ID_GPS_RAW_INT, health_limit.poll_rate_hz);
        request_message_interval(vehicle, MAVLINK_MSG_ID_EKF_STATUS_REPORT, health_limit.poll_rate_hz);
    }
    // HEARTBEAT is broadcast on its own (~1 Hz) without needing a request.

    TargetRangeReceiver receiver(udp_port);
    GcsCommandStateSender gcs_command_state_sender(
        static_cast<int>(track.get_long_or("gcs_command_state_udp_port", 15021)));
    FlightLogger logger(resolve_log_dir());
    std::cout << "타겟 접근 모드 시작 (UDP " << udp_port << ", 주기 " << cycle_ms << "ms, 최대 "
              << duration_sec << "초, 고도제한 " << alt_limit.soft_limit_m << "~"
              << alt_limit.hard_limit_m << "m)" << std::endl;

    // Zero velocity on the way out no matter how this function exits -
    // duration elapsed, target lost, or an exception - so the vehicle never
    // keeps coasting on the last command it was given.
    struct FinallyGuard {
        autopilot::CommandSender& commands;

        ~FinallyGuard() {
            try {
                commands.send_zero_velocity();
            } catch (...) {
            }
        }
    } finally_guard{commands};

    mission::GuidanceDiagnostics guidance_diagnostics;
    auto guidance_force_zero = [&](const std::string& reason, double observation_age_ms) {
        return use_diagonal_approach
                   ? diagonal_guidance.force_zero(reason, observation_age_ms)
                   : guidance.force_zero(reason, observation_age_ms);
    };
    auto guidance_update = [&](double x_px, double y_px, double altitude_m,
                               double distance_m, double dt_sec, double observation_age_ms,
                               bool allow_descent_without_horizontal) {
        return use_diagonal_approach
                   ? diagonal_guidance.update(x_px, y_px, altitude_m, distance_m, true,
                                              dt_sec, observation_age_ms)
                   : guidance.update(x_px, y_px, altitude_m, true, dt_sec,
                                     observation_age_ms,
                                     allow_descent_without_horizontal);
    };
    auto send_immediate_zero = [&](const std::string& reason, double observation_age_ms = -1.0) {
        guidance_diagnostics = guidance_force_zero(reason, observation_age_ms);
        try {
            commands.send_zero_velocity();
        } catch (const std::exception& e) {
            std::cerr << "즉시 zero velocity 전송 실패: " << e.what() << std::endl;
        }
    };

    // The app-level altitude ceiling is still enforced for diagonal approach,
    // but its corrective descent must not destroy the selected path angle by
    // changing vz alone. Reconstruct the horizontal magnitude from the same
    // descent command while preserving its existing horizontal direction.
    auto preserve_diagonal_path_angle = [&](double& vx, double& vy, double& vz) {
        if (!use_diagonal_approach || vz <= 1e-9) return;
        const double tangent = std::tan(diagonal_config.desired_path_angle_rad);
        const double current_horizontal = std::hypot(vx, vy);
        if (tangent <= 1e-9 || current_horizontal <= 1e-9) return;
        vz = std::min(vz, diagonal_config.max_descent_speed_mps);
        const double desired_horizontal = vz / tangent;
        const double scale = desired_horizontal / current_horizontal;
        vx *= scale;
        vy *= scale;
    };

    auto start = std::chrono::steady_clock::now();
    auto last_valid = start - std::chrono::seconds(10);
    TargetRangeMsg last{};

    HealthState health;
    health.last_heartbeat = start;  // drone::connect() already saw one to get here

    bool alt_enforcing = false;
    bool in_loiter = false;
    auto loiter_since = start;
    std::string emergency_reason;
    // Set once target_distance.cpp has ever reported a fresh ALTITUDE
    // reading (altitude_valid=1) - gates the staleness breach below so
    // start-up, before target_distance.cpp has sent its first packet,
    // isn't itself treated as an emergency (same "seen at least once"
    // gating evaluate_health_breach() uses for battery/gps/sys_status).
    bool altitude_ever_valid = false;

    // Verification/logging only: tracks how far the vehicle has actually
    // moved, by GPS, since this approach began - a ground-truth cross-check
    // against the vision-derived dist_m column, not something any velocity
    // command below is computed from.
    bool have_gps_start = false;
    double gps_start_lat = 0.0;
    double gps_start_lon = 0.0;
    double gps_moved_m = 0.0;

    // Resets to cycle_start every cycle the target is actually tracked;
    // read below as "how long has it been continuously lost" (same
    // loop-start initialization as health.last_heartbeat above, so a
    // target that's never found from the very start still starts the
    // clock instead of never triggering).
    auto target_lost_since = start;
    bool center_locking = false;
    bool center_locked = false;
    // Once this approach has reached a centered hold, its normal intercept
    // timeout must not hand control back to AUTO. A later target loss is
    // handled only by the explicit target-loss policy below.
    bool hold_ever_entered = false;
    bool timeout_hover = false;
    auto center_since = start;
    mission::TargetCenteredHold centered_hold;

    while (true) {
        if (observe_telemetry) observe_telemetry(nullptr);
        auto cycle_start = std::chrono::steady_clock::now();
        double elapsed_sec = std::chrono::duration<double>(cycle_start - start).count();
        const bool approach_timeout = duration_sec > 0 && elapsed_sec >= duration_sec;

        TargetRangeMsg msg;
        if (receiver.poll(msg)) {
            last = msg;
            last_valid = cycle_start;
        }
        double link_age_ms = std::chrono::duration<double, std::milli>(cycle_start - last_valid).count();
        bool link_fresh = link_age_ms <= link_stale_ms;
        if (last.altitude_valid) altitude_ever_valid = true;

        bool tracking = link_fresh && last.valid && last.found;
        if (flight_state != nullptr) {
            if (!tracking && flight_state->current() == safety::FlightState::GUIDED) {
                flight_state->transition(safety::FlightState::TARGET_LOSS_HOVER,
                                         "target observation stale or lost");
            } else if (tracking &&
                       flight_state->current() == safety::FlightState::TARGET_LOSS_HOVER) {
                flight_state->transition(safety::FlightState::GUIDED,
                                         "target observation re-confirmed");
            }
        }
        if (tracking) target_lost_since = cycle_start;
        double target_lost_sec = std::chrono::duration<double>(cycle_start - target_lost_since).count();

        // Opportunistically pick up whatever vehicle-health telemetry has
        // arrived since the last tick (see kHealthPollTimeoutSec above).
        mavlink_message_t hmsg;
        if (vehicle.recv_match({MAVLINK_MSG_ID_SYS_STATUS, MAVLINK_MSG_ID_GPS_RAW_INT,
                                 MAVLINK_MSG_ID_HEARTBEAT, MAVLINK_MSG_ID_EKF_STATUS_REPORT,
                                 MAVLINK_MSG_ID_STATUSTEXT},
                               hmsg, kHealthPollTimeoutSec)) {
            if (hmsg.msgid == MAVLINK_MSG_ID_HEARTBEAT &&
                !is_valid_ardupilot_heartbeat(hmsg)) {
                continue;
            }
            if (observe_telemetry) observe_telemetry(&hmsg);
            switch (hmsg.msgid) {
                case MAVLINK_MSG_ID_SYS_STATUS: {
                    mavlink_sys_status_t s;
                    mavlink_msg_sys_status_decode(&hmsg, &s);
                    health.battery_percent = s.battery_remaining;
                    health.battery_voltage_v =
                        (s.voltage_battery == 0 || s.voltage_battery == 65535)
                            ? -1
                            : s.voltage_battery / 1000.0;
                    health.battery_valid = s.voltage_battery > 0 &&
                                           s.voltage_battery != 65535 &&
                                           s.battery_remaining >= 0;
                    health.prearm_healthy =
                        (s.onboard_control_sensors_health & MAV_SYS_STATUS_PREARM_CHECK) != 0;
                    health.have_battery = true;
                    health.have_sys_status = true;
                    break;
                }
                case MAVLINK_MSG_ID_GPS_RAW_INT: {
                    mavlink_gps_raw_int_t g;
                    mavlink_msg_gps_raw_int_decode(&hmsg, &g);
                    health.fix_type = g.fix_type;
                    health.satellites = g.satellites_visible;
                    health.have_gps = true;
                    // Same get_gps_location.cpp reading, kept here purely
                    // for the gps_moved_m verification/logging below - it
                    // never feeds a velocity command.
                    if (g.fix_type >= 2) {
                        health.lat = g.lat / 1e7;
                        health.lon = g.lon / 1e7;
                        health.have_position = true;
                    }
                    break;
                }
                case MAVLINK_MSG_ID_EKF_STATUS_REPORT: {
                    mavlink_ekf_status_report_t e;
                    mavlink_msg_ekf_status_report_decode(&hmsg, &e);
                    health.ekf_pos_horiz_variance = e.pos_horiz_variance;
                    health.ekf_velocity_variance = e.velocity_variance;
                    health.have_ekf = true;
                    break;
                }
                case MAVLINK_MSG_ID_HEARTBEAT: {
                    mavlink_heartbeat_t hb;
                    mavlink_msg_heartbeat_decode(&hmsg, &hb);
                    health.system_status = hb.system_status;
                    health.custom_mode = hb.custom_mode;
                    bool now_armed = is_armed_from_heartbeat(hb);

                    // Unexpected disarm: this loop never disarms on its own
                    // (arm_disarm(false) only ever happens outside
                    // approach_target(), after it returns), so any
                    // true -> false transition observed here is external -
                    // a crash, a firmware failsafe cutting motors, physical
                    // damage, etc. Request LAND immediately (best-effort;
                    // if it's truly unresponsive this does nothing, but if
                    // it's still listening it can only help) and stop -
                    // there is nothing else this loop can usefully command
                    // with the motors off.
                    if (health.have_armed && health.armed && !now_armed) {
                        std::cout << "[EMERGENCY] 예상치 못한 disarm 감지 (armed: true -> false) - "
                                     "긴급 LAND 명령 전송 후 종료"
                                  << std::endl;
                        send_immediate_zero("unexpected_disarm");
                        logger.log(elapsed_sec, mode_string(health.custom_mode), false, false, last.distance_m,
                                   0, 0, 0, last.altitude_m, "", health.battery_percent,
                                   health.battery_voltage_v, health.fix_type, health.satellites,
                                   health.prearm_healthy, health.system_status, true,
                                   "unexpected disarm mid-flight", "UNEXPECTED_DISARM", health.lat,
                                   health.lon, gps_moved_m, TargetRangeMsg{}, 0.0,
                                   guidance_diagnostics);
                        try {
                            commands.land();
                        } catch (const std::exception& e) {
                            std::cerr << "LAND 명령 전송 실패: " << e.what() << std::endl;
                        }
                        return ApproachOutcome::kLanded;
                    }
                    health.armed = now_armed;
                    health.have_armed = true;
                    health.last_heartbeat = cycle_start;
                    break;
                }
                default:
                    break;
            }
        }

        if (health.have_position) {
            if (!have_gps_start) {
                gps_start_lat = health.lat;
                gps_start_lon = health.lon;
                have_gps_start = true;
            }
            gps_moved_m = gps_distance_m(gps_start_lat, gps_start_lon, health.lat, health.lon);
        }

        // Heartbeat loss past land_gap_sec is treated as more severe than
        // the generic health breach below: we can no longer trust that
        // anything we command will even arrive, so instead of sitting in
        // LOITER hoping the link comes back, request LAND (best-effort -
        // harmless if the link really is dead, could still land through if
        // it's only degraded) and stop, the same way an unexpected disarm
        // does above.
        double heartbeat_gap = std::chrono::duration<double>(cycle_start - health.last_heartbeat).count();
        if (heartbeat_gap > health_limit.land_gap_sec) {
            std::cout << "[EMERGENCY] heartbeat gap " << std::fixed << std::setprecision(1) << heartbeat_gap
                      << "s > land_gap_sec " << health_limit.land_gap_sec << "s - 긴급 LAND 명령 전송 후 종료"
                      << std::endl;
            send_immediate_zero("heartbeat_timeout");
            logger.log(elapsed_sec, mode_string(health.custom_mode), health.armed, false, last.distance_m, 0,
                       0, 0, last.altitude_m, "", health.battery_percent, health.battery_voltage_v,
                       health.fix_type, health.satellites, health.prearm_healthy, health.system_status, true,
                       "heartbeat gap exceeded land_gap_sec", "HEARTBEAT_LAND_ESCALATION", health.lat,
                       health.lon, gps_moved_m, TargetRangeMsg{}, 0.0, guidance_diagnostics);
            try {
                commands.land();
            } catch (const std::exception& e) {
                std::cerr << "LAND 명령 전송 실패: " << e.what() << std::endl;
            }
            return ApproachOutcome::kLanded;
        }

        // After a continuous target loss, stop the autonomous mission at the
        // current position. The default LOITER policy prevents AUTO from
        // immediately continuing the mission past the target.
        if (target_lost_sec > target_lost_land_sec) {
            if (resume_mode == "LOITER") {
                std::cout << "[AUTO_INTERCEPT] 타겟 로스트 " << std::fixed
                          << std::setprecision(1) << target_lost_sec
                          << "s - LOITER 호버링 유지" << std::endl;
                send_immediate_zero("target_loss", last.observation_age_ms);
                logger.log(elapsed_sec, mode_string(health.custom_mode), health.armed, false,
                           last.distance_m, 0, 0, 0, last.altitude_m, "",
                           health.battery_percent, health.battery_voltage_v, health.fix_type,
                           health.satellites, health.prearm_healthy, health.system_status, false,
                           "", "TARGET_LOST_HOVER", health.lat, health.lon, gps_moved_m,
                           TargetRangeMsg{}, 0.0, guidance_diagnostics);
                if (!commands.set_mode("LOITER")) {
                    std::cerr << "[AUTO_INTERCEPT] LOITER 전환 명령이 허용되지 않았습니다."
                              << std::endl;
                }
                return ApproachOutcome::kHeld;
            }
            if (!resume_mode.empty()) {
                std::cout << "[AUTO_INTERCEPT] 타겟 로스트 " << std::fixed
                          << std::setprecision(1) << target_lost_sec << "s - "
                          << resume_mode << "로 복귀" << std::endl;
                send_immediate_zero("target_loss", last.observation_age_ms);
                logger.log(elapsed_sec, mode_string(health.custom_mode), health.armed, false,
                           last.distance_m, 0, 0, 0, last.altitude_m, "",
                           health.battery_percent, health.battery_voltage_v, health.fix_type,
                           health.satellites, health.prearm_healthy, health.system_status, false,
                           "", "TARGET_LOST_RESUME_" + resume_mode, health.lat, health.lon,
                           gps_moved_m, TargetRangeMsg{}, 0.0, guidance_diagnostics);
                commands.set_mode(resume_mode);
                return ApproachOutcome::kHandedBack;
            }
            std::cout << "[EMERGENCY] 타겟 로스트 " << std::fixed << std::setprecision(1)
                      << target_lost_sec << "s > target_lost_land_sec " << target_lost_land_sec
                      << "s - 긴급 LAND 명령 전송 후 종료" << std::endl;
            send_immediate_zero("target_loss", last.observation_age_ms);
            logger.log(elapsed_sec, mode_string(health.custom_mode), health.armed, false, last.distance_m, 0,
                       0, 0, last.altitude_m, "", health.battery_percent, health.battery_voltage_v,
                       health.fix_type, health.satellites, health.prearm_healthy, health.system_status, true,
                       "target lost exceeded target_lost_land_sec", "TARGET_LOST_LAND_ESCALATION",
                       health.lat, health.lon, gps_moved_m, TargetRangeMsg{}, 0.0,
                       guidance_diagnostics);
            try {
                commands.land();
            } catch (const std::exception& e) {
                std::cerr << "LAND 명령 전송 실패: " << e.what() << std::endl;
            }
            return ApproachOutcome::kLanded;
        }

        std::vector<std::string> reasons = evaluate_health_breach(health, health_limit, cycle_start);
        // target_distance.cpp clears altitude_valid the moment its own
        // ALTITUDE reading goes stale (see its altitude_stale_ms), so
        // !last.altitude_valid here already reflects "no fresh altitude
        // right now" - no extra timing needed on this side. Without this,
        // the altitude-limit block a few lines down would keep using
        // last.altitude_m as if current even after it stopped updating.
        if (altitude_ever_valid && !last.altitude_valid) {
            reasons.push_back("altitude data stale (target_distance.cpp)");
        }
        bool emergency_active = !reasons.empty();
        if (flight_state != nullptr && emergency_active &&
            flight_state->current() != safety::FlightState::SAFE_HOVER) {
            flight_state->transition(safety::FlightState::SAFE_HOVER,
                                     reasons.front());
        }
        emergency_reason.clear();
        for (size_t i = 0; i < reasons.size(); ++i) {
            if (i > 0) emergency_reason += "; ";
            emergency_reason += reasons[i];
        }

        auto next_tick = cycle_start + std::chrono::duration<double>(cycle_ms / 1000.0);
        std::string mode_name = mode_string(health.custom_mode);

        if (emergency_active) {
            send_immediate_zero("health_safety_failure", last.observation_age_ms);
            std::string event;
            if (!in_loiter) {
                std::cout << "[EMERGENCY] " << emergency_reason << ": LOITER로 전환" << std::endl;
                commands.set_mode("LOITER");
                in_loiter = true;
                event = "LOITER_ENTER";
            }
            loiter_since = cycle_start;  // keep refreshing while active
            logger.log(elapsed_sec, mode_name, health.armed, false, last.distance_m, 0, 0, 0,
                       last.altitude_m, "", health.battery_percent, health.battery_voltage_v,
                       health.fix_type, health.satellites, health.prearm_healthy, health.system_status,
                       true, emergency_reason, event, health.lat, health.lon, gps_moved_m,
                       TargetRangeMsg{}, 0.0, guidance_diagnostics);
            std::this_thread::sleep_until(next_tick);
            continue;
        }

        if (commands.control_locked()) {
            send_immediate_zero("control_locked", last.observation_age_ms);
            logger.log(elapsed_sec, mode_name, health.armed, false, last.distance_m, 0, 0, 0,
                       last.altitude_m, "", health.battery_percent, health.battery_voltage_v,
                       health.fix_type, health.satellites, health.prearm_healthy,
                       health.system_status, true, "ControlLocked", "CONTROL_LOCKED",
                       health.lat, health.lon, gps_moved_m, TargetRangeMsg{}, 0.0,
                       guidance_diagnostics);
            std::this_thread::sleep_until(next_tick);
            continue;
        }

        if (in_loiter) {
            double held_sec = std::chrono::duration<double>(cycle_start - loiter_since).count();
            if (held_sec < kMinLoiterHoldSec) {
                std::cout << std::fixed << std::setprecision(1) << "LOITER 최소 유지 중 (" << held_sec
                          << "/" << kMinLoiterHoldSec << "s)" << std::endl;
                send_immediate_zero("loiter_safety_hold", last.observation_age_ms);
                logger.log(elapsed_sec, mode_name, health.armed, false, last.distance_m, 0, 0, 0,
                           last.altitude_m, "", health.battery_percent, health.battery_voltage_v,
                           health.fix_type, health.satellites, health.prearm_healthy, health.system_status,
                           false, "", "LOITER_HOLD", health.lat, health.lon, gps_moved_m,
                           TargetRangeMsg{}, 0.0, guidance_diagnostics);
                std::this_thread::sleep_until(next_tick);
                continue;
            }
            std::cout << "위험 상태 해제, 최소 유지시간 경과 - GUIDED로 복귀." << std::endl;
            commands.set_mode("GUIDED");
            in_loiter = false;
            logger.log(elapsed_sec, mode_name, health.armed, false, last.distance_m, 0, 0, 0,
                       last.altitude_m, "", health.battery_percent, health.battery_voltage_v,
                       health.fix_type, health.satellites, health.prearm_healthy, health.system_status,
                       false, "", "LOITER_EXIT", health.lat, health.lon, gps_moved_m);
        }

        double vx = 0.0, vy = 0.0, yaw_rate = 0.0;
        double vz = 0.0;  // body frame: +z down
        std::string guidance_event;

        // A normal approach timeout must never hand control back to AUTO
        // before the target has reached stop_distance. Keep the vehicle in
        // GUIDED with zero velocity; target-loss and health safety paths
        // above still take precedence and may apply their existing policy.
        if (approach_timeout && tracking && !hold_ever_entered &&
            !centered_hold.holding() && !timeout_hover) {
            timeout_hover = true;
            std::cout << "접근 제한 시간(" << duration_sec
                      << "초) 도달 - Hold 전 안전 호버링 유지" << std::endl;
            logger.log(elapsed_sec, mode_name, health.armed, true, last.distance_m, 0, 0, 0,
                       last.altitude_m, "", health.battery_percent, health.battery_voltage_v,
                       health.fix_type, health.satellites, health.prearm_healthy,
                       health.system_status, false, "", "INTERCEPT_TIMEOUT_HOVER", health.lat,
                       health.lon, gps_moved_m, last, 0.0, guidance_diagnostics);
        }

        if (timeout_hover && !centered_hold.holding()) {
            send_immediate_zero("intercept_timeout_hover", last.observation_age_ms);
            logger.log(elapsed_sec, mode_name, health.armed, tracking, last.distance_m, 0, 0, 0,
                       last.altitude_m, "", health.battery_percent, health.battery_voltage_v,
                       health.fix_type, health.satellites, health.prearm_healthy,
                       health.system_status, false, "", "INTERCEPT_TIMEOUT_HOVER_HOLD",
                       health.lat, health.lon, gps_moved_m, last, 0.0, guidance_diagnostics);
            std::this_thread::sleep_until(next_tick);
            continue;
        }

        if (tracking) {
            const double distance_m = static_cast<double>(last.distance_m);
            const double active_stop_distance = use_diagonal_approach
                                                    ? diagonal_config.stop_distance_m
                                                    : stop_distance;
            const double x_px = static_cast<double>(last.x_px);
            const double y_px = static_cast<double>(last.y_px);
            const bool centered = std::abs(x_px) <= center_tolerance_px &&
                                  std::abs(y_px) <= center_tolerance_px;
            guidance_diagnostics = guidance_update(
                x_px, y_px, std::max(0.05, static_cast<double>(last.altitude_m)), distance_m,
                cycle_ms / 1000.0, last.observation_age_ms,
                center_locked && distance_m > stop_distance);
            vx = guidance_diagnostics.vx;
            vy = guidance_diagnostics.vy;

            if (centered) {
                if (!center_locking) {
                    center_locking = true;
                    center_since = cycle_start;
                }
                if (!center_locked &&
                    std::chrono::duration<double>(cycle_start - center_since).count() >=
                        center_hold_sec) {
                    center_locked = true;
                    guidance_event = "TARGET_CENTER_LOCKED";
                    std::cout << "[GUIDANCE] 표적 중앙 유지 확인 - 수평 정렬 상태 기록"
                              << std::endl;
                }
            } else {
                if (center_locked) guidance_event = "TARGET_CENTER_LOST";
                center_locking = false;
                center_locked = false;
            }

            if (centered_hold.holding()) {
                guidance_diagnostics = guidance_force_zero(
                    "target_centered_hold", last.observation_age_ms);
                vx = 0.0;
                vy = 0.0;
                vz = 0.0;
                yaw_rate = 0.0;
            } else if (distance_m <= active_stop_distance &&
                       centered_hold.enter_if_ready(tracking, centered, distance_m,
                                                    active_stop_distance)) {
                guidance_diagnostics = guidance_force_zero(
                    "target_centered_hold", last.observation_age_ms);
                vx = 0.0;
                vy = 0.0;
                vz = 0.0;
                yaw_rate = 0.0;
                guidance_event = "TARGET_CENTERED_HOLD";
                hold_ever_entered = true;
                std::cout << "[GUIDANCE] TargetCenteredHold 진입 - GUIDED 제로 속도 호버링 유지"
                          << std::endl;
            } else if (distance_m > active_stop_distance) {
                // TARGET_CENTER_LOCKED is only a lateral-alignment marker.
                // Keep diagonal approach active while the target remains
                // outside the mode-specific stop distance.
                vz = guidance_diagnostics.vz;
                yaw_rate = 0.0;
                if (use_diagonal_approach) guidance_event = "DIAGONAL_APPROACH";
            } else {
                // In diagonal mode this is FINAL_CENTERING: only the
                // horizontal pixel correction remains; descent is stopped.
                vz = 0.0;
                yaw_rate = 0.0;
                if (use_diagonal_approach) guidance_event = "FINAL_CENTERING";
            }
        } else {
            if (centered_hold.holding()) {
                std::cout << "[GUIDANCE] TargetCenteredHold 이탈 - target loss safety 적용"
                          << std::endl;
                guidance_event = "TARGET_CENTER_HOLD_EXIT_TARGET_LOSS";
            }
            guidance_diagnostics = guidance_force_zero(
                "target_loss_or_stale", last.observation_age_ms);
            // A centered lock is valid only while the confirmed target stays
            // fresh. Reset it before the existing target-loss timeout policy
            // is evaluated on subsequent cycles.
            centered_hold.reset();
            center_locking = false;
            center_locked = false;
            vx = 0.0;
            vy = 0.0;
            vz = 0.0;
        }

        std::string alt_status;
        // A lost/stale target and a centered hold are explicit zero-velocity
        // states. Do not let the independent altitude limiter overwrite them.
        if (link_fresh && tracking && !centered_hold.holding()) {
            double alt = last.altitude_m;
            if (alt > alt_limit.hard_limit_m) {
                if (!alt_enforcing) {
                    std::cout << "[ALT-LIMIT] altitude " << std::fixed << std::setprecision(2) << alt
                              << "m > hard_limit " << alt_limit.hard_limit_m << "m: 하강 보정 시작"
                              << std::endl;
                    alt_enforcing = true;
                }
                double overshoot = alt - alt_limit.hard_limit_m;
                vz = std::clamp(kAltGain * overshoot, kAltMinDescent, alt_limit.descent_speed_mps);
                alt_status = "DESCENDING";
            } else if (alt_enforcing) {
                if (alt <= alt_limit.hard_limit_m - alt_limit.recovery_margin_m) {
                    alt_enforcing = false;
                    alt_status = "RECOVERED";
                } else {
                    vz = kAltMinDescent;
                    alt_status = "SETTLING";
                }
            } else if (alt > alt_limit.soft_limit_m) {
                alt_status = "SOFT-LIMIT";
            }
        }

        preserve_diagonal_path_angle(vx, vy, vz);
        if (use_diagonal_approach && std::hypot(vx, vy) > 1e-9) {
            guidance_diagnostics.commanded_path_angle_rad =
                std::atan2(std::abs(vz), std::hypot(vx, vy));
        }

        const std::string command_safety_reason =
            !tracking ? "target_loss_or_stale" : alt_status;
        gcs_command_state_sender.send(
            static_cast<float>(vx), static_cast<float>(vy), static_cast<float>(vz),
            static_cast<float>(guidance_diagnostics.commanded_path_angle_rad),
            allow_mavlink_writes && health.armed,
            guidance_event.c_str(), command_safety_reason.c_str());

        commands.send_velocity_body(vx, vy, vz, yaw_rate);

        const char* center_state = centered_hold.holding()
                                       ? "HOLD"
                                       : (center_locked ? "LOCKED" : "ALIGN");
        std::cout << std::fixed << std::setprecision(2) << "t=" << elapsed_sec << "s "
                  << (tracking ? "TRACK" : "LOST ") << " dist=" << last.distance_m << "m vx=" << vx
                  << " vy=" << vy << " center=" << center_state
                  << " yaw_rate=" << yaw_rate << " alt=" << last.altitude_m << "m vz=" << vz
                  << " armed=" << (health.armed ? "Y" : "N");
        if (!alt_status.empty()) std::cout << " [" << alt_status << "]";
        if (health.have_position) {
            std::cout << " gps_moved=" << gps_moved_m << "m";
        }
        std::cout << std::endl;

        logger.log(elapsed_sec, mode_name, health.armed, tracking, last.distance_m, vx, yaw_rate, vz,
                   last.altitude_m, alt_status, health.battery_percent, health.battery_voltage_v,
                   health.fix_type, health.satellites, health.prearm_healthy, health.system_status, false,
                   "", guidance_event, health.lat, health.lon, gps_moved_m, last, vy,
                   guidance_diagnostics);

        std::this_thread::sleep_until(next_tick);
    }

    if (!resume_mode.empty()) {
        const std::string timeout_mode = resume_mode == "LOITER" ? "AUTO" : resume_mode;
        std::cout << "[AUTO_INTERCEPT] 접근 제한 시간(" << duration_sec << "초) 도달 - "
                  << timeout_mode << "로 복귀" << std::endl;
        commands.set_mode(timeout_mode);
    }
    return ApproachOutcome::kHandedBack;
}

void wait_for_auto_armed(MavConnection& vehicle, double max_heartbeat_gap_sec,
                         const TelemetryObserver& observe_telemetry,
                         unsigned required_heartbeats = 1) {
    auto last_heartbeat = std::chrono::steady_clock::now();
    auto last_status_print = last_heartbeat - std::chrono::seconds(10);
    unsigned stable_heartbeats = 0;
    while (true) {
        if (observe_telemetry) observe_telemetry(nullptr);
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration<double>(now - last_heartbeat).count() >
            max_heartbeat_gap_sec) {
            throw std::runtime_error("AUTO 대기 중 MAVLink heartbeat가 끊겼습니다.");
        }

        mavlink_message_t msg;
        if (vehicle.recv_match({MAVLINK_MSG_ID_HEARTBEAT, MAVLINK_MSG_ID_STATUSTEXT}, msg, 0.5)) {
            if (msg.msgid != MAVLINK_MSG_ID_HEARTBEAT) {
                if (observe_telemetry) observe_telemetry(&msg);
                continue;
            }
            mavlink_heartbeat_t heartbeat{};
            if (!is_valid_ardupilot_heartbeat(msg, &heartbeat)) continue;
            if (observe_telemetry) observe_telemetry(&msg);
            last_heartbeat = std::chrono::steady_clock::now();
            if (is_armed_from_heartbeat(heartbeat) &&
                heartbeat.custom_mode == copter_mode_mapping().at("AUTO")) {
                ++stable_heartbeats;
                if (stable_heartbeats >= required_heartbeats) return;
            } else {
                stable_heartbeats = 0;
            }
        }

        now = std::chrono::steady_clock::now();
        if (std::chrono::duration<double>(now - last_status_print).count() >= 5.0) {
            std::cout << "[AUTO_INTERCEPT] AUTO 전환 및 ARM 대기 중..." << std::endl;
            last_status_print = now;
            }
        }
    }
bool wait_for_lock(MavConnection& vehicle, TargetRangeReceiver& receiver,
                   const HealthLimit& health_limit, double link_stale_ms,
                   double lock_confirm_sec, double handoff_min_altitude_m,
                   HealthState& health, const TelemetryObserver& observe_telemetry) {
    auto last_valid = std::chrono::steady_clock::now() - std::chrono::seconds(10);
    auto lock_since = std::chrono::steady_clock::now();
    auto last_status_print = lock_since - std::chrono::seconds(10);
    TargetRangeMsg last{};
    bool locking = false;

    while (true) {
        if (observe_telemetry) observe_telemetry(nullptr);
        auto now = std::chrono::steady_clock::now();
        TargetRangeMsg target;
        if (receiver.poll(target)) {
            last = target;
            last_valid = now;
        }
        bool tracking =
            std::chrono::duration<double, std::milli>(now - last_valid).count() <=
                link_stale_ms &&
            last.valid && last.found;

        mavlink_message_t msg;
        if (vehicle.recv_match({MAVLINK_MSG_ID_SYS_STATUS, MAVLINK_MSG_ID_GPS_RAW_INT,
                                MAVLINK_MSG_ID_HEARTBEAT, MAVLINK_MSG_ID_EKF_STATUS_REPORT,
                                MAVLINK_MSG_ID_STATUSTEXT},
                               msg, 0.1)) {
            if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT &&
                !is_valid_ardupilot_heartbeat(msg)) {
                continue;
            }
            if (observe_telemetry) observe_telemetry(&msg);
            apply_health_message(msg, health, now);
        }

        double heartbeat_gap =
            std::chrono::duration<double>(now - health.last_heartbeat).count();
        if (heartbeat_gap > health_limit.max_heartbeat_gap_sec) {
            std::cerr << "[AUTO_INTERCEPT] heartbeat가 끊겨 탐지 대기를 중단합니다."
                      << std::endl;
            return false;
        }
        if (health.have_armed &&
            (!health.armed || health.custom_mode != copter_mode_mapping().at("AUTO"))) {
            return false;
        }

        bool auto_armed = health.have_armed && health.armed &&
                          health.custom_mode == copter_mode_mapping().at("AUTO");
        bool healthy = auto_armed &&
                       evaluate_health_breach(health, health_limit, now).empty();
        const bool altitude_ready = last.altitude_valid &&
                                    last.altitude_m >= handoff_min_altitude_m;
        if (tracking && healthy && altitude_ready) {
            if (!locking) {
                locking = true;
                lock_since = now;
            }
            if (std::chrono::duration<double>(now - lock_since).count() >= lock_confirm_sec) {
                return true;
            }
        } else {
            locking = false;
        }

        if (std::chrono::duration<double>(now - last_status_print).count() >= 5.0) {
            std::cout << "[AUTO_INTERCEPT] AUTO 비행 중 - 타겟 대기 (tracking="
                      << (tracking ? "Y" : "N") << " healthy=" << (healthy ? "Y" : "N")
                      << " altitude=" << std::fixed << std::setprecision(2)
                      << last.altitude_m << "m/"
                      << (altitude_ready ? "ready" : "below-handoff-min")
                      << ")" << std::endl;
            last_status_print = now;
        }
    }
}

void run_auto_intercept(const YamlValue& track, const AltitudeLimit& alt_limit,
                        const HealthLimit& health_limit,
                        autopilot::CommandSender& commands,
                        safety::ControlAuthority& control_authority,
                        const TelemetryObserver& observe_telemetry,
                        const std::string& target_loss_resume_mode,
                        safety::FlightStateMachine* flight_state = nullptr,
                        bool allow_mavlink_writes = true) {
    const double link_stale_ms = track.get_double_or("link_stale_ms", 500.0);
    const double lock_confirm_sec = track.get_double_or("lock_confirm_sec", 1.0);
    const double handoff_min_altitude_m =
        track.get_double_or("handoff_min_altitude_m", 4.5);
    const double intercept_duration_sec =
        track.get_double_or("intercept_approach_duration_sec", 30.0);
    const double reacquire_cooldown_sec =
        track.get_double_or("reacquire_cooldown_sec", 3.0);
    const int udp_port = static_cast<int>(track.get_long_or("udp_port", 15020));

    MavConnection& vehicle = drone::require_connection();
    if (allow_mavlink_writes) {
        request_message_interval(vehicle, MAVLINK_MSG_ID_SYS_STATUS, health_limit.poll_rate_hz);
        request_message_interval(vehicle, MAVLINK_MSG_ID_GPS_RAW_INT, health_limit.poll_rate_hz);
        request_message_interval(vehicle, MAVLINK_MSG_ID_EKF_STATUS_REPORT, health_limit.poll_rate_hz);
    }

    std::cout << "AUTO->GUIDED 타겟 인계 시작 (잠금 " << lock_confirm_sec
              << "초, 인계 최소고도 " << handoff_min_altitude_m
              << "m, 접근 제한 " << intercept_duration_sec << "초)" << std::endl;

    while (true) {
        wait_for_auto_armed(vehicle, health_limit.max_heartbeat_gap_sec, observe_telemetry);
        std::cout << "[AUTO_INTERCEPT] AUTO + ARMED 확인, 타겟을 기다립니다." << std::endl;

        HealthState health;
        health.last_heartbeat = std::chrono::steady_clock::now();
        bool locked = false;
        {
            TargetRangeReceiver receiver(udp_port);
            locked = wait_for_lock(vehicle, receiver, health_limit, link_stale_ms,
                                   lock_confirm_sec, handoff_min_altitude_m, health,
                                   observe_telemetry);
        }
        if (!locked) continue;

        // wait_for_lock returns only after a fresh, confirmed target is
        // present at the configured handoff altitude. Publish TARGET_SEARCH
        // at that evidence point, rather than at the earlier AUTO+ARM check.
        if (flight_state != nullptr && flight_state->current() == safety::FlightState::ARMED_TAKEOFF) {
            flight_state->transition(safety::FlightState::TARGET_SEARCH,
                                     "목표 고도 및 표적 확인, GUIDED 전환 준비");
        }

        std::cout << "[AUTO_INTERCEPT] 타겟 확정 - GUIDED로 전환합니다." << std::endl;
        if (!control_authority.prepare_mode_request("GUIDED")) {
            std::cerr << "[AUTO_INTERCEPT] GUIDED 예상 모드 등록 실패" << std::endl;
            continue;
        }
        if (!commands.set_mode("GUIDED")) continue;
        try {
            wait_for_heartbeat(vehicle, 5.0, health_limit.max_heartbeat_gap_sec,
                               "GUIDED 전환",
                               [](const mavlink_heartbeat_t& heartbeat) {
                                   return heartbeat.custom_mode ==
                                          copter_mode_mapping().at("GUIDED");
                               },
                               observe_telemetry);
        } catch (const std::exception& e) {
            std::cerr << "[AUTO_INTERCEPT] " << e.what() << " - AUTO 복귀 시도"
                      << std::endl;
            commands.set_mode("AUTO");
            continue;
        }

        // A SET_MODE write is only a request. Publish GUIDED after the
        // validated ArduPilot heartbeat confirms the actual mode.
        if (flight_state != nullptr && flight_state->current() == safety::FlightState::TARGET_SEARCH) {
            flight_state->transition(safety::FlightState::GUIDED,
                                     "GUIDED heartbeat 확인");
        }

        ApproachOutcome outcome = approach_target(track, alt_limit, health_limit, commands,
                                                   observe_telemetry, intercept_duration_sec,
                                                   target_loss_resume_mode, flight_state,
                                                   allow_mavlink_writes);
        if (outcome == ApproachOutcome::kLanded) return;
        if (outcome == ApproachOutcome::kHeld) {
            std::cout << "[AUTO_INTERCEPT] 표적 유실 후 LOITER 유지. 자동 임무를 재개하지 않습니다."
                      << std::endl;
            return;
        }

        wait_for_heartbeat(vehicle, 5.0, health_limit.max_heartbeat_gap_sec, "AUTO 복귀",
                           [](const mavlink_heartbeat_t& heartbeat) {
                               return heartbeat.custom_mode ==
                                      copter_mode_mapping().at("AUTO");
                           },
                           observe_telemetry);
        std::cout << "[AUTO_INTERCEPT] AUTO 복귀 확인, " << reacquire_cooldown_sec
                  << "초 후 재탐지합니다." << std::endl;
        std::this_thread::sleep_for(std::chrono::duration<double>(reacquire_cooldown_sec));
    }
}

}  // namespace

int main(int argc, char** argv) {
    bool self_launch = false;
    for (int i = 1; i < argc; ++i) {
        const std::string arg(argv[i]);
        if (arg == "--self-launch") {
            self_launch = true;
        } else if (arg != "--auto-intercept") {
            std::cerr << "알 수 없는 옵션: " << arg
                      << " (지원: --auto-intercept, --self-launch)" << std::endl;
            return 2;
        }
    }

    try {
        YamlValue settings = drone::load_mavlink_settings();
        YamlValue track = settings["target_track"];

        AltitudeLimit alt_limit;
        HealthLimit health_limit;
        try {
            YamlValue safety = drone::load_safety_settings();

            YamlValue al = safety["altitude_limit"];
            alt_limit.soft_limit_m = al.get_double_or("soft_limit_m", alt_limit.soft_limit_m);
            alt_limit.hard_limit_m = al.get_double_or("hard_limit_m", alt_limit.hard_limit_m);
            alt_limit.descent_speed_mps =
                al.get_double_or("descent_speed_mps", alt_limit.descent_speed_mps);
            alt_limit.recovery_margin_m =
                al.get_double_or("recovery_margin_m", alt_limit.recovery_margin_m);
            health_limit.poll_rate_hz = al.get_double_or("poll_rate_hz", health_limit.poll_rate_hz);

            YamlValue battery_limit = safety["battery_limit"];
            health_limit.min_battery_percent =
                battery_limit.get_long_or("min_percent", health_limit.min_battery_percent);
            health_limit.min_battery_voltage_v =
                battery_limit.get_double_or("min_voltage_v", health_limit.min_battery_voltage_v);

            YamlValue heartbeat_limit = safety["heartbeat_limit"];
            health_limit.max_heartbeat_gap_sec =
                heartbeat_limit.get_double_or("max_gap_sec", health_limit.max_heartbeat_gap_sec);
            health_limit.land_gap_sec =
                heartbeat_limit.get_double_or("land_gap_sec", health_limit.land_gap_sec);

            YamlValue gps_limit = safety["gps_limit"];
            health_limit.min_fix_type = gps_limit.get_long_or("min_fix_type", health_limit.min_fix_type);
            health_limit.min_satellites =
                gps_limit.get_long_or("min_satellites", health_limit.min_satellites);

            YamlValue vehicle_health_limit = safety["vehicle_health_limit"];
            health_limit.require_prearm_healthy = get_bool_or(
                vehicle_health_limit, "require_prearm_healthy", health_limit.require_prearm_healthy);
            health_limit.require_normal_state = get_bool_or(
                vehicle_health_limit, "require_normal_state", health_limit.require_normal_state);

            YamlValue ekf_limit = safety["ekf_limit"];
            health_limit.ekf_pos_horiz_variance_max = ekf_limit.get_double_or(
                "pos_horiz_variance_max", health_limit.ekf_pos_horiz_variance_max);
            health_limit.ekf_velocity_variance_max = ekf_limit.get_double_or(
                "velocity_variance_max", health_limit.ekf_velocity_variance_max);
        } catch (const std::exception& e) {
            std::cerr << "Warning: could not fully load setting/safety.yaml (" << e.what()
                      << "), missing sections fall back to built-in defaults." << std::endl;
        }

        const autopilot::RuntimeTarget runtime_target =
            autopilot::runtime_target_from_environment();
        const autopilot::RuntimeTransportConfig transport =
            autopilot::load_runtime_transport(runtime_target,
                                               autopilot::TransportRole::CommandOwner);
        const autopilot::RuntimeTransportConfig telemetry_transport =
            autopilot::load_runtime_transport(runtime_target,
                                               autopilot::TransportRole::TelemetrySubscriber);
        if (runtime_target == autopilot::RuntimeTarget::Real &&
            transport.command_mode == autopilot::CommandMode::Flight) {
            autopilot::RealFlightOptions options;
            if (const char* value = std::getenv("ASTRODRONE_ROUTER_SERIAL")) {
                options.router_serial = value;
            }
            options.command_endpoint = transport.endpoint;
            options.telemetry_endpoint = telemetry_transport.endpoint;
            options.allow_arm = env_flag_enabled("ASTRODRONE_ALLOW_ARM");
            options.confirm_real_flight = env_flag_enabled("ASTRODRONE_CONFIRM_REAL_FLIGHT");
            options.commands_enabled = env_flag_enabled("ASTRODRONE_COMMANDS_ENABLED");
            options.router_ownership_confirmed =
                env_flag_enabled("ASTRODRONE_ROUTER_OWNERSHIP_CONFIRMED");
            const auto decision = autopilot::validate_real_flight(transport, options);
            if (!decision.allowed) {
                throw std::runtime_error("real-flight guard: " + decision.reason);
            }
        }
        std::cerr << "runtime target=" << autopilot::runtime_target_name(runtime_target)
                  << " command endpoint=" << transport.endpoint << std::endl;
        drone::connect(transport.endpoint, transport.baud);
        MavConnection& vehicle = drone::require_connection();
        autopilot::CommandGate command_gate;
        autopilot::CommandAuthority command_authority;
        command_authority.automation_enabled = true;
        command_authority.vehicle_commands_enabled = transport.allow_vehicle_commands;
        command_gate.set_authority(command_authority);
        command_gate.set_preflight_required(true);
        command_gate.set_preflight_ready(false);
        safety::ControlAuthority control_authority;
        logging::CommandAuditLogger command_audit_logger;
        safety::FlightStateMachine flight_state(
            [&](const safety::FlightStateEvent& event) {
                command_audit_logger.record_state_event(event);
            });

        mavlink_heartbeat_t initial_heartbeat{};
        if (!vehicle.wait_heartbeat(5.0, &initial_heartbeat)) {
            throw std::runtime_error("자동 제어 시작 전 fresh HEARTBEAT를 받지 못했습니다.");
        }
        control_authority.observe_heartbeat(initial_heartbeat);

        const TelemetryObserver observe_telemetry = [&](const mavlink_message_t* message) {
            const bool was_control_locked = control_authority.control_locked();
            if (message && message->msgid == MAVLINK_MSG_ID_HEARTBEAT) {
                mavlink_heartbeat_t heartbeat;
                if (!is_valid_ardupilot_heartbeat(*message, &heartbeat)) {
                    control_authority.poll();
                    control_authority.apply_to(command_gate);
                    return;
                }
                control_authority.observe_heartbeat(heartbeat);
            } else if (message && message->msgid == MAVLINK_MSG_ID_STATUSTEXT) {
                mavlink_statustext_t status_text;
                mavlink_msg_statustext_decode(message, &status_text);
                std::string text(status_text.text, sizeof(status_text.text));
                const auto nul = text.find('\0');
                if (nul != std::string::npos) text.resize(nul);
                control_authority.observe_failsafe_evidence(text);
            }
            control_authority.poll();
            control_authority.apply_to(command_gate);
            if (!was_control_locked && control_authority.control_locked()) {
                command_audit_logger.record_control_lock(
                    safety::control_lock_reason_name(control_authority.lock_reason()),
                    control_authority.snapshot());
            }
        };

        autopilot::CommandSender commands(
            vehicle, command_gate, [&](const autopilot::CommandDecision& decision) {
                command_audit_logger.record_decision(decision, control_authority.snapshot());
                if (!decision.allowed) {
                    std::cerr << "[COMMAND_GATE] 차단: "
                              << autopilot::command_type_name(decision.type) << " ("
                              << autopilot::gate_block_reason_name(decision.block_reason);
                    if (!decision.control_lock_reason.empty()) {
                        std::cerr << ", reason=" << decision.control_lock_reason;
                    }
                    std::cerr << ")" << std::endl;
                }
            },
            [&](const autopilot::CommandRequest& request, bool write_succeeded) {
                control_authority.record_mode_request(request, write_succeeded);
            });

        // Telemetry configuration is intentionally outside CommandGate. It
        // is needed to collect the evidence required by the preflight gate;
        // no mode/ARM/takeoff/velocity/LAND command can pass yet.
        if (transport.allow_telemetry_configuration) {
            request_message_interval(vehicle, MAVLINK_MSG_ID_SYS_STATUS, health_limit.poll_rate_hz);
            request_message_interval(vehicle, MAVLINK_MSG_ID_GPS_RAW_INT, health_limit.poll_rate_hz);
            request_message_interval(vehicle, MAVLINK_MSG_ID_EKF_STATUS_REPORT, health_limit.poll_rate_hz);
        }

        safety::PreflightPolicy preflight_policy;
        preflight_policy.require_rc_policy = env_flag_enabled("REQUIRE_RC_PREFLIGHT");
        preflight_policy.require_vision = env_flag_enabled("REQUIRE_VISION_PREFLIGHT");
        safety::PreflightGate preflight(preflight_policy);
        safety::HealthMonitor preflight_health;
        wait_for_preflight(vehicle, health_limit, preflight, preflight_health,
                           observe_telemetry);
        command_gate.set_preflight_ready(true);
        write_marker_file_from_env("AUTONOMY_PREFLIGHT_READY_FILE", "preflight_ready");
        std::cout << "Preflight passed: HEARTBEAT/GPS/EKF/battery/telemetry stable." << std::endl;

        if (transport.command_mode == autopilot::CommandMode::Shadow) {
            // Shadow performs the real telemetry/target/guidance calculation,
            // while CommandGate blocks every vehicle-affecting request. It
            // deliberately does not enter AUTO/GUIDED setup.
            std::cout << "[RUNTIME] shadow mode: guidance calculation only; vehicle commands blocked."
                      << std::endl;
            approach_target(track, alt_limit, health_limit, commands, observe_telemetry,
                            track.get_double_or("approach_duration_sec", 60.0), "", &flight_state,
                            false);
            return 0;
        }

        if (!self_launch) {
            // A lost target must not cause the vehicle to resume the mission
            // and fly away. Default to LOITER; AUTO remains available only as
            // an explicit compatibility override for SITL experiments.
            std::string target_loss_resume_mode = "LOITER";
            if (const char* policy = std::getenv("SITL_TARGET_LOSS_POLICY")) {
                const std::string value(policy);
                if (value == "LAND") {
                    target_loss_resume_mode.clear();
                } else if (value == "AUTO" || value == "LOITER") {
                    target_loss_resume_mode = value;
                } else if (!value.empty()) {
                    throw std::runtime_error(
                        "SITL_TARGET_LOSS_POLICY must be AUTO, LOITER, or LAND");
                }
            }
            // The launcher performs mission upload/AUTO/ARM only after the
            // preflight marker is present. Keep the control authority session
            // unopened while those setup mode changes are still in progress.
            wait_for_marker_file_from_env("AUTONOMY_MISSION_SETUP_COMPLETE_FILE",
                                          "mission_setup_complete");
            // Require two fresh AUTO+ARM heartbeats before rebasing authority.
            wait_for_auto_armed(vehicle, health_limit.max_heartbeat_gap_sec,
                                observe_telemetry, 2);
            if (flight_state.current() == safety::FlightState::AUTO_WAIT) {
                flight_state.transition(safety::FlightState::ARMED_TAKEOFF,
                                        "실제 AUTO+ARM heartbeat 확인");
            }
            if (!control_authority.begin_automation_session(
                    copter_mode_mapping().at("AUTO"))) {
                throw std::runtime_error(
                    "AUTO+ARM 안정 heartbeat 이후 authority session 시작에 실패했습니다.");
            }
            command_audit_logger.record_session_started(control_authority.snapshot());
            run_auto_intercept(track, alt_limit, health_limit, commands, control_authority,
                               observe_telemetry,
                               target_loss_resume_mode, &flight_state,
                               transport.allow_mavlink_writes);
            return 0;
        }

        if (!commands.set_mode("GUIDED")) {
            throw std::runtime_error("GUIDED 모드 전환 명령 전송 실패 (지원하지 않는 모드)");
        }
        wait_for_heartbeat(vehicle, 5.0, health_limit.max_heartbeat_gap_sec, "GUIDED 모드 전환",
                            [](const mavlink_heartbeat_t& hb) {
                                return hb.custom_mode == copter_mode_mapping().at("GUIDED");
                            },
                            observe_telemetry);
        std::cout << "GUIDED 모드 확인됨." << std::endl;

        commands.arm_disarm(true);
        wait_for_heartbeat(vehicle, 5.0, health_limit.max_heartbeat_gap_sec, "시동 확인",
                           [](const mavlink_heartbeat_t& heartbeat) {
                               return is_armed_from_heartbeat(heartbeat);
                           },
                           observe_telemetry);
        control_authority.begin_automation_session();
        constexpr double kSelfLaunchAltitudeM = 4.5;
        commands.takeoff(kSelfLaunchAltitudeM);
        wait_for_heartbeat(vehicle, 10.0, health_limit.max_heartbeat_gap_sec, "이륙 대기", nullptr,
                           observe_telemetry);

        ApproachOutcome outcome = approach_target(
            track, alt_limit, health_limit, commands, observe_telemetry,
            track.get_double_or("approach_duration_sec", 60.0), "LOITER", &flight_state,
            transport.allow_mavlink_writes);

        if (outcome == ApproachOutcome::kHeld) return 0;
        if (outcome != ApproachOutcome::kLanded) {
            flight_state.transition(safety::FlightState::LANDING, "LAND command requested");
            commands.land();
            std::cout << "착륙 중..." << std::endl;
        }
        return 0;
    } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return 1;
    }
}
