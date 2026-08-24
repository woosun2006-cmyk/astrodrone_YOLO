#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cerrno>
#include <cstring>
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

#include "flight_mission_app.hpp"
#include "autopilot/autopilot_mavlink_adapter.hpp"
#include "drone_lib.hpp"
#include "runtime_config.hpp"
#include "logging/command_audit_logger.hpp"
#include "mission/diagonal_approach_algorithm.hpp"
#include "mission/mission_setup.hpp"
#include "mission/centered_vertical_descent_algorithm.hpp"
#include "pos_calculator.hpp"
#include "preflight_readiness.hpp"
#include "safety/safety_monitor.hpp"
#include "target_link.hpp"

namespace {

constexpr double kControlConfidenceComparisonEpsilon = 1e-6;

bool meets_control_confidence(float confidence, double minimum_confidence) {
    return std::isfinite(confidence) &&
           static_cast<double>(confidence) + kControlConfidenceComparisonEpsilon >=
               minimum_confidence;
}

using HealthLimit = safety::HealthLimit;
using HealthState = safety::HealthState;
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
        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: return "GLOBAL_POSITION_INT";
        case MAVLINK_MSG_ID_LOCAL_POSITION_NED: return "LOCAL_POSITION_NED";
        case MAVLINK_MSG_ID_ALTITUDE: return "ALTITUDE";
        case MAVLINK_MSG_ID_EKF_STATUS_REPORT: return "EKF_STATUS_REPORT";
        case MAVLINK_MSG_ID_SYS_STATUS: return "SYS_STATUS";
        case MAVLINK_MSG_ID_RC_CHANNELS: return "RC_CHANNELS";
        case MAVLINK_MSG_ID_ATTITUDE: return "ATTITUDE";
        case MAVLINK_MSG_ID_RAW_IMU: return "RAW_IMU";
        case MAVLINK_MSG_ID_HIGHRES_IMU: return "HIGHRES_IMU";
        case MAVLINK_MSG_ID_VIBRATION: return "VIBRATION";
        case MAVLINK_MSG_ID_STATUSTEXT: return "STATUSTEXT";
        case MAVLINK_MSG_ID_COMMAND_ACK: return "COMMAND_ACK";
        case MAVLINK_MSG_ID_HOME_POSITION: return "HOME_POSITION";
        case MAVLINK_MSG_ID_MISSION_CURRENT: return "MISSION_CURRENT";
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
// vehicle-affecting flight command and remains outside SafetyMonitor.
void request_message_interval(autopilot::AutopilotMavlinkAdapter& adapter,
                              uint32_t message_id, double frequency_hz) {
    if (!adapter.request_message_interval(message_id, frequency_hz)) {
        throw std::runtime_error("텔레메트리 설정이 실행 정책에 의해 차단되었습니다.");
    }
}

std::string mode_string(uint32_t custom_mode) {
    for (const auto& entry : copter_mode_mapping()) {
        if (entry.second == custom_mode) return entry.first;
    }
    return "Mode(" + std::to_string(custom_mode) + ")";
}

const char* phase_display_name(app::FlightPhase phase) {
    switch (phase) {
        case app::FlightPhase::PRECHECK: return "비행 사전 안전검사";
        case app::FlightPhase::TAKEOFF: return "이륙";
        case app::FlightPhase::TARGET_SEARCH: return "표적 탐색";
        case app::FlightPhase::GUIDED: return "GUIDED 접근";
        case app::FlightPhase::REACQUIRE: return "표적 재탐지 대기";
        case app::FlightPhase::SAFE_HOVER: return "안전 호버";
        case app::FlightPhase::LANDING: return "LANDING";
        case app::FlightPhase::FINISHED: return "종료";
    }
    return "알 수 없는 상태";
}

void log_loiter_rc3(const autopilot::AutopilotState& state) {
    if (!state.have_rc || state.rc_channel_count < 3) {
        std::cout << "[경고] LOITER 진입 전 RC3를 확인할 수 없습니다."
                  << " RC telemetry가 없습니다." << std::endl;
        return;
    }
    const uint16_t rc3 = state.rc_channels[2];
    std::cout << "[정보] LOITER 진입 전 RC3=" << rc3 << std::endl;
    const char* target = std::getenv("ASTRODRONE_TARGET");
    if (target != nullptr && std::string(target) == "real" && rc3 != 1500) {
        std::cout << "[경고] 실기체 RC3가 중립값이 아닙니다."
                  << " RC3=" << rc3 << " LOITER 전환 전 운용자 확인이 필요합니다."
                  << std::endl;
    }
}

// target-distance publishes only the fresh YOLO pixel observation. Keep the
// vehicle-dependent range calculation at the single telemetry owner: the
// FlightMissionApp consumes AutopilotState from its Adapter and enriches the
// observation before any guidance or safety decision uses it.
void enrich_target_observation(TargetRangeMsg& observation,
                               const autopilot::AutopilotState& state,
                               const YamlValue& track,
                               std::chrono::steady_clock::time_point now,
                               double* previous_distance_m = nullptr,
                               bool* have_previous_distance = nullptr) {
    if (!state.have_altitude) {
        observation.altitude_valid = 0;
        observation.valid = 0;
        return;
    }

    observation.altitude_m = static_cast<float>(state.altitude_m);
    const double altitude_stale_ms = track.get_double_or("altitude_stale_ms", 1000.0);
    const double altitude_age_ms = state.altitude_updated_at ==
                                           autopilot::AutopilotState::Clock::time_point{}
                                       ? std::numeric_limits<double>::infinity()
                                       : std::chrono::duration<double, std::milli>(
                                             now - state.altitude_updated_at)
                                             .count();
    observation.altitude_valid = std::isfinite(altitude_age_ms) &&
                                 altitude_age_ms <= altitude_stale_ms;

    if (!observation.found || !observation.altitude_valid ||
        !std::isfinite(observation.x_px) || !std::isfinite(observation.y_px)) {
        observation.valid = 0;
        return;
    }

    const PixelCalibration calibration{
        track.get_double_or("pixel_focal_length_x_px",
                            track.get_double_or("pixel_focal_length_px", 530.0)),
        track.get_double_or("pixel_focal_length_y_px",
                            track.get_double_or("pixel_focal_length_px", 530.0)),
        track.get_double_or("pixel_principal_point_x_px", 320.0),
        track.get_double_or("pixel_principal_point_y_px", 240.0)};
    const auto body_offset = downward_pixel_to_body_offset(
        observation.x_px, observation.y_px, observation.altitude_m, calibration);
    observation.body_forward_m = static_cast<float>(body_offset.forward_m);
    observation.body_right_m = static_cast<float>(body_offset.right_m);
    observation.ground_offset_m = static_cast<float>(
        std::hypot(body_offset.forward_m, body_offset.right_m));
    observation.distance_m = static_cast<float>(std::hypot(
        observation.ground_offset_m, static_cast<double>(observation.altitude_m)));

    const double max_distance_m = track.get_double_or("max_plausible_distance_m", 50.0);
    const double max_jump_m = track.get_double_or("max_distance_jump_m", 5.0);
    const double distance = observation.distance_m;
    const bool plausible = std::isfinite(distance) && distance >= 0.0 &&
                           distance <= max_distance_m;
    const bool continuous = previous_distance_m == nullptr ||
                            have_previous_distance == nullptr ||
                            !*have_previous_distance ||
                            std::abs(distance - *previous_distance_m) <= max_jump_m;
    if (!plausible || !continuous) {
        observation.valid = 0;
        return;
    }

    observation.valid = 1;
    if (previous_distance_m != nullptr && have_previous_distance != nullptr) {
        *previous_distance_m = distance;
        *have_previous_distance = true;
    }
}

bool target_control_eligible(const TargetRangeMsg& observation, bool link_fresh,
                             double minimum_confidence) {
    const size_t class_length = strnlen(observation.class_name,
                                        sizeof(observation.class_name));
    return link_fresh && observation.valid && observation.found &&
           observation.altitude_valid && std::isfinite(observation.altitude_m) &&
           std::isfinite(observation.observation_age_ms) &&
           observation.observation_age_ms >= 0.0 &&
           meets_control_confidence(observation.target_confidence, minimum_confidence) &&
           class_length > 0;
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
void wait_for_heartbeat(autopilot::AutopilotMavlinkAdapter& vehicle,
                         double timeout_sec, double max_heartbeat_gap_sec,
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

bool process_alive_from_environment(const char* name) {
    const char* value = std::getenv(name);
    if (value == nullptr || *value == '\0') return false;
    try {
        const pid_t pid = static_cast<pid_t>(std::stol(value));
        return pid > 0 && (::kill(pid, 0) == 0 || errno == EPERM);
    } catch (...) {
        return false;
    }
}

// Preflight is deliberately a read-only telemetry phase. It does not send a
// vehicle command; the only command sent before it is the existing telemetry
// interval request. Vehicle-affecting commands stay blocked by SafetyMonitor
// until this function returns successfully.
void wait_for_preflight(
    autopilot::AutopilotMavlinkAdapter& vehicle, const HealthLimit& health_limit,
    safety::SafetyMonitor& preflight, const TelemetryObserver& observe_telemetry,
    double timeout_sec = 30.0) {
    using Clock = std::chrono::steady_clock;
    const auto start = Clock::now();
    std::map<uint32_t, PreflightMessageDiagnostic> received_messages;
    const bool verbose = env_flag_enabled("PREFLIGHT_VERBOSE");
    const std::string camera_marker_path =
        std::getenv("CAMERA_FRAME_READY_FILE") != nullptr
            ? std::getenv("CAMERA_FRAME_READY_FILE")
            : "";
    const std::string yolo_endpoint =
        std::getenv("YOLO_HTTP_ENDPOINT") != nullptr
            ? std::getenv("YOLO_HTTP_ENDPOINT")
            : "http://127.0.0.1:8002";
    const std::string yolo_ready_marker_path =
        std::getenv("YOLO_READY_FILE") != nullptr
            ? std::getenv("YOLO_READY_FILE")
            : "";
    const std::string camera_source_marker_path =
        std::getenv("CAMERA_SOURCE_READY_FILE") != nullptr
            ? std::getenv("CAMERA_SOURCE_READY_FILE")
            : "";
    const double camera_stale_timeout_sec = 1.0;
    app::CameraFrameReadiness latest_camera;
    app::VisionReadiness latest_vision;
    bool latest_http_alive = false;
    const auto update_vision_readiness = [&]() {
        latest_camera = app::read_camera_frame_readiness(
            camera_marker_path, Clock::now(), camera_stale_timeout_sec);
        latest_http_alive = app::http_endpoint_alive(yolo_endpoint);
        const bool yolo_marker_present = !yolo_ready_marker_path.empty() &&
                                          ::access(yolo_ready_marker_path.c_str(), F_OK) == 0;
        const bool camera_source_ready = !camera_source_marker_path.empty() &&
                                         ::access(camera_source_marker_path.c_str(), F_OK) == 0;
        latest_vision = app::evaluate_vision_readiness(
            process_alive_from_environment("YOLO_PROCESS_PID"), latest_http_alive,
            yolo_marker_present,
            camera_source_ready,
            latest_camera);
    };
    const auto print_diagnostic_summary = [&](bool force) {
        if (!force && !verbose) return;
        const char* prefix = force ? "[사전검사 실패]" : "[사전검사 상세]";
        for (uint32_t message_id : {MAVLINK_MSG_ID_HEARTBEAT, MAVLINK_MSG_ID_GPS_RAW_INT,
                                    MAVLINK_MSG_ID_EKF_STATUS_REPORT, MAVLINK_MSG_ID_SYS_STATUS}) {
            const auto it = received_messages.find(message_id);
            if (it == received_messages.end()) {
                std::cerr << prefix << " 수신 없음 메시지=" << preflight_message_name(message_id)
                          << std::endl;
                continue;
            }
            const auto& diagnostic = it->second;
            const double age_sec = std::chrono::duration<double>(
                Clock::now() - diagnostic.last_at).count();
            std::cerr << prefix << " 메시지=" << preflight_message_name(message_id)
                      << " 마지막수신지연=" << std::fixed << std::setprecision(3) << age_sec << "초"
                      << " 시스템=" << static_cast<int>(diagnostic.last_sysid)
                      << " 컴포넌트=" << static_cast<int>(diagnostic.last_compid);
            if (verbose) std::cerr << " 누적수신=" << diagnostic.count;
            std::cerr << std::endl;
        }
        const auto& health = vehicle.state();
        const auto age_ms = [&](Clock::time_point updated_at) {
            if (updated_at == Clock::time_point{}) return -1.0;
            return std::chrono::duration<double, std::milli>(
                       Clock::now() - updated_at).count();
        };
        std::cerr << prefix << " GPS수신=" << (health.have_gps ? "예" : "아니오")
                  << " fix=" << static_cast<int>(health.fix_type)
                  << " 위성=" << static_cast<int>(health.satellites)
                  << " GPS지연=" << age_ms(health.gps_updated_at) << "ms"
                  << " EKF수신=" << (health.have_ekf ? "예" : "아니오")
                  << " EKF수평분산=" << health.ekf_pos_horiz_variance
                  << " EKF속도분산=" << health.ekf_velocity_variance
                  << " EKF지연=" << age_ms(health.ekf_updated_at) << "ms"
                  << " 배터리수신=" << (health.have_battery ? "예" : "아니오")
                  << " 배터리유효=" << (health.battery_valid ? "예" : "아니오")
                  << " 전압=" << health.battery_voltage_v << "V"
                  << " 잔량=" << health.battery_percent << "%"
                  << " 배터리지연=" << age_ms(health.battery_updated_at) << "ms"
                  << " HEARTBEAT지연=" << age_ms(health.last_heartbeat) << "ms"
                  << std::endl;
        std::cerr << prefix << " YOLO/카메라 준비="
                  << (latest_vision.ready ? "통과" : "실패")
                  << " 사유=" << (latest_vision.reason.empty() ? "없음" : latest_vision.reason)
                  << " HTTP=" << (latest_http_alive ? "정상" : "실패")
                  << " frame_sequence=" << latest_camera.frame_sequence
                  << " frame_timestamp_ns=" << latest_camera.frame_timestamp_ns
                  << " frame_age=" << latest_camera.age_sec << "초"
                  << " 크기=" << latest_camera.frame_width << "x" << latest_camera.frame_height
                  << " source=" << (latest_camera.source.empty() ? "없음" : latest_camera.source)
                  << std::endl;
    };
    if (verbose) {
        std::cerr << "[사전검사 상세] MAVLink 상태 수신을 시작합니다: HEARTBEAT, GPS_RAW_INT, "
                  << "EKF_STATUS_REPORT, SYS_STATUS" << std::endl;
    }
    const bool require_rc = env_flag_enabled("REQUIRE_RC_PREFLIGHT");
    bool rc_seen = false;
    update_vision_readiness();

    safety::PreflightSample sample;
    sample.heartbeat_event = true;
    sample.valid_autopilot_heartbeat = true;  // initial wait_heartbeat() already validated it
    sample.heartbeat_fresh = true;
    sample.rc_policy_ok = !require_rc;
    sample.yolo_ready = latest_vision.ready;
    sample.camera_frame = latest_vision.ready;
    sample.now = start;
    preflight.observe_preflight(sample);

    while (!preflight.preflight_ready()) {
        const auto now = Clock::now();
        if (std::chrono::duration<double>(now - start).count() >= timeout_sec) {
            update_vision_readiness();
            print_diagnostic_summary(true);
            std::string reason = "preflight timeout";
            for (const auto& item : preflight.preflight_reasons()) reason += "; " + item;
            if (!latest_vision.ready) reason += "; " + latest_vision.reason;
            throw std::runtime_error(reason);
        }

        sample = safety::PreflightSample{};
        sample.now = now;
        sample.rc_policy_ok = !require_rc || rc_seen;
        update_vision_readiness();
        sample.yolo_ready = latest_vision.ready;
        sample.camera_frame = latest_vision.ready;
        const auto previous_heartbeat = vehicle.state().last_heartbeat;
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
            if (verbose && (diagnostic.count == 1 || diagnostic.count % 10 == 0)) {
                std::cerr << "[사전검사 상세] 수신 메시지=" << preflight_message_name(message.msgid)
                          << " 시스템=" << static_cast<int>(message.sysid)
                          << " 컴포넌트=" << static_cast<int>(message.compid)
                          << " 누적횟수=" << diagnostic.count << std::endl;
            }
            if (observe_telemetry) observe_telemetry(&message);
        }

        const auto& state = vehicle.state();
        sample.heartbeat_event = state.last_heartbeat != previous_heartbeat &&
                                 state.last_heartbeat != Clock::time_point{};
        sample.valid_autopilot_heartbeat = sample.heartbeat_event;
        rc_seen = state.have_rc;
        sample.gps_ok = state.have_gps && state.fix_type >= health_limit.min_fix_type &&
                        (state.satellites == 255 || state.satellites >= health_limit.min_satellites);
        sample.ekf_ok = state.have_ekf &&
                       state.ekf_pos_horiz_variance <= health_limit.ekf_pos_horiz_variance_max &&
                       state.ekf_velocity_variance <= health_limit.ekf_velocity_variance_max;
        sample.battery_valid = state.battery_valid &&
                               state.battery_voltage_v >= health_limit.min_battery_voltage_v &&
                               state.battery_percent >= health_limit.min_battery_percent;
        sample.prearm_healthy = state.have_sys_status && state.prearm_healthy;
        const auto fresh = [&](Clock::time_point updated_at) {
            return updated_at != Clock::time_point{} &&
                   std::chrono::duration<double>(now - updated_at).count() <=
                       health_limit.max_heartbeat_gap_sec;
        };
        sample.heartbeat_fresh = fresh(state.last_heartbeat);
        sample.telemetry_fresh = fresh(state.gps_updated_at) &&
                                 fresh(state.ekf_updated_at) && fresh(state.battery_updated_at);
        sample.rc_policy_ok = !require_rc || rc_seen;
        preflight.observe_preflight(sample);
        preflight.poll_preflight(now);
    }

    if (verbose) print_diagnostic_summary(false);
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
            std::cout << "[정보] 비행 로그: " << path_ << std::endl;
        } else {
            std::cerr << "[경고] 비행 로그를 열 수 없습니다: " << path_ << " 계속 실행합니다."
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

    void record_algorithm_state(const std::string& algorithm,
                                const std::string& previous_state,
                                const std::string& current_state,
                                double elapsed_sec, bool tracking,
                                double observation_age_ms) {
        (void)observation_age_ms;
        log(elapsed_sec, algorithm, false, tracking, 0.0, 0.0, 0.0, 0.0, 0.0, "", -1,
            -1.0, -1, -1, true, 0, false, "",
            "ALGORITHM_STATE:" + previous_state + "->" + current_state,
            0.0, 0.0, 0.0, TargetRangeMsg{}, 0.0);
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

enum class ApproachOutcome { kLanded, kHandedBack, kDisarmed };

// Stage 2+3 of the target-approach pipeline, plus this program's safety
// layers: an altitude ceiling (setting/safety.yaml's altitude_limit), a
// vehicle-health watch (battery/heartbeat/gps/prearm) that hands off to
// LOITER on a breach, and an unexpected-disarm watch that waits for the
// disarm heartbeat before exiting (see the HEARTBEAT case below). control.cpp is the only
// process that ever sends MAVLink commands to the vehicle, so all of this
// lives here instead of in a separate emergency process - see
// setting/safety.yaml's top comment.
ApproachOutcome approach_target(const YamlValue& track, const AltitudeLimit& alt_limit,
                                const HealthLimit& health_limit,
                                autopilot::AutopilotMavlinkAdapter& vehicle,
                                const TelemetryObserver& observe_telemetry, double duration_sec,
                                const std::string& resume_mode = "",
                                app::FlightPhaseController* phase_controller = nullptr,
                                bool allow_mavlink_writes = true,
                                const std::function<bool()>& control_locked = {},
                                const TargetRangeMsg* initial_observation = nullptr,
                                bool start_in_safe_hover = false) {
    // Kept in the call signature for launcher/config compatibility. Target
    // loss now always remains in GUIDED while the app waits for reacquisition.
    (void)resume_mode;
    int udp_port = static_cast<int>(track.get_long_or("udp_port", 15020));
    double cycle_ms = track.get_double_or("control_cycle_ms", 50);
    double max_horizontal_speed = track.get_double_or("max_forward_speed", 0.5);
    double max_lateral_speed = track.get_double_or("max_lateral_speed", max_horizontal_speed);
    const char* simulation_profile = std::getenv("SIMULATION_PROFILE");
    std::string guidance_mode = "centered_vertical_descent";
    try {
        guidance_mode = track["guidance_algorithm"].as_string();
    } catch (const std::exception&) {
        try {
            guidance_mode = track["guidance_mode"].as_string();
        } catch (const std::exception&) {
            // Older settings files retain the centered_descent baseline.
        }
    }
    if (guidance_mode == "centered_descent") {
        guidance_mode = "centered_vertical_descent";
    }
    if (guidance_mode != "centered_vertical_descent" && guidance_mode != "diagonal_approach") {
        throw std::runtime_error(
            "guidance_algorithm은 centered_vertical_descent 또는 diagonal_approach여야 합니다.");
    }
    double center_tolerance_px = track.get_double_or("center_tolerance_px", 25.0);
    double center_hold_sec = track.get_double_or("center_hold_sec", 1.0);
    const double hold_altitude_m = track.get_double_or("hold_altitude_m", 1.0);
    double vertical_descent_speed = track.get_double_or(
        "vertical_descent_speed_mps",
        track.get_double_or("vertical_descent_speed",
                            track.get_double_or("approach_descent_speed", 0.2)));
    double max_descent_speed = track.get_double_or(
        "guidance_max_descent_speed_mps", vertical_descent_speed);
    double diagonal_stop_distance = track.get_double_or(
        "diagonal_approach_stop_distance_m", 1.0);
    mission::CenteredVerticalDescentConfig guidance_config;
    guidance_config.search_altitude_m = track.get_double_or("search_altitude_m", 5.0);
    guidance_config.search_altitude_tolerance_m =
        track.get_double_or("search_altitude_tolerance_m", 0.3);
    guidance_config.center_hold_sec = center_hold_sec;
    guidance_config.hold_altitude_m = hold_altitude_m;
    guidance_config.center_tolerance_px = center_tolerance_px;
    guidance_config.center_dwell_tolerance_px =
        track.get_double_or("center_dwell_tolerance_px", 15.0);
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

    mission::DiagonalApproachAlgorithmConfig diagonal_config;
    diagonal_config.calibration = guidance_config.calibration;
    diagonal_config.gain = guidance_config.gain;
    diagonal_config.distance_gain = track.get_double_or(
        "diagonal_approach_distance_gain", 0.15);
    diagonal_config.max_forward_speed = max_horizontal_speed;
    diagonal_config.max_lateral_speed = max_lateral_speed;
    diagonal_config.max_vector_speed = guidance_config.max_vector_speed;
    diagonal_config.max_descent_speed_mps = guidance_config.max_descent_speed_mps;
    diagonal_config.desired_path_angle_rad = guidance_config.desired_path_angle_rad;
    const double diagonal_path_angle_deg =
        track.get_double_or("diagonal_path_angle_deg", 45.0);
    if (std::isfinite(diagonal_path_angle_deg) && diagonal_path_angle_deg > 0.0) {
        diagonal_config.desired_path_angle_rad =
            diagonal_path_angle_deg * 0.017453292519943295;
    } else {
        diagonal_config.desired_path_angle_rad = 0.7853981633974483;
    }
    diagonal_config.center_tolerance_px = center_tolerance_px;
    diagonal_config.stop_distance_m = diagonal_stop_distance;
    diagonal_config.hold_altitude_m = hold_altitude_m;
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
            std::cout << "[검증 설정] " << name << "=" << target << std::endl;
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
    if (guidance_mode != "diagonal_approach") {
        diagonal_config.desired_path_angle_rad = guidance_config.desired_path_angle_rad;
    }
    diagonal_config.filter_alpha = guidance_config.filter_alpha;
    diagonal_config.max_acceleration_mps2 = guidance_config.max_acceleration_mps2;
    diagonal_config.max_jerk_mps3 = guidance_config.max_jerk_mps3;
    diagonal_config.velocity_deadband_rad = guidance_config.velocity_deadband_rad;
    diagonal_config.velocity_hysteresis_rad = guidance_config.velocity_hysteresis_rad;
    diagonal_config.theta_safe_threshold_rad = diagonal_config.safe_fov_angle_rad;
    double link_stale_ms = track.get_double_or("link_stale_ms", 500);
    // Target loss is handled immediately by the GUIDED re-acquisition hold;
    // health failures retain their independent LAND/SAFE_HOVER policies.

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
    mission::CenteredVerticalDescentAlgorithm guidance(guidance_config);
    mission::DiagonalApproachAlgorithm diagonal_guidance(diagonal_config);
    const bool use_diagonal_approach = guidance_mode == "diagonal_approach";
    const auto algorithm_holding = [&]() {
        return use_diagonal_approach ? diagonal_guidance.holding() : guidance.holding();
    };
    std::cout << "[정보] 알고리즘=" << guidance_mode << std::endl;
    if (use_diagonal_approach) {
        std::cout << "[정보] 접근 경로각=" << std::fixed << std::setprecision(1)
                  << diagonal_config.desired_path_angle_rad * 180.0 / M_PI << "도" << std::endl;
    }

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
    const char* configured_detail_path = std::getenv("AUTONOMY_DETAIL_LOG");
    const std::string detail_path = configured_detail_path != nullptr && *configured_detail_path != '\0'
                                        ? configured_detail_path
                                        : resolve_log_dir() + "/autonomy.log";
    std::ofstream detail_log(detail_path, std::ios::app);
    std::cout << "[정보] 표적 접근 모드 시작 (UDP " << udp_port << ", 주기 " << cycle_ms << "ms, 최대 "
              << duration_sec << "초, 고도제한 " << alt_limit.soft_limit_m << "~"
              << alt_limit.hard_limit_m << "m)" << std::endl;

    // Zero velocity on the way out no matter how this function exits -
    // duration elapsed, target lost, or an exception - so the vehicle never
    // keeps coasting on the last command it was given.
    struct FinallyGuard {
        autopilot::AutopilotMavlinkAdapter& vehicle;

        ~FinallyGuard() {
            try {
                vehicle.send_zero_velocity();
            } catch (...) {
            }
        }
    } finally_guard{vehicle};

    mission::GuidanceDiagnostics guidance_diagnostics;
    auto guidance_force_zero = [&](const std::string& reason, double observation_age_ms) {
        return use_diagonal_approach
                   ? diagonal_guidance.force_zero(reason, observation_age_ms)
                   : guidance.force_zero(reason, observation_age_ms);
    };
    auto guidance_update = [&](double x_px, double y_px, double altitude_m,
                               double distance_m, double dt_sec, double observation_age_ms,
                               bool allow_descent_without_horizontal,
                               bool allow_descent) {
        return use_diagonal_approach
                   ? diagonal_guidance.update(x_px, y_px, altitude_m, distance_m, true,
                                              dt_sec, observation_age_ms)
                   : guidance.update(x_px, y_px, altitude_m, true, dt_sec,
                                     observation_age_ms,
                                     allow_descent_without_horizontal, allow_descent);
    };
    auto send_immediate_zero = [&](const std::string& reason, double observation_age_ms = -1.0) {
        guidance_diagnostics = guidance_force_zero(reason, observation_age_ms);
        try {
            vehicle.send_zero_velocity();
        } catch (const std::exception& e) {
            std::cerr << "[실패] 즉시 속도 0 전송 실패: " << e.what() << std::endl;
        }
    };

    // Target-loss hover must not call the algorithms' force_zero() methods:
    // those methods intentionally reset internal filters/state for a safety
    // stop. Reacquisition needs the pre-loss algorithm state unchanged.
    auto send_target_loss_zero = [&](const std::string& reason,
                                     double observation_age_ms = -1.0) {
        guidance_diagnostics.vx = 0.0;
        guidance_diagnostics.vy = 0.0;
        guidance_diagnostics.vz = 0.0;
        guidance_diagnostics.raw_vx = 0.0;
        guidance_diagnostics.raw_vy = 0.0;
        guidance_diagnostics.raw_vz = 0.0;
        guidance_diagnostics.filtered_vx = 0.0;
        guidance_diagnostics.filtered_vy = 0.0;
        guidance_diagnostics.filtered_vz = 0.0;
        guidance_diagnostics.horizontal_speed_mps = 0.0;
        guidance_diagnostics.commanded_path_angle_rad = 0.0;
        guidance_diagnostics.observation_age_ms = observation_age_ms;
        guidance_diagnostics.safety_override = reason;
        try {
            vehicle.send_zero_velocity();
        } catch (const std::exception& e) {
            std::cerr << "[실패] 표적 유실 후 속도 0 전송 실패: " << e.what() << std::endl;
        }
    };

    std::string last_algorithm_state;
    auto last_terminal_summary = std::chrono::steady_clock::time_point{};
    auto record_algorithm_state = [&](const std::string& state, double elapsed_sec,
                                      bool tracking, double observation_age_ms) {
        if (state.empty() || state == last_algorithm_state) return;
        const std::string previous = last_algorithm_state.empty() ? "NONE" : last_algorithm_state;
        std::cout << "[현재 상태] " << state << std::endl;
        logger.record_algorithm_state(guidance_mode, previous, state, elapsed_sec,
                                      tracking, observation_age_ms);
        last_algorithm_state = state;
    };
    auto print_terminal_summary = [&](std::chrono::steady_clock::time_point now,
                                      const std::string& state, const std::string& mode,
                                      bool armed, double altitude_m, bool tracking,
                                      double distance_m, double vx, double vy, double vz,
                                      bool target_loss_hover = false) {
        if (last_terminal_summary != std::chrono::steady_clock::time_point{} &&
            std::chrono::duration<double>(now - last_terminal_summary).count() < 1.0) {
            return;
        }
        last_terminal_summary = now;
        std::cout << std::fixed << std::setprecision(2);
        if (target_loss_hover) {
            std::cout << "[안전] TARGET_LOSS_HOVER 모드=" << mode << " ARM="
                      << (armed ? "예" : "아니오") << " 고도=" << altitude_m << "m";
        } else {
            std::cout << "[요약] 상태=" << state << " 모드=" << mode
                      << " ARM=" << (armed ? "예" : "아니오")
                      << " 고도=" << altitude_m << "m 표적="
                      << (tracking ? "확정" : "유실") << " 거리=" << distance_m
                      << "m vx=" << vx << " vy=" << vy << " vz=" << vz;
        }
        std::cout << std::endl;
    };
    record_algorithm_state(start_in_safe_hover ? "SAFE_HOVER" : "TARGET_SEARCH", 0.0,
                           false, -1.0);

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
    // Start the approach timeout when the selected algorithm enters its
    // active descent/diagonal state, not during handoff or centering.
    auto algorithm_approach_started_at = std::chrono::steady_clock::time_point{};
    bool algorithm_approach_timer_started = false;
    auto last_valid = start - std::chrono::seconds(10);
    TargetRangeMsg last{};
    app::TargetObservationGate observation_gate;
    if (initial_observation != nullptr &&
        observation_gate.update(*initial_observation, true)) {
        last = *initial_observation;
        last_valid = start;
    }

    HealthState health;
  health.last_heartbeat = start;  // the adapter already validated the initial heartbeat
    double previous_distance_m = 0.0;
    bool have_previous_distance = false;
    const bool require_rc_monitor = env_flag_enabled("REQUIRE_RC_PREFLIGHT");
    bool rc_seen = false;
    auto last_rc = start - std::chrono::seconds(10);

    bool alt_enforcing = false;
    bool in_loiter = start_in_safe_hover;
    // Kept as a local hold-latch name for minimal scope. The canonical target
    // loss behavior is GUIDED hover; it must not request LOITER or resume
    // guidance after a later rediscovery.
    bool target_loss_loiter_hold = false;
    bool target_loss_guided_hover = false;
    std::string target_loss_class;
    std::string last_tracked_class;
    std::string target_loss_algorithm_state;
    app::TargetReacquireGate target_reacquire_gate;
    auto last_reacquire_print = std::chrono::steady_clock::time_point{};
    const double target_control_min_confidence =
        track.get_double_or("target_control_min_confidence", 0.60);
    const double target_reacquire_dwell_sec =
        track.get_double_or("target_reacquire_dwell_sec", 2.0);
    bool safe_hover_latched = start_in_safe_hover;
    bool landing_requested = false;
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
    // Once this approach has reached a centered hold, its normal intercept
    // timeout must not hand control back to AUTO. A later target loss is
    // handled only by the explicit target-loss policy below.
    bool hold_ever_entered = false;
    bool timeout_hover = false;

    auto log_loiter_rc3_before_mode = [&]() {
        log_loiter_rc3(vehicle.state());
    };

    while (true) {
        if (observe_telemetry) observe_telemetry(nullptr);
        auto cycle_start = std::chrono::steady_clock::now();
        double elapsed_sec = std::chrono::duration<double>(cycle_start - start).count();
        const bool approach_timeout =
            algorithm_approach_timer_started && duration_sec > 0 &&
            std::chrono::duration<double>(cycle_start - algorithm_approach_started_at).count() >=
                duration_sec;

        TargetRangeMsg msg;
        if (receiver.poll(msg)) {
            last = msg;
            last_valid = cycle_start;
            observation_gate.update(msg, true);
        }
        // Opportunistically pick up whatever vehicle-health telemetry has
        // arrived since the last tick (see kHealthPollTimeoutSec above).
        mavlink_message_t hmsg;
        const bool was_armed = health.have_armed && health.armed;
        if (vehicle.recv_match({MAVLINK_MSG_ID_SYS_STATUS, MAVLINK_MSG_ID_GPS_RAW_INT,
                                MAVLINK_MSG_ID_HEARTBEAT, MAVLINK_MSG_ID_EKF_STATUS_REPORT,
                                MAVLINK_MSG_ID_RC_CHANNELS, MAVLINK_MSG_ID_STATUSTEXT},
                               hmsg, kHealthPollTimeoutSec)) {
            if (hmsg.msgid != MAVLINK_MSG_ID_HEARTBEAT ||
                is_valid_ardupilot_heartbeat(hmsg)) {
                if (observe_telemetry) observe_telemetry(&hmsg);
            }
        }

        // MAVLink decoding and freshness timestamps are owned by the adapter.
        // The app consumes one immutable state snapshot for policy and logging;
        // it does not decode vehicle telemetry itself.
        health = vehicle.state();
        enrich_target_observation(last, health, track, cycle_start,
                                  &previous_distance_m, &have_previous_distance);
        const double link_age_ms =
            std::chrono::duration<double, std::milli>(cycle_start - last_valid).count();
        const bool link_fresh = link_age_ms <= link_stale_ms;
        if (last.altitude_valid) altitude_ever_valid = true;

        const bool tracking = target_control_eligible(last, link_fresh,
                                                      target_control_min_confidence) &&
                              observation_gate.target_seen();
        if (phase_controller != nullptr) {
            if (observation_gate.target_seen() && !tracking &&
                phase_controller->current() == app::FlightPhase::GUIDED) {
                phase_controller->transition(app::FlightPhase::REACQUIRE,
                                         "target observation stale or lost");
            }
        }
        if (tracking) target_lost_since = cycle_start;
        if (tracking) {
            last_tracked_class.assign(last.class_name,
                                      strnlen(last.class_name, sizeof(last.class_name)));
        }
        const double target_lost_sec = observation_gate.target_seen()
                                           ? std::chrono::duration<double>(
                                                 cycle_start - target_lost_since)
                                                 .count()
                                           : 0.0;
        if (health.have_rc) {
            rc_seen = true;
            last_rc = health.rc_updated_at;
        }
        if (was_armed && health.have_armed && !health.armed) {
            std::cout << "[통과] DISARMED 상태 확인" << std::endl;
            record_algorithm_state("DISARMED", elapsed_sec, false, last.observation_age_ms);
            logger.log(elapsed_sec, mode_string(health.custom_mode), false, false, last.distance_m,
                       0, 0, 0, last.altitude_m, "", health.battery_percent,
                       health.battery_voltage_v, health.fix_type, health.satellites,
                       health.prearm_healthy, health.system_status, false,
                       "disarm observed", "DISARM_OBSERVED", health.lat, health.lon,
                       gps_moved_m, TargetRangeMsg{}, 0.0, guidance_diagnostics);
            if (phase_controller != nullptr &&
                phase_controller->current() != app::FlightPhase::FINISHED) {
                phase_controller->transition(app::FlightPhase::FINISHED,
                                         "disarm heartbeat observed");
            }
            return ApproachOutcome::kDisarmed;
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
        // the generic health breach below. Request LAND once, then remain in
        // the monitor loop until a disarm heartbeat is actually observed.
        // Returning here would make launcher cleanup race an armed vehicle.
        double heartbeat_gap = std::chrono::duration<double>(cycle_start - health.last_heartbeat).count();
        if (heartbeat_gap > health_limit.land_gap_sec && !landing_requested) {
            std::cout << "[비상] HEARTBEAT 지연=" << std::fixed << std::setprecision(1) << heartbeat_gap
                      << "s > land_gap_sec " << health_limit.land_gap_sec << "s - 긴급 LAND 요청 후 disarm 대기"
                      << std::endl;
            send_immediate_zero("heartbeat_timeout");
            logger.log(elapsed_sec, mode_string(health.custom_mode), health.armed, false, last.distance_m, 0,
                       0, 0, last.altitude_m, "", health.battery_percent, health.battery_voltage_v,
                       health.fix_type, health.satellites, health.prearm_healthy, health.system_status, true,
                       "heartbeat gap exceeded land_gap_sec", "HEARTBEAT_LAND_REQUESTED", health.lat,
                       health.lon, gps_moved_m, TargetRangeMsg{}, 0.0, guidance_diagnostics);
            try {
                vehicle.land();
                landing_requested = true;
                record_algorithm_state("LANDING", elapsed_sec, false, last.observation_age_ms);
                if (phase_controller != nullptr &&
                    phase_controller->current() != app::FlightPhase::LANDING) {
                    phase_controller->transition(app::FlightPhase::LANDING,
                                             "heartbeat timeout LAND requested");
                }
            } catch (const std::exception& e) {
                std::cerr << "[실패] LAND 명령 전송 실패: " << e.what() << std::endl;
            }
            in_loiter = false;
            safe_hover_latched = true;
            continue;
        }

        // After a continuous target loss, stop the autonomous mission at the
        // current position. The legacy LOITER policy name maps to a latched
        // GUIDED zero-velocity hold so the vehicle keeps its current altitude
        // and position without a mode change.
        const bool target_loss_guided_hover_due =
            observation_gate.target_seen() && !tracking;
        if (!target_loss_loiter_hold && !landing_requested && target_loss_guided_hover_due) {
            std::cout << "[경고] 표적 관측 유실 " << std::fixed
                      << std::setprecision(1) << target_lost_sec
                      << "s - GUIDED 호버링 및 재탐지 대기" << std::endl;
            if (phase_controller != nullptr &&
                phase_controller->current() != app::FlightPhase::REACQUIRE) {
                phase_controller->transition(app::FlightPhase::REACQUIRE,
                                             "표적 관측 유실, GUIDED 재탐지 대기");
            }
            target_loss_class = last_tracked_class;
            target_loss_algorithm_state = guidance_diagnostics.algorithm_state;
            if (target_loss_algorithm_state.empty()) {
                target_loss_algorithm_state = use_diagonal_approach
                                                   ? diagonal_approach_state_name(
                                                         diagonal_guidance.state())
                                                   : centered_vertical_descent_state_name(
                                                         guidance.state());
            }
            send_target_loss_zero("target_loss", last.observation_age_ms);
            record_algorithm_state("TARGET_LOSS", elapsed_sec, false,
                                   last.observation_age_ms);
            logger.log(elapsed_sec, mode_string(health.custom_mode), health.armed, false,
                       last.distance_m, 0, 0, 0, last.altitude_m, "",
                       health.battery_percent, health.battery_voltage_v, health.fix_type,
                       health.satellites, health.prearm_healthy, health.system_status, false,
                       "", "TARGET_LOST_GUIDED_HOVER", health.lat, health.lon, gps_moved_m,
                       TargetRangeMsg{}, 0.0, guidance_diagnostics);
            target_loss_loiter_hold = true;
            target_loss_guided_hover = true;
            target_reacquire_gate.reset();
            in_loiter = true;
            loiter_since = cycle_start;
            continue;
        }

        std::vector<std::string> reasons = evaluate_health_breach(health, health_limit, cycle_start);
        const auto stale = [&](std::chrono::steady_clock::time_point updated_at) {
            return updated_at == std::chrono::steady_clock::time_point{} ||
                   std::chrono::duration<double>(cycle_start - updated_at).count() >
                       health_limit.max_heartbeat_gap_sec;
        };
        if (health.have_gps && stale(health.gps_updated_at)) {
            reasons.emplace_back("GPS telemetry stale");
        }
        if (health.have_ekf && stale(health.ekf_updated_at)) {
            reasons.emplace_back("EKF telemetry stale");
        }
        if (health.have_battery && stale(health.battery_updated_at)) {
            reasons.emplace_back("battery telemetry stale");
        }
        if (require_rc_monitor &&
            (!rc_seen || std::chrono::duration<double>(cycle_start - last_rc).count() >
                             health_limit.max_heartbeat_gap_sec)) {
            reasons.emplace_back("RC telemetry stale or unavailable");
        }
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
        if (phase_controller != nullptr && emergency_active &&
            phase_controller->current() != app::FlightPhase::SAFE_HOVER) {
            phase_controller->transition(app::FlightPhase::SAFE_HOVER,
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
            record_algorithm_state("SAFE_HOVER", elapsed_sec, false,
                                   last.observation_age_ms);
            safe_hover_latched = true;
            std::string event;
            if (!in_loiter) {
                std::cout << "[비상] " << emergency_reason << ": LOITER로 전환" << std::endl;
                log_loiter_rc3_before_mode();
                vehicle.set_mode("LOITER");
                in_loiter = true;
                event = "LOITER_ENTER";
            }
            loiter_since = cycle_start;  // keep refreshing while active
            logger.log(elapsed_sec, mode_name, health.armed, false, last.distance_m, 0, 0, 0,
                       last.altitude_m, "", health.battery_percent, health.battery_voltage_v,
                       health.fix_type, health.satellites, health.prearm_healthy, health.system_status,
                       true, emergency_reason, event, health.lat, health.lon, gps_moved_m,
                       TargetRangeMsg{}, 0.0, guidance_diagnostics);
            print_terminal_summary(cycle_start, "SAFE_HOVER", "LOITER", health.armed,
                                   last.altitude_m, false, last.distance_m, 0.0, 0.0, 0.0);
            std::this_thread::sleep_until(next_tick);
            continue;
        }

        if (control_locked && control_locked()) {
            safe_hover_latched = true;
            if (phase_controller != nullptr &&
                phase_controller->current() != app::FlightPhase::SAFE_HOVER) {
                phase_controller->transition(app::FlightPhase::SAFE_HOVER,
                                         "ControlLocked");
            }
            send_immediate_zero("control_locked", last.observation_age_ms);
            record_algorithm_state("SAFE_HOVER", elapsed_sec, false,
                                   last.observation_age_ms);
            logger.log(elapsed_sec, mode_name, health.armed, false, last.distance_m, 0, 0, 0,
                       last.altitude_m, "", health.battery_percent, health.battery_voltage_v,
                       health.fix_type, health.satellites, health.prearm_healthy,
                       health.system_status, true, "ControlLocked", "CONTROL_LOCKED",
                       health.lat, health.lon, gps_moved_m, TargetRangeMsg{}, 0.0,
                       guidance_diagnostics);
            print_terminal_summary(cycle_start, "SAFE_HOVER", mode_name, health.armed,
                                   last.altitude_m, false, last.distance_m, 0.0, 0.0, 0.0);
            std::this_thread::sleep_until(next_tick);
            continue;
        }

        if (in_loiter) {
            double held_sec = std::chrono::duration<double>(cycle_start - loiter_since).count();
            if (target_loss_loiter_hold || safe_hover_latched || landing_requested) {
                if (target_loss_guided_hover && !safe_hover_latched && !landing_requested) {
                    std::vector<std::string> reacquire_health_reasons =
                        evaluate_health_breach(health, health_limit, cycle_start);
                    if (health.have_gps && stale(health.gps_updated_at)) {
                        reacquire_health_reasons.emplace_back("GPS telemetry stale");
                    }
                    if (health.have_ekf && stale(health.ekf_updated_at)) {
                        reacquire_health_reasons.emplace_back("EKF telemetry stale");
                    }
                    if (health.have_battery && stale(health.battery_updated_at)) {
                        reacquire_health_reasons.emplace_back("battery telemetry stale");
                    }
                    if (require_rc_monitor &&
                        (!rc_seen || std::chrono::duration<double>(cycle_start - last_rc).count() >
                                         health_limit.max_heartbeat_gap_sec)) {
                        reacquire_health_reasons.emplace_back("RC telemetry stale or unavailable");
                    }
                    const bool health_ok = reacquire_health_reasons.empty();
                    const bool locked = control_locked && control_locked();
                    const bool reacquired = target_reacquire_gate.update(
                        last, tracking, health_ok, locked, cycle_start, target_loss_class,
                        target_control_min_confidence, target_reacquire_dwell_sec);
                    if (reacquired) {
                        const std::string resumed_state = target_loss_algorithm_state;
                        std::cout << "[통과] TARGET_REACQUIRED 클래스=" << target_loss_class
                                  << " 신뢰도=" << std::fixed << std::setprecision(2)
                                  << last.target_confidence << std::endl;
                        logger.log(elapsed_sec, mode_name, health.armed, true,
                                   last.distance_m, 0, 0, 0, last.altitude_m, "",
                                   health.battery_percent, health.battery_voltage_v,
                                   health.fix_type, health.satellites, health.prearm_healthy,
                                   health.system_status, false, "", "TARGET_REACQUIRED",
                                   health.lat, health.lon, gps_moved_m, last, 0.0,
                                   guidance_diagnostics);
                        target_loss_loiter_hold = false;
                        target_loss_guided_hover = false;
                        target_reacquire_gate.reset();
                        in_loiter = false;
                        target_lost_since = cycle_start;
                        if (phase_controller != nullptr &&
                            phase_controller->current() == app::FlightPhase::REACQUIRE) {
                            phase_controller->transition(app::FlightPhase::GUIDED,
                                                         "동일 표적 2초 재탐지 확인");
                        }
                        if (resumed_state == "VERTICAL_DESCENT") {
                            std::cout << "[통과] RESUME_VERTICAL_DESCENT" << std::endl;
                            logger.log(elapsed_sec, mode_name, health.armed, true,
                                       last.distance_m, 0, 0, 0, last.altitude_m, "",
                                       health.battery_percent, health.battery_voltage_v,
                                       health.fix_type, health.satellites, health.prearm_healthy,
                                       health.system_status, false, "", "RESUME_VERTICAL_DESCENT",
                                       health.lat, health.lon, gps_moved_m, last, 0.0,
                                       guidance_diagnostics);
                        } else if (resumed_state == "APPROACH_45_DEG") {
                            std::cout << "[통과] RESUME_APPROACH_45_DEG" << std::endl;
                            logger.log(elapsed_sec, mode_name, health.armed, true,
                                       last.distance_m, 0, 0, 0, last.altitude_m, "",
                                       health.battery_percent, health.battery_voltage_v,
                                       health.fix_type, health.satellites, health.prearm_healthy,
                                       health.system_status, false, "", "RESUME_APPROACH_45_DEG",
                                       health.lat, health.lon, gps_moved_m, last, 0.0,
                                       guidance_diagnostics);
                        }
                        // Do not send a mode request here. The vehicle stayed
                        // in GUIDED throughout the re-acquisition dwell.
                        continue;
                    }

                    const double dwell_sec = target_reacquire_gate.elapsed_sec(cycle_start);
                    if (last_reacquire_print == std::chrono::steady_clock::time_point{} ||
                        std::chrono::duration<double>(cycle_start - last_reacquire_print).count() >=
                            1.0) {
                        std::cout << "[대기] REACQUIRE_WAIT 클래스="
                                  << (target_loss_class.empty() ? "확인 불가" : target_loss_class)
                                  << " 신뢰도=" << std::fixed << std::setprecision(2)
                                  << last.target_confidence << " 유지=" << std::setprecision(1)
                                  << dwell_sec << "/" << target_reacquire_dwell_sec << "초"
                                  << std::endl;
                        last_reacquire_print = cycle_start;
                    }
                    send_target_loss_zero("reacquire_wait", last.observation_age_ms);
                    logger.log(elapsed_sec, mode_name, health.armed, false,
                               last.distance_m, 0, 0, 0, last.altitude_m, "",
                               health.battery_percent, health.battery_voltage_v,
                               health.fix_type, health.satellites, health.prearm_healthy,
                               health.system_status, false,
                               "same class, confidence, link, altitude and health dwell",
                               "REACQUIRE_WAIT", health.lat, health.lon, gps_moved_m,
                               last, 0.0, guidance_diagnostics);
                    gcs_command_state_sender.send(
                        0.0f, 0.0f, 0.0f, 0.0f, false, "REACQUIRE", "reacquire_wait");
                    print_terminal_summary(cycle_start, "REACQUIRE", "GUIDED", health.armed,
                                           last.altitude_m, false, last.distance_m,
                                           0.0, 0.0, 0.0, true);
                    std::this_thread::sleep_until(next_tick);
                    continue;
                }
                record_algorithm_state(target_loss_loiter_hold ? "TARGET_LOSS" :
                                                                    (landing_requested ? "LANDING"
                                                                                         : "SAFE_HOVER"),
                                       elapsed_sec, tracking, last.observation_age_ms);
                const char* hold_event = landing_requested
                                             ? "LANDING_WAIT_DISARM"
                                             : (safe_hover_latched ? "SAFE_HOVER_HOLD"
                                                                    : (target_loss_guided_hover
                                                                           ? "TARGET_LOSS_GUIDED_HOVER"
                                                                           : "TARGET_LOSS_HOVER"));
                send_immediate_zero(landing_requested ? "landing_wait_disarm"
                                                       : (safe_hover_latched ? "safe_hover_latched"
                                                                              : "target_loss_hover"),
                                    last.observation_age_ms);
                logger.log(elapsed_sec, mode_name, health.armed, false, last.distance_m, 0, 0, 0,
                           last.altitude_m, "", health.battery_percent, health.battery_voltage_v,
                           health.fix_type, health.satellites, health.prearm_healthy,
                           health.system_status, safe_hover_latched,
                           landing_requested ? "waiting for disarm after LAND"
                                             : (safe_hover_latched ? "SAFE_HOVER latched"
                                                                    : "target loss latched"),
                           hold_event, health.lat, health.lon, gps_moved_m,
                           TargetRangeMsg{}, 0.0, guidance_diagnostics);
                gcs_command_state_sender.send(
                    0.0f, 0.0f, 0.0f, 0.0f, allow_mavlink_writes && health.armed,
                    hold_event, landing_requested ? "landing_wait_disarm"
                                                  : (safe_hover_latched ? "safe_hover_latched"
                                                                         : "target_loss_hover"));
                print_terminal_summary(cycle_start,
                                       landing_requested ? "LANDING" :
                                       (safe_hover_latched ? "SAFE_HOVER" : "TARGET_LOSS_HOVER"),
                                       target_loss_guided_hover ? "GUIDED" : "LOITER",
                                       health.armed, last.altitude_m, false,
                                       last.distance_m, 0.0, 0.0, 0.0,
                                       target_loss_guided_hover && !landing_requested &&
                                       !safe_hover_latched);
                std::this_thread::sleep_until(next_tick);
                continue;
            }
            if (held_sec < kMinLoiterHoldSec) {
                send_immediate_zero("loiter_safety_hold", last.observation_age_ms);
                logger.log(elapsed_sec, mode_name, health.armed, false, last.distance_m, 0, 0, 0,
                           last.altitude_m, "", health.battery_percent, health.battery_voltage_v,
                           health.fix_type, health.satellites, health.prearm_healthy, health.system_status,
                           false, "", "LOITER_HOLD", health.lat, health.lon, gps_moved_m,
                           TargetRangeMsg{}, 0.0, guidance_diagnostics);
                print_terminal_summary(cycle_start, "SAFE_HOVER", "LOITER", health.armed,
                                       last.altitude_m, false, last.distance_m, 0.0, 0.0, 0.0);
                std::this_thread::sleep_until(next_tick);
                continue;
            }
            std::cout << "[정보] 위험 상태 해제, 최소 유지시간 경과 - GUIDED로 복귀합니다." << std::endl;
            vehicle.set_mode("GUIDED");
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
        // before the target has reached the 1m hold condition. The timer is
        // started by the actual VERTICAL_DESCENT/APPROACH_45_DEG state above.
        // Keep the vehicle in GUIDED with zero velocity; target-loss and
        // health safety paths above still take precedence.
        if (approach_timeout && tracking && !hold_ever_entered &&
            !algorithm_holding() && !timeout_hover) {
            timeout_hover = true;
            std::cout << "[정보] 접근 제한 시간(" << duration_sec
                      << "초) 도달 - Hold 전 안전 호버링 유지" << std::endl;
            logger.log(elapsed_sec, mode_name, health.armed, true, last.distance_m, 0, 0, 0,
                       last.altitude_m, "", health.battery_percent, health.battery_voltage_v,
                       health.fix_type, health.satellites, health.prearm_healthy,
                       health.system_status, false, "", "INTERCEPT_TIMEOUT_HOVER", health.lat,
                       health.lon, gps_moved_m, last, 0.0, guidance_diagnostics);
        }

        if (timeout_hover && !algorithm_holding()) {
            send_immediate_zero("intercept_timeout_hover", last.observation_age_ms);
            logger.log(elapsed_sec, mode_name, health.armed, tracking, last.distance_m, 0, 0, 0,
                       last.altitude_m, "", health.battery_percent, health.battery_voltage_v,
                       health.fix_type, health.satellites, health.prearm_healthy,
                       health.system_status, false, "", "INTERCEPT_TIMEOUT_HOVER_HOLD",
                       health.lat, health.lon, gps_moved_m, last, 0.0, guidance_diagnostics);
            print_terminal_summary(cycle_start, "SAFE_HOVER", mode_name, health.armed,
                                   last.altitude_m, tracking, last.distance_m, 0.0, 0.0, 0.0);
            std::this_thread::sleep_until(next_tick);
            continue;
        }

            if (tracking) {
            const double distance_m = static_cast<double>(last.distance_m);
            const double x_px = static_cast<double>(last.x_px);
            const double y_px = static_cast<double>(last.y_px);
            guidance_diagnostics = guidance_update(
                x_px, y_px, std::max(0.05, static_cast<double>(last.altitude_m)), distance_m,
                cycle_ms / 1000.0, last.observation_age_ms,
                !use_diagonal_approach, true);
            vx = guidance_diagnostics.vx;
            vy = guidance_diagnostics.vy;

            if (algorithm_holding()) {
                guidance_diagnostics = guidance_force_zero(
                    "target_centered_hold", last.observation_age_ms);
                vx = 0.0;
                vy = 0.0;
                vz = 0.0;
                yaw_rate = 0.0;
                guidance_event = "TARGET_CENTERED_HOLD";
                const bool first_hold_entry = !hold_ever_entered;
                hold_ever_entered = true;
                record_algorithm_state("HOLD_1M", elapsed_sec, true,
                                       last.observation_age_ms);
                if (first_hold_entry) {
                    std::cout << "[통과] TargetCenteredHold 진입 - GUIDED 속도 0 호버링 유지"
                              << std::endl;
                }
            } else {
                vz = guidance_diagnostics.vz;
                yaw_rate = 0.0;
                guidance_event = guidance_diagnostics.algorithm_state;
                record_algorithm_state(guidance_diagnostics.algorithm_state, elapsed_sec, true,
                                       last.observation_age_ms);
                const bool approach_state_started =
                    guidance_diagnostics.algorithm_state == "VERTICAL_DESCENT" ||
                    guidance_diagnostics.algorithm_state == "APPROACH_45_DEG";
                if (approach_state_started && !algorithm_approach_timer_started) {
                    algorithm_approach_timer_started = true;
                    algorithm_approach_started_at = cycle_start;
                    std::cout << "[정보] 알고리즘 접근 타이머 시작 상태="
                              << guidance_diagnostics.algorithm_state
                              << " 제한=" << duration_sec << "초" << std::endl;
                }
            }
        } else {
            if (algorithm_holding()) {
                std::cout << "[경고] TargetCenteredHold 이탈 - 표적 유실 안전 정책 적용"
                          << std::endl;
                guidance_event = "TARGET_CENTER_HOLD_EXIT_TARGET_LOSS";
            }
            guidance_diagnostics = guidance_force_zero(
                "target_loss_or_stale", last.observation_age_ms);
            if (observation_gate.target_seen()) {
                guidance_event = "TARGET_LOSS";
                record_algorithm_state("TARGET_LOSS", elapsed_sec, false,
                                       last.observation_age_ms);
            }
            vx = 0.0;
            vy = 0.0;
            vz = 0.0;
        }

        std::string alt_status;
        // A lost/stale target and a centered hold are explicit zero-velocity
        // states. Do not let the independent altitude limiter overwrite them.
        const bool centered_search_hold =
            !use_diagonal_approach &&
            (guidance_diagnostics.algorithm_state == "CENTERING" ||
             guidance_diagnostics.algorithm_state == "CENTER_DWELL");
        if (centered_search_hold && last.altitude_m <= guidance_config.search_altitude_m) {
            // A previous descent-limit hysteresis must not reintroduce descent
            // while the centered algorithm is holding its 5m search altitude.
            alt_enforcing = false;
            alt_status = "SEARCH_ALTITUDE_HOLD";
        } else if (link_fresh && tracking && !algorithm_holding()) {
            double alt = last.altitude_m;
            if (alt > alt_limit.hard_limit_m) {
                if (!alt_enforcing) {
                std::cout << "[고도 제한] 고도=" << std::fixed << std::setprecision(2) << alt
                              << "m 제한=" << alt_limit.hard_limit_m << "m: 하강 보정 시작"
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

        vehicle.send_velocity_body(vx, vy, vz, yaw_rate);

        const std::string& center_state = guidance_diagnostics.algorithm_state;
        std::ostringstream detailed_summary;
        detailed_summary << std::fixed << std::setprecision(2)
                         << "[상태 요약] 경과=" << elapsed_sec
                         << "초 비행 단계=" << mode_name
                         << " ARM=" << (health.armed ? "예" : "아니오")
                         << " 표적=" << (tracking ? "확정" : "유실")
                         << " 거리=" << last.distance_m << "m"
                         << " 고도=" << last.altitude_m << "m"
                         << " vx=" << vx << " vy=" << vy << " vz=" << vz
                         << " 알고리즘 상태=" << center_state;
        if (!alt_status.empty()) detailed_summary << " [고도 제한 적용]";
        if (health.have_position) detailed_summary << " 실제 위치 이동=" << gps_moved_m << "m";
        detailed_summary << '\n';
        if (detail_log) {
            detail_log << detailed_summary.str();
            detail_log.flush();
        }
        print_terminal_summary(cycle_start, center_state, mode_name, health.armed,
                               last.altitude_m, tracking, last.distance_m, vx, vy, vz);

        logger.log(elapsed_sec, mode_name, health.armed, tracking, last.distance_m, vx, yaw_rate, vz,
                   last.altitude_m, alt_status, health.battery_percent, health.battery_voltage_v,
                   health.fix_type, health.satellites, health.prearm_healthy, health.system_status, false,
                   "", guidance_event, health.lat, health.lon, gps_moved_m, last, vy,
                   guidance_diagnostics);

        std::this_thread::sleep_until(next_tick);
    }

    return ApproachOutcome::kDisarmed;
}

void wait_for_auto_armed(autopilot::AutopilotMavlinkAdapter& vehicle,
                         double max_heartbeat_gap_sec,
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
            std::cout << "[대기] AUTO 모드 및 ARM 상태 대기" << std::endl;
            last_status_print = now;
            }
        }
    }
enum class LockWaitResult { kReady, kHeartbeatFailure, kSearchTimeout };

LockWaitResult wait_for_lock(const YamlValue& track,
                             autopilot::AutopilotMavlinkAdapter& vehicle,
                             TargetRangeReceiver& receiver,
                             const HealthLimit& health_limit, double link_stale_ms,
                             double handoff_dwell_sec, double handoff_min_altitude_m,
                             double target_control_min_confidence,
                             double search_timeout_sec, HealthState& health,
                             const TelemetryObserver& observe_telemetry,
                             TargetRangeMsg* ready_target) {
    const auto search_started = std::chrono::steady_clock::now();
    auto last_valid = std::chrono::steady_clock::now() - std::chrono::seconds(10);
    auto last_status_print = search_started - std::chrono::seconds(10);
    TargetRangeMsg last{};
    double previous_distance_m = 0.0;
    bool have_previous_distance = false;
    app::TargetHandoffGate handoff_gate;

    while (true) {
        if (observe_telemetry) observe_telemetry(nullptr);
        auto now = std::chrono::steady_clock::now();
        TargetRangeMsg target;
        if (receiver.poll(target)) {
            last = target;
            last_valid = now;
        }

        mavlink_message_t msg;
        if (vehicle.recv_match({MAVLINK_MSG_ID_SYS_STATUS, MAVLINK_MSG_ID_GPS_RAW_INT,
                                MAVLINK_MSG_ID_HEARTBEAT, MAVLINK_MSG_ID_EKF_STATUS_REPORT,
                                MAVLINK_MSG_ID_STATUSTEXT},
                               msg, 0.1)) {
            if (msg.msgid != MAVLINK_MSG_ID_HEARTBEAT ||
                is_valid_ardupilot_heartbeat(msg)) {
                if (observe_telemetry) observe_telemetry(&msg);
            }
        }
        health = vehicle.state();
        enrich_target_observation(last, health, track, now,
                                  &previous_distance_m, &have_previous_distance);

        const bool link_fresh =
            std::chrono::duration<double, std::milli>(now - last_valid).count() <=
                link_stale_ms;

        double heartbeat_gap =
            std::chrono::duration<double>(now - health.last_heartbeat).count();
        if (heartbeat_gap > health_limit.max_heartbeat_gap_sec) {
            std::cerr << "[실패] HEARTBEAT가 끊겨 표적 탐지 대기를 중단합니다."
                      << std::endl;
            return LockWaitResult::kHeartbeatFailure;
        }
        if (health.have_armed &&
            (!health.armed || health.custom_mode != copter_mode_mapping().at("AUTO"))) {
            return LockWaitResult::kHeartbeatFailure;
        }

        bool auto_armed = health.have_armed && health.armed &&
                          health.custom_mode == copter_mode_mapping().at("AUTO");
        bool healthy = auto_armed &&
                       evaluate_health_breach(health, health_limit, now).empty();
        const bool handoff_ready = handoff_gate.update(
            last, link_fresh, now, handoff_min_altitude_m, handoff_dwell_sec,
            target_control_min_confidence);
        if (!healthy) {
            handoff_gate.reset();
        }
        if (handoff_ready && healthy) {
            if (ready_target != nullptr) *ready_target = last;
            return LockWaitResult::kReady;
        }

        if (std::chrono::duration<double>(now - last_status_print).count() >= 5.0) {
            std::cout << "[대기] 표적 탐지 및 동일 class 2초 유지 대기"
                      << " 고도=" << std::fixed << std::setprecision(2) << last.altitude_m
                      << "m 상태=" << (healthy ? "정상" : "확인 중") << std::endl;
            last_status_print = now;
        }
        if (std::chrono::duration<double>(now - search_started).count() >= search_timeout_sec) {
            std::cerr << "[실패] 표적 탐색 시간이 초과되었습니다. 안전 호버로 전환합니다."
                      << " 제한시간=" << std::fixed << std::setprecision(1)
                      << search_timeout_sec << "초" << std::endl;
            return LockWaitResult::kSearchTimeout;
        }
    }
}

void run_auto_intercept(const YamlValue& track, const AltitudeLimit& alt_limit,
                        const HealthLimit& health_limit,
                        autopilot::AutopilotMavlinkAdapter& vehicle,
                        safety::SafetyMonitor& safety_monitor,
                        const TelemetryObserver& observe_telemetry,
                        const std::string& target_loss_resume_mode,
                        app::FlightPhaseController* phase_controller = nullptr,
                        bool allow_mavlink_writes = true) {
    const double link_stale_ms = track.get_double_or("link_stale_ms", 500.0);
    const double handoff_dwell_sec =
        track.get_double_or("target_handoff_dwell_sec", 2.0);
    const double target_control_min_confidence =
        track.get_double_or("target_control_min_confidence", 0.60);
    const double handoff_min_altitude_m =
        track.get_double_or("handoff_min_altitude_m", 4.7);
    const double intercept_duration_sec =
        track.get_double_or("intercept_approach_duration_sec", 60.0);
    const double target_search_timeout_sec =
        track.get_double_or("target_search_timeout_sec", 30.0);
    const double reacquire_cooldown_sec =
        track.get_double_or("reacquire_cooldown_sec", 3.0);
    const int udp_port = static_cast<int>(track.get_long_or("udp_port", 15020));

    if (allow_mavlink_writes) {
        request_message_interval(vehicle, MAVLINK_MSG_ID_SYS_STATUS, health_limit.poll_rate_hz);
        request_message_interval(vehicle, MAVLINK_MSG_ID_GPS_RAW_INT, health_limit.poll_rate_hz);
        request_message_interval(vehicle, MAVLINK_MSG_ID_EKF_STATUS_REPORT, health_limit.poll_rate_hz);
    }

    std::cout << "[정보] AUTO에서 GUIDED로 표적을 인계합니다. 동일 class 유지시간=" << handoff_dwell_sec
              << "초 최소고도=" << handoff_min_altitude_m
              << "m 접근 제한=" << intercept_duration_sec << "초" << std::endl;

    while (true) {
        wait_for_auto_armed(vehicle, health_limit.max_heartbeat_gap_sec, observe_telemetry);
        std::cout << "[통과] AUTO+ARM 상태 확인" << std::endl;
        std::cout << "[대기] 표적 탐지 및 동일 class 2초 유지 대기" << std::endl;

        if (phase_controller != nullptr &&
            (phase_controller->current() == app::FlightPhase::TAKEOFF ||
             phase_controller->current() == app::FlightPhase::PRECHECK)) {
            phase_controller->transition(app::FlightPhase::TARGET_SEARCH,
                                         "AUTO+ARM 확인, 첫 fresh target dwell 대기");
        }

        HealthState health;
        health.last_heartbeat = std::chrono::steady_clock::now();
        TargetRangeMsg initial_target{};
        LockWaitResult lock_result = LockWaitResult::kHeartbeatFailure;
        {
            TargetRangeReceiver receiver(udp_port);
            lock_result = wait_for_lock(track, vehicle, receiver, health_limit, link_stale_ms,
                                        handoff_dwell_sec, handoff_min_altitude_m,
                                        target_control_min_confidence,
                                        target_search_timeout_sec, health, observe_telemetry,
                                        &initial_target);
        }
        if (lock_result == LockWaitResult::kHeartbeatFailure) continue;
        if (lock_result == LockWaitResult::kSearchTimeout) {
            if (phase_controller != nullptr &&
                phase_controller->current() != app::FlightPhase::SAFE_HOVER) {
                phase_controller->transition(app::FlightPhase::SAFE_HOVER,
                                             "target search timeout before first handoff dwell");
            }
            if (allow_mavlink_writes) {
                try {
                    vehicle.send_zero_velocity();
                    log_loiter_rc3(vehicle.state());
                    vehicle.set_mode("LOITER");
                } catch (const std::exception& e) {
                    std::cerr << "[실패] 표적 탐색 시간 초과 후 안전 호버 전환 실패: "
                              << e.what() << std::endl;
                }
            }
            // Keep the control process alive and continue telemetry/safety
            // monitoring after a pre-handoff timeout. No target-loss timer or
            // algorithm update is started before the handoff dwell completes.
            approach_target(track, alt_limit, health_limit, vehicle, observe_telemetry, 0.0,
                            "LOITER", phase_controller, allow_mavlink_writes,
                            [&safety_monitor]() { return safety_monitor.control_locked(); },
                            nullptr, true);
            return;
        }

        const std::string target_class(initial_target.class_name);
        std::cout << "[통과] 표적 탐지 및 동일 class 유지" << std::endl;
        std::cout << "       클래스=" << (target_class.empty() ? "확인 불가" : target_class)
                  << " 신뢰도=" << std::fixed << std::setprecision(2)
                  << initial_target.target_confidence << std::endl;
        std::cout << "       유지시간=" << std::fixed << std::setprecision(2)
                  << handoff_dwell_sec << "초 거리="
                  << std::fixed << std::setprecision(2)
                  << initial_target.distance_m << "m 중심 오차=("
                  << initial_target.x_px << ", " << initial_target.y_px << ")px"
                  << " 관측 지연=" << initial_target.observation_age_ms << "ms" << std::endl;
        std::cout << "[정보] GUIDED 전환을 요청하고 실제 heartbeat를 확인합니다." << std::endl;
        if (!safety_monitor.prepare_mode_request("GUIDED")) {
            std::cerr << "[실패] GUIDED 예상 모드 등록에 실패했습니다." << std::endl;
            continue;
        }
        if (!vehicle.set_mode("GUIDED")) continue;
        try {
            wait_for_heartbeat(vehicle, 5.0, health_limit.max_heartbeat_gap_sec,
                               "GUIDED 전환",
                               [](const mavlink_heartbeat_t& heartbeat) {
                                   return heartbeat.custom_mode ==
                                          copter_mode_mapping().at("GUIDED");
                               },
                               observe_telemetry);
        } catch (const std::exception& e) {
            std::cerr << "[실패] GUIDED 확인 실패: " << e.what() << " AUTO 복귀를 시도합니다."
                      << std::endl;
            vehicle.set_mode("AUTO");
            continue;
        }

        // A SET_MODE write is only a request. Publish GUIDED after the
        // validated ArduPilot heartbeat confirms the actual mode.
        if (phase_controller != nullptr &&
            phase_controller->current() == app::FlightPhase::TARGET_SEARCH) {
            phase_controller->transition(app::FlightPhase::GUIDED,
                                     "GUIDED heartbeat 확인");
        }

        ApproachOutcome outcome = approach_target(track, alt_limit, health_limit, vehicle,
                                                  observe_telemetry, intercept_duration_sec,
                                                   target_loss_resume_mode, phase_controller,
                                                   allow_mavlink_writes,
                                                   [&safety_monitor]() {
                                                       return safety_monitor.control_locked();
                                                   }, &initial_target);
        if (outcome == ApproachOutcome::kLanded) return;
        if (outcome == ApproachOutcome::kDisarmed) {
            std::cout << "[통과] DISARMED 상태 확인" << std::endl;
            return;
        }

        wait_for_heartbeat(vehicle, 5.0, health_limit.max_heartbeat_gap_sec, "AUTO 복귀",
                           [](const mavlink_heartbeat_t& heartbeat) {
                               return heartbeat.custom_mode ==
                                      copter_mode_mapping().at("AUTO");
                           },
                           observe_telemetry);
        std::cout << "[정보] AUTO 복귀 확인, " << reacquire_cooldown_sec
                  << "초 후 표적을 다시 탐지합니다." << std::endl;
        std::this_thread::sleep_for(std::chrono::duration<double>(reacquire_cooldown_sec));
    }
}

}  // namespace

namespace app {

bool TargetHandoffGate::update(const TargetRangeMsg& observation, bool transport_fresh,
                               Clock::time_point now, double minimum_altitude_m,
                               double dwell_sec, double minimum_confidence) {
    const size_t class_capacity = sizeof(observation.class_name);
    const size_t class_length = strnlen(observation.class_name, class_capacity);
    const bool eligible = transport_fresh && observation.valid && observation.found &&
                          observation.altitude_valid &&
                          std::isfinite(observation.altitude_m) &&
                          observation.altitude_m >= minimum_altitude_m &&
                          std::isfinite(observation.observation_age_ms) &&
                          observation.observation_age_ms >= 0.0 &&
                          meets_control_confidence(observation.target_confidence,
                                                   minimum_confidence) &&
                          class_length > 0;
    if (!eligible) {
        reset();
        return false;
    }

    const std::string current_class(observation.class_name, class_length);
    if (!active_ || current_class != class_name_) {
        active_ = true;
        started_at_ = now;
        class_name_ = current_class;
    }
    return std::chrono::duration<double>(now - started_at_).count() >=
           std::max(0.0, dwell_sec);
}

void TargetHandoffGate::reset() {
    active_ = false;
    started_at_ = Clock::time_point{};
    class_name_.clear();
}

bool TargetReacquireGate::update(const TargetRangeMsg& observation,
                                 bool transport_fresh, bool health_ok,
                                 bool control_locked, Clock::time_point now,
                                 const std::string& expected_class,
                                 double minimum_confidence, double dwell_sec) {
    const size_t class_length = strnlen(observation.class_name,
                                        sizeof(observation.class_name));
    const std::string current_class(observation.class_name, class_length);
    const bool eligible = transport_fresh && health_ok && !control_locked &&
                          observation.valid && observation.found &&
                          observation.altitude_valid &&
                          std::isfinite(observation.altitude_m) &&
                          std::isfinite(observation.observation_age_ms) &&
                          observation.observation_age_ms >= 0.0 &&
                          meets_control_confidence(observation.target_confidence,
                                                   minimum_confidence) &&
                          class_length > 0 && current_class == expected_class;
    if (!eligible) {
        reset();
        return false;
    }
    if (!active_) {
        active_ = true;
        started_at_ = now;
    }
    return elapsed_sec(now) >= std::max(0.0, dwell_sec);
}

void TargetReacquireGate::reset() {
    active_ = false;
    started_at_ = Clock::time_point{};
}

double TargetReacquireGate::elapsed_sec(Clock::time_point now) const {
    if (!active_) return 0.0;
    return std::chrono::duration<double>(now - started_at_).count();
}

bool target_loss_keeps_guided_hover(const std::string& resume_mode) {
    return resume_mode == "LOITER" || resume_mode == "GUIDED";
}

bool wait_for_fly_approval(std::istream& input, std::ostream& output) {
    std::string line;
    output << "[대기] 미션을 시작하려면 FLY를 입력하고 Enter를 누르세요." << std::endl;
    const bool is_process_stdin = (&input == &std::cin);
    while (true) {
        if (std::getline(input, line)) {
            if (line == "FLY") return true;
            output << "[대기] FLY 입력이 필요합니다." << std::endl;
            continue;
        }

        // A launcher may put FlightMissionApp behind a background supervisor.
        // In that arrangement the inherited pipe/FIFO can report EOF even
        // though the operator terminal is still open. Do not interpret that
        // transient transport condition as an operator request to terminate.
        // Generic streams remain finite for unit tests; only process stdin
        // gets the terminal reconnect behavior.
        if (!is_process_stdin) {
            output << "[정보] 입력이 종료되어 미션을 시작하지 않고 안전하게 종료합니다." << std::endl;
            return false;
        }
        input.clear();

        std::ifstream terminal("/dev/tty");
        if (!terminal) {
            output << "[정보] 운용자 터미널을 찾을 수 없어 미션을 시작하지 않습니다." << std::endl;
            return false;
        }
        while (std::getline(terminal, line)) {
            if (line == "FLY") return true;
            output << "[대기] FLY 입력이 필요합니다." << std::endl;
        }
        // Reopen /dev/tty after an EOF/EIO-like read result. This keeps the
        // gate waiting without ever sending a vehicle command automatically.
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

FlightMissionApp::FlightMissionApp(FlightMissionAppOptions options)
    : options_(options) {}

int FlightMissionApp::run() {
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
            std::cerr << "[경고] safety.yaml 일부를 읽지 못했습니다 (" << e.what()
                      << "). 누락된 항목은 기본값을 사용합니다." << std::endl;
        }

        const RuntimeTarget runtime_target = runtime_target_from_environment();
        const RuntimeConfig transport =
            load_runtime_config(runtime_target, TransportRole::CommandOwner);
        const RuntimeConfig telemetry_transport =
            load_runtime_config(runtime_target, TransportRole::TelemetrySubscriber);
        bool real_flight_approved =
            runtime_target != RuntimeTarget::Real || transport.command_mode != CommandMode::Flight;
        if (runtime_target == RuntimeTarget::Real &&
            transport.command_mode == CommandMode::Flight) {
            RealFlightOptions options;
            if (const char* value = std::getenv("ASTRODRONE_SERIAL_ENDPOINT")) {
                options.serial_endpoint = value;
            }
            options.telemetry_endpoint = telemetry_transport.endpoint;
            options.allow_arm = env_flag_enabled("ASTRODRONE_ALLOW_ARM");
            options.confirm_real_flight = env_flag_enabled("ASTRODRONE_CONFIRM_REAL_FLIGHT");
            options.commands_enabled = env_flag_enabled("ASTRODRONE_COMMANDS_ENABLED");
            options.serial_owner_confirmed =
                env_flag_enabled("ASTRODRONE_SERIAL_OWNER_CONFIRMED");
            const auto decision = validate_real_flight(transport, options);
            if (!decision.allowed) {
                throw std::runtime_error("실기체 안전 검사 실패: " + decision.reason);
            }
            real_flight_approved = true;
        }
        std::cout << "[정보] 실행 대상=" << runtime_target_name(runtime_target)
                  << " 연결 대상=" << transport.endpoint << std::endl;
        safety::SafetyMonitor safety_monitor;
        safety::CommandAuthority command_authority;
        command_authority.automation_enabled = true;
        command_authority.vehicle_commands_enabled = transport.allow_vehicle_commands;
        command_authority.arm_commands_enabled = transport.allow_arm;
        command_authority.real_flight_approved = real_flight_approved;
        safety_monitor.set_authority(command_authority);
        safety_monitor.set_preflight_required(true);
        safety_monitor.set_preflight_ready(false);
        logging::CommandAuditLogger command_audit_logger;
        phase_controller_.set_event_sink(
            [&](const app::FlightPhaseEvent& event) {
                command_audit_logger.record_phase_event(event);
                std::cout << "[현재 상태] " << phase_display_name(event.current) << std::endl;
            });
        auto& phase_controller = phase_controller_;

        auto vehicle = std::make_unique<autopilot::AutopilotMavlinkAdapter>(
            open_transport(transport.endpoint, transport.baud),
            transport,
            [&](const safety::CommandRequest& request) {
                return safety_monitor.authorize(request);
            },
            [&](const safety::CommandDecision& decision) {
                command_audit_logger.record_decision(decision, safety_monitor.snapshot());
                if (!decision.allowed) {
                    std::cerr << "[명령 차단] "
                              << safety::command_type_name(decision.type) << " ("
                              << safety::gate_block_reason_name(decision.block_reason);
                    if (!decision.control_lock_reason.empty()) {
                        std::cerr << ", 사유=" << decision.control_lock_reason;
                    }
                    std::cerr << ")" << std::endl;
                }
            },
            [&](const safety::CommandRequest& request, bool write_succeeded) {
                safety_monitor.record_mode_request(request, write_succeeded);
            });
        mavlink_heartbeat_t initial_heartbeat{};
        if (!vehicle->wait_heartbeat(5.0, &initial_heartbeat)) {
            throw std::runtime_error("자동 제어 시작 전 fresh HEARTBEAT를 받지 못했습니다.");
        }
        safety_monitor.observe_heartbeat(initial_heartbeat);

        bool telemetry_gap_active = false;
        std::chrono::steady_clock::time_point telemetry_gap_started{};
        std::string last_received_message_type = "NONE";
        const TelemetryObserver observe_telemetry = [&](const mavlink_message_t* message) {
            if (message != nullptr) {
                last_received_message_type = preflight_message_name(message->msgid);
            }
            const bool was_control_locked = safety_monitor.control_locked();
            if (message && message->msgid == MAVLINK_MSG_ID_HEARTBEAT) {
                safety_monitor.observe_autopilot_state(vehicle->state());
            } else if (message && message->msgid == MAVLINK_MSG_ID_STATUSTEXT) {
                safety_monitor.observe_failsafe_evidence(vehicle->state().last_status_text);
            }
            safety_monitor.poll();
            if (!was_control_locked && safety_monitor.control_locked()) {
                command_audit_logger.record_control_lock(
                    safety::control_lock_reason_name(safety_monitor.lock_reason()),
                    safety_monitor.snapshot());
            }
            const auto now = std::chrono::steady_clock::now();
            const auto state = vehicle->state();
            const auto age_sec = [&](std::chrono::steady_clock::time_point updated_at) {
                if (updated_at == std::chrono::steady_clock::time_point{}) return -1.0;
                return std::chrono::duration<double>(now - updated_at).count();
            };
            const double heartbeat_age = age_sec(state.last_heartbeat);
            const double gps_age = age_sec(state.gps_updated_at);
            const double ekf_age = age_sec(state.ekf_updated_at);
            const double battery_age = age_sec(state.battery_updated_at);
            const bool heartbeat_gap = heartbeat_age > health_limit.max_heartbeat_gap_sec;
            const bool gps_gap = state.have_gps && gps_age > health_limit.max_heartbeat_gap_sec;
            const bool ekf_gap = state.have_ekf && ekf_age > health_limit.max_heartbeat_gap_sec;
            const bool battery_gap = state.have_battery &&
                                     battery_age > health_limit.max_heartbeat_gap_sec;
            const bool freshness_gap = heartbeat_gap || gps_gap || ekf_gap || battery_gap;
            if (freshness_gap && !telemetry_gap_active) {
                telemetry_gap_active = true;
                telemetry_gap_started = now;
                std::cerr << "[경고] 텔레메트리 지연 시작"
                          << " 연결 대상=" << transport.endpoint
                          << " HEARTBEAT지연=" << heartbeat_age << "초"
                          << " GPS지연=" << gps_age << "초"
                          << " EKF지연=" << ekf_age << "초"
                          << " 배터리지연=" << battery_age << "초"
                          << " 마지막 메시지=" << last_received_message_type
                          << std::endl;
            } else if (!freshness_gap && telemetry_gap_active) {
                const double duration = std::chrono::duration<double>(now - telemetry_gap_started).count();
                telemetry_gap_active = false;
                std::cerr << "[정보] 텔레메트리 지연 종료"
                          << " 연결 대상=" << transport.endpoint
                          << " 지속시간=" << duration << "초"
                          << " 마지막 메시지=" << last_received_message_type
                          << std::endl;
            }
        };

        // Telemetry configuration is intentionally outside the vehicle command policy. It
        // is needed to collect the evidence required by the preflight gate;
        // no mode/ARM/takeoff/velocity/LAND command can pass yet.
        if (transport.allow_telemetry_configuration) {
            request_message_interval(*vehicle, MAVLINK_MSG_ID_SYS_STATUS, health_limit.poll_rate_hz);
            request_message_interval(*vehicle, MAVLINK_MSG_ID_GPS_RAW_INT, health_limit.poll_rate_hz);
            request_message_interval(*vehicle, MAVLINK_MSG_ID_EKF_STATUS_REPORT, health_limit.poll_rate_hz);
            request_message_interval(*vehicle, MAVLINK_MSG_ID_GLOBAL_POSITION_INT, health_limit.poll_rate_hz);
            request_message_interval(*vehicle, MAVLINK_MSG_ID_LOCAL_POSITION_NED, health_limit.poll_rate_hz);
            request_message_interval(*vehicle, MAVLINK_MSG_ID_ALTITUDE, health_limit.poll_rate_hz);
            request_message_interval(*vehicle, MAVLINK_MSG_ID_ATTITUDE, health_limit.poll_rate_hz);
            request_message_interval(*vehicle, MAVLINK_MSG_ID_RC_CHANNELS, health_limit.poll_rate_hz);
            // The SITL ARM gate requires a real RAW_IMU stability sample stream.
            // Request the evidence messages through the same Adapter connection;
            // do not weaken the gate when a vehicle does not emit them by default.
            request_message_interval(*vehicle, MAVLINK_MSG_ID_RAW_IMU, health_limit.poll_rate_hz);
            request_message_interval(*vehicle, MAVLINK_MSG_ID_HIGHRES_IMU, health_limit.poll_rate_hz);
            request_message_interval(*vehicle, MAVLINK_MSG_ID_VIBRATION, health_limit.poll_rate_hz);
        }

        safety::PreflightPolicy preflight_policy;
        preflight_policy.require_rc_policy = env_flag_enabled("REQUIRE_RC_PREFLIGHT");
        // Vision readiness is a mandatory preflight input for both SITL and real.
        // The launcher still controls how YOLO/FrameSource is started; it cannot
        // make an HTTP-only process pass without a fresh frame marker.
        preflight_policy.require_vision = true;
        preflight_policy.require_prearm_healthy = health_limit.require_prearm_healthy;
        safety_monitor.configure_preflight(preflight_policy);
        std::cout << "=== 비행 사전 안전검사 ===" << std::endl;
        wait_for_preflight(*vehicle, health_limit, safety_monitor, observe_telemetry);
        safety_monitor.set_preflight_ready(true);
        write_marker_file_from_env("AUTONOMY_PREFLIGHT_READY_FILE", "preflight_ready");

        const auto& preflight_state = vehicle->state();
        const std::string camera_marker_path =
            std::getenv("CAMERA_FRAME_READY_FILE") != nullptr
                ? std::getenv("CAMERA_FRAME_READY_FILE")
                : "";
        const std::string yolo_endpoint =
            std::getenv("YOLO_HTTP_ENDPOINT") != nullptr
                ? std::getenv("YOLO_HTTP_ENDPOINT")
                : "http://127.0.0.1:8002";
        const auto latest_camera = app::read_camera_frame_readiness(camera_marker_path);
        const bool latest_http_alive = app::http_endpoint_alive(yolo_endpoint);
        const std::string yolo_ready_marker_path =
            std::getenv("YOLO_READY_FILE") != nullptr
                ? std::getenv("YOLO_READY_FILE")
                : "";
        const std::string camera_source_marker_path =
            std::getenv("CAMERA_SOURCE_READY_FILE") != nullptr
                ? std::getenv("CAMERA_SOURCE_READY_FILE")
                : "";
        const bool yolo_marker_present = !yolo_ready_marker_path.empty() &&
                                          ::access(yolo_ready_marker_path.c_str(), F_OK) == 0;
        const bool camera_source_ready = !camera_source_marker_path.empty() &&
                                         ::access(camera_source_marker_path.c_str(), F_OK) == 0;
        const auto latest_vision = app::evaluate_vision_readiness(
            process_alive_from_environment("YOLO_PROCESS_PID"), latest_http_alive,
            yolo_marker_present,
            camera_source_ready,
            latest_camera);
        const bool require_rc = env_flag_enabled("REQUIRE_RC_PREFLIGHT");
        const bool rc_ok = !require_rc || preflight_state.have_rc;
        const bool vision_ok = latest_vision.ready;
        const auto print_check = [](const char* name, bool passed, const std::string& detail) {
            std::cout << (passed ? "[통과] " : "[실패] ") << name << std::endl;
            if (!detail.empty()) std::cout << "       " << detail << std::endl;
        };
        print_check("MAVLink 연결", true, "연결 대상=" + transport.endpoint);
        print_check("HEARTBEAT 수신", preflight_state.last_heartbeat !=
                    std::chrono::steady_clock::time_point{},
                    "시스템=" + std::to_string(preflight_state.heartbeat_system_id) +
                    " 컴포넌트=" + std::to_string(preflight_state.heartbeat_component_id));
        print_check("GPS 상태", preflight_state.have_gps,
                    "GPS fix=" + std::to_string(preflight_state.fix_type) +
                    " 위성=" + std::to_string(preflight_state.satellites));
        print_check("EKF 상태", preflight_state.have_ekf,
                    "EKF=" + std::string(preflight_state.have_ekf ? "정상" : "비정상") +
                    " 수평분산=" + std::to_string(preflight_state.ekf_pos_horiz_variance));
        print_check("배터리 상태", preflight_state.battery_valid,
                    "전압=" + std::to_string(preflight_state.battery_voltage_v) + "V 잔량=" +
                    std::to_string(preflight_state.battery_percent) + "%");
        print_check("Telemetry 신선도", true, "HEARTBEAT/GPS/EKF/배터리 정상");
        print_check("RC 정책", rc_ok, require_rc ?
                    (preflight_state.have_rc ? "RC 입력 정상" : "RC 입력 없음") :
                    "RC 확인 불필요");
        print_check("YOLO/카메라 준비", vision_ok,
                    "사유=" + (latest_vision.reason.empty() ? "정상" : latest_vision.reason) +
                    " HTTP=" + (latest_http_alive ? "정상" : "실패") +
                    " frame_sequence=" + std::to_string(latest_camera.frame_sequence) +
                    " frame_age=" + std::to_string(latest_camera.age_sec) + "초 크기=" +
                    std::to_string(latest_camera.frame_width) + "x" +
                    std::to_string(latest_camera.frame_height));
        const int preflight_passed = 8 - (!preflight_state.have_gps) - (!preflight_state.have_ekf) -
                                     (!preflight_state.battery_valid) - (!rc_ok) - (!vision_ok);
        std::cout << "안전검사 결과: " << preflight_passed << "/8 통과" << std::endl;

        if (transport.command_mode == CommandMode::Flight) {
            if (!wait_for_fly_approval(std::cin, std::cout)) return 0;
            std::cout << "[현재 상태] 이륙 대기" << std::endl;
            std::cout << "[정보] AUTO 모드 및 ARM 상태 대기" << std::endl;
        }

        if (transport.command_mode == CommandMode::Shadow) {
            // Shadow performs the real telemetry/target/guidance calculation,
            // while SafetyMonitor blocks every vehicle-affecting request. It
            // deliberately does not enter AUTO/GUIDED setup.
            std::cout << "[정보] shadow 모드: guidance 계산만 수행하며 기체 명령은 차단합니다."
                      << std::endl;
            approach_target(track, alt_limit, health_limit, *vehicle, observe_telemetry,
                            track.get_double_or("approach_duration_sec", 60.0), "", &phase_controller,
                            false, [&safety_monitor]() {
                                return safety_monitor.control_locked();
                            });
            return 0;
        }

        if (!options_.self_launch) {
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
            const bool mission_setup_requested =
                env_flag_enabled("AUTONOMY_MISSION_SETUP_REQUESTED");
            if (mission_setup_requested) {
                mission::MissionSetupConfig mission_config;
                if (const char* value = std::getenv("MISSION_WAYPOINT_LAT")) {
                    mission_config.waypoint_lat = std::stod(value);
                }
                if (const char* value = std::getenv("MISSION_WAYPOINT_LON")) {
                    mission_config.waypoint_lon = std::stod(value);
                }
                if (const char* value = std::getenv("MISSION_ALTITUDE_M")) {
                    mission_config.altitude_m = std::stod(value);
                }
                if (const char* value = std::getenv("MISSION_SETUP_TIMEOUT_SEC")) {
                    mission_config.timeout_sec = std::stod(value);
                }
                mission::MissionSetup mission_setup(*vehicle, mission_config);
                mission_setup.run();
                write_marker_file_from_env("AUTONOMY_MISSION_SETUP_COMPLETE_FILE",
                                           "mission_setup_complete");
            } else {
                // Legacy callers may still perform setup externally, but the
                // real flight launcher never uses this path.
                wait_for_marker_file_from_env("AUTONOMY_MISSION_SETUP_COMPLETE_FILE",
                                              "mission_setup_complete");
            }
            // Require two fresh AUTO+ARM heartbeats before rebasing authority.
            wait_for_auto_armed(*vehicle, health_limit.max_heartbeat_gap_sec,
                                observe_telemetry, 2);
            if (phase_controller.current() == app::FlightPhase::PRECHECK) {
                phase_controller.transition(app::FlightPhase::TAKEOFF,
                                        "실제 AUTO+ARM heartbeat 확인");
            }
            if (!safety_monitor.begin_automation_session(
                    copter_mode_mapping().at("AUTO"))) {
                throw std::runtime_error(
                    "AUTO+ARM 안정 heartbeat 이후 authority session 시작에 실패했습니다.");
            }
            command_audit_logger.record_session_started(safety_monitor.snapshot());
            run_auto_intercept(track, alt_limit, health_limit, *vehicle, safety_monitor,
                               observe_telemetry,
                               target_loss_resume_mode, &phase_controller,
                               transport.allow_mavlink_writes);
            return 0;
        }

        if (!vehicle->set_mode("GUIDED")) {
            throw std::runtime_error("GUIDED 모드 전환 명령 전송 실패 (지원하지 않는 모드)");
        }
        wait_for_heartbeat(*vehicle, 5.0, health_limit.max_heartbeat_gap_sec, "GUIDED 모드 전환",
                            [](const mavlink_heartbeat_t& hb) {
                                return hb.custom_mode == copter_mode_mapping().at("GUIDED");
                            },
                            observe_telemetry);
        std::cout << "[통과] 실제 GUIDED 모드 확인" << std::endl;

        vehicle->arm_disarm(true);
        wait_for_heartbeat(*vehicle, 5.0, health_limit.max_heartbeat_gap_sec, "시동 확인",
                           [](const mavlink_heartbeat_t& heartbeat) {
                               return is_armed_from_heartbeat(heartbeat);
                           },
                           observe_telemetry);
        safety_monitor.begin_automation_session();
        constexpr double kSelfLaunchAltitudeM = 4.5;
        vehicle->takeoff(kSelfLaunchAltitudeM);
        wait_for_heartbeat(*vehicle, 10.0, health_limit.max_heartbeat_gap_sec, "이륙 대기", nullptr,
                           observe_telemetry);

        ApproachOutcome outcome = approach_target(
            track, alt_limit, health_limit, *vehicle, observe_telemetry,
            track.get_double_or("approach_duration_sec", 60.0), "LOITER", &phase_controller,
            transport.allow_mavlink_writes, [&safety_monitor]() {
                return safety_monitor.control_locked();
            });

        if (outcome == ApproachOutcome::kDisarmed) return 0;
        if (outcome != ApproachOutcome::kLanded) {
            phase_controller.transition(app::FlightPhase::LANDING, "LAND command requested");
            vehicle->land();
            std::cout << "착륙 중..." << std::endl;
        }
        return 0;
}

}  // namespace app
