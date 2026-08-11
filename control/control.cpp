#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits.h>
#include <sstream>
#include <sys/stat.h>
#include <thread>
#include <unistd.h>
#include <vector>

#include "drone_lib.hpp"
#include "target_link.hpp"

namespace {

// safety.yaml stores booleans as the bare strings "true"/"false" --
// YamlValue has no bool accessor of its own.
bool get_bool_or(const YamlValue& parent, const std::string& key, bool default_value) {
    try {
        return parent[key].as_string() == "true";
    } catch (const std::exception&) {
        return default_value;
    }
}

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
                         const std::function<bool(const mavlink_heartbeat_t&)>& is_ready = nullptr) {
    auto start = std::chrono::steady_clock::now();
    auto last_heartbeat = start;
    while (true) {
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
        if (vehicle.recv_match({MAVLINK_MSG_ID_HEARTBEAT}, msg, 0.2)) {
            mavlink_heartbeat_t hb;
            mavlink_msg_heartbeat_decode(&msg, &hb);
            last_heartbeat = now;
            if (is_ready && is_ready(hb)) return;
        }
    }
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

// Resolves <repo_root>/Document/logs, the same executable-relative lookup
// yaml_settings.cpp uses for setting/. Falls back to next to the
// executable if Document/logs isn't found, rather than failing the flight
// over a logging path.
std::string resolve_log_dir() {
    char exe_path[PATH_MAX];
    ssize_t len = readlink("/proc/self/exe", exe_path, sizeof(exe_path) - 1);
    std::string exe_dir = ".";
    if (len != -1) {
        exe_path[len] = '\0';
        std::string full(exe_path);
        exe_dir = full.substr(0, full.find_last_of('/'));
    }
    std::string candidates[] = {exe_dir + "/../../Document/logs", exe_dir + "/../Document/logs"};
    for (const auto& candidate : candidates) {
        struct stat st;
        if (stat(candidate.c_str(), &st) == 0 && S_ISDIR(st.st_mode)) return candidate;
    }
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
// unexpected disarm). Written as CSV to Document/logs/flight_<timestamp>.csv
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
                     "system_status,emergency_active,emergency_reason,event,gps_lat,gps_lon,gps_moved_m\n";
            file_.flush();
            std::cout << "Flight log: " << path_ << std::endl;
        } else {
            std::cerr << "Warning: could not open flight log at " << path_ << " - continuing without it."
                      << std::endl;
        }
    }

    void log(double elapsed_s, const std::string& mode, bool armed, bool tracking, double dist_m, double vx,
              double yaw_rate, double vz, double alt_m, const std::string& alt_status, int battery_percent,
              double battery_voltage_v, int gps_fix_type, int gps_satellites, bool prearm_healthy,
              int system_status, bool emergency_active, const std::string& emergency_reason,
              const std::string& event, double gps_lat, double gps_lon, double gps_moved_m) {
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
              << gps_moved_m << '\n';
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

struct HealthLimit {
    long min_battery_percent = 20;
    double min_battery_voltage_v = 14.8;
    double max_heartbeat_gap_sec = 3;
    long min_fix_type = 3;
    long min_satellites = 6;
    bool require_prearm_healthy = true;
    bool require_normal_state = true;
    double poll_rate_hz = 5;
    double land_gap_sec = 5;  // heartbeat gap past this: LAND instead of LOITER
    double ekf_pos_horiz_variance_max = 1.0;
    double ekf_velocity_variance_max = 1.0;
};

// Vehicle-health state, updated as SYS_STATUS/GPS_RAW_INT/HEARTBEAT trickle
// in and checked every cycle against HealthLimit. Same fields and
// thresholds emergency.cpp's standalone diagnostic prints, but evaluated
// here so control.cpp - the only process that ever sends MAVLink commands
// to the vehicle - can act on a breach directly instead of relaying it.
struct HealthState {
    std::chrono::steady_clock::time_point last_heartbeat;
    bool have_battery = false;
    int battery_percent = -1;
    double battery_voltage_v = -1;
    bool have_gps = false;
    uint8_t fix_type = 0;
    uint8_t satellites = 255;  // 255 == unknown, per MAVLink convention
    bool have_position = false;  // fix_type >= 2: lat/lon below are meaningful
    double lat = 0;
    double lon = 0;
    bool have_sys_status = false;
    bool prearm_healthy = true;
    uint8_t system_status = MAV_STATE_STANDBY;
    uint32_t custom_mode = 0;
    bool have_armed = false;
    bool armed = false;
    bool have_ekf = false;
    double ekf_pos_horiz_variance = 0;
    double ekf_velocity_variance = 0;
};

// Returns the breach reasons currently active, or an empty vector if none.
std::vector<std::string> evaluate_health_breach(const HealthState& h, const HealthLimit& limit,
                                                 std::chrono::steady_clock::time_point now) {
    std::vector<std::string> reasons;

    double heartbeat_gap = std::chrono::duration<double>(now - h.last_heartbeat).count();
    if (heartbeat_gap > limit.max_heartbeat_gap_sec) {
        std::ostringstream oss;
        oss << "heartbeat gap " << std::fixed << std::setprecision(1) << heartbeat_gap << "s > "
            << limit.max_heartbeat_gap_sec << "s";
        reasons.push_back(oss.str());
    }
    // battery_remaining == -1 / voltage_battery unreported both mean "not
    // reported" - can't be assumed safe or unsafe, so never a breach.
    if (h.have_battery && h.battery_percent >= 0 && h.battery_percent < limit.min_battery_percent) {
        reasons.push_back("battery " + std::to_string(h.battery_percent) + "% < " +
                           std::to_string(limit.min_battery_percent) + "%");
    }
    if (h.have_battery && h.battery_voltage_v >= 0 && h.battery_voltage_v < limit.min_battery_voltage_v) {
        std::ostringstream oss;
        oss << "battery " << std::fixed << std::setprecision(2) << h.battery_voltage_v << "V < "
            << limit.min_battery_voltage_v << "V";
        reasons.push_back(oss.str());
    }
    // satellites_visible == 255 means "not reported".
    if (h.have_gps && h.satellites != 255 &&
        (h.fix_type < limit.min_fix_type || h.satellites < limit.min_satellites)) {
        reasons.push_back("gps fix_type=" + std::to_string(static_cast<int>(h.fix_type)) +
                           " satellites=" + std::to_string(static_cast<int>(h.satellites)));
    }
    if (h.have_sys_status && limit.require_prearm_healthy && !h.prearm_healthy) {
        reasons.push_back("prearm check unhealthy");
    }
    if (h.have_sys_status && limit.require_normal_state &&
        (h.system_status == MAV_STATE_CRITICAL || h.system_status == MAV_STATE_EMERGENCY)) {
        reasons.push_back("system_status=" + std::to_string(static_cast<int>(h.system_status)) +
                           " (CRITICAL/EMERGENCY - ArduPilot internal failsafe, e.g. EKF)");
    }
    // Same predictor astroquad/uav-onboard's SafetyMonitor uses: ArduCopter
    // only accepts GUIDED when these ratios are below ~1.0, so this checks
    // whether GUIDED tracking is actually trustworthy instead of relying on
    // system_status alone. Only enforced once at least one EKF_STATUS_REPORT
    // has arrived - not every firmware/config streams it.
    if (h.have_ekf && (h.ekf_pos_horiz_variance > limit.ekf_pos_horiz_variance_max ||
                       h.ekf_velocity_variance > limit.ekf_velocity_variance_max)) {
        std::ostringstream oss;
        oss << "ekf pos_horiz_variance=" << std::fixed << std::setprecision(2) << h.ekf_pos_horiz_variance
            << " velocity_variance=" << h.ekf_velocity_variance;
        reasons.push_back(oss.str());
    }
    return reasons;
}

// Decodes one vehicle-health MAVLink message (SYS_STATUS/GPS_RAW_INT/
// EKF_STATUS_REPORT/HEARTBEAT) into `health`. Shared between
// approach_target()'s active-control loop and wait_for_lock()'s passive
// AUTO-mode watch further down - both need the same battery/gps/heartbeat/
// ekf picture, they just react to it differently (approach_target()
// commands LOITER/LAND on a breach; wait_for_lock() just refuses to grab
// control from AUTO while one is active).
void apply_health_message(const mavlink_message_t& msg, HealthState& health,
                           std::chrono::steady_clock::time_point now) {
    switch (msg.msgid) {
        case MAVLINK_MSG_ID_SYS_STATUS: {
            mavlink_sys_status_t s;
            mavlink_msg_sys_status_decode(&msg, &s);
            health.battery_percent = s.battery_remaining;
            health.battery_voltage_v = s.voltage_battery == 65535 ? -1 : s.voltage_battery / 1000.0;
            health.prearm_healthy = (s.onboard_control_sensors_health & MAV_SYS_STATUS_PREARM_CHECK) != 0;
            health.have_battery = true;
            health.have_sys_status = true;
            break;
        }
        case MAVLINK_MSG_ID_GPS_RAW_INT: {
            mavlink_gps_raw_int_t g;
            mavlink_msg_gps_raw_int_decode(&msg, &g);
            health.fix_type = g.fix_type;
            health.satellites = g.satellites_visible;
            health.have_gps = true;
            // Same get_gps_location.cpp reading, kept here purely for the
            // gps_moved_m verification/logging in approach_target() - it
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
            mavlink_msg_ekf_status_report_decode(&msg, &e);
            health.ekf_pos_horiz_variance = e.pos_horiz_variance;
            health.ekf_velocity_variance = e.velocity_variance;
            health.have_ekf = true;
            break;
        }
        case MAVLINK_MSG_ID_HEARTBEAT: {
            mavlink_heartbeat_t hb;
            mavlink_msg_heartbeat_decode(&msg, &hb);
            health.system_status = hb.system_status;
            health.custom_mode = hb.custom_mode;
            health.armed = is_armed_from_heartbeat(hb);
            health.have_armed = true;
            health.last_heartbeat = now;
            break;
        }
        default:
            break;
    }
}

// What approach_target() did when it returned, so a caller that might run
// it more than once (run_auto_intercept() further down) knows whether to
// keep going or stop:
//   - kLanded: a LAND command was sent from inside this call (heartbeat
//     lost, unexpected disarm, or - when resume_mode is empty - the target
//     stayed lost past target_lost_land_sec). The flight is over.
//   - kHandedBack: duration_sec elapsed, or the target stayed lost past
//     target_lost_land_sec while resume_mode was non-empty (mode was
//     switched back to resume_mode instead of landing). Safe to try again.
enum class ApproachOutcome { kLanded, kHandedBack };

// Stage 2+3 of the target-approach pipeline, plus this program's safety
// layers: an altitude ceiling (setting/safety.yaml's altitude_limit), a
// vehicle-health watch (battery/heartbeat/gps/prearm) that hands off to
// LOITER on a breach, and an unexpected-disarm watch that requests LAND
// immediately (see the HEARTBEAT case below). control.cpp is the only
// process that ever sends MAVLink commands to the vehicle, so all of this
// lives here instead of in a separate emergency process - see
// setting/safety.yaml's top comment.
//
// resume_mode: what to switch back to instead of landing when the target
// stays lost past target_lost_land_sec, or once duration_sec elapses.
// Empty (the default - the self-launched flow in main() uses this) keeps
// the original always-land-eventually contract. Non-empty is
// run_auto_intercept()'s case: the vehicle was already flying resume_mode
// (its own AUTO mission) before this call took over, so losing the target
// or running out of intercept time isn't a reason to end the flight - it's
// a reason to give the mission back control so it can keep going (and
// maybe present another target later).
ApproachOutcome approach_target(const YamlValue& track, const AltitudeLimit& alt_limit,
                                 const HealthLimit& health_limit, double duration_sec,
                                 const std::string& resume_mode = "") {
    int udp_port = static_cast<int>(track.get_long_or("udp_port", 15020));
    double cycle_ms = track.get_double_or("control_cycle_ms", 50);
    double max_forward = track.get_double_or("max_forward_speed", 1.5);
    double max_yaw_rate = track.get_double_or("max_yaw_rate", 0.6);
    double yaw_align_px = track.get_double_or("yaw_align_px", 80.0);
    double stop_distance = track.get_double_or("stop_distance", 2.0);
    double link_stale_ms = track.get_double_or("link_stale_ms", 500);
    // Continuous target loss past this many seconds escalates straight to
    // LAND, the same way heartbeat loss past land_gap_sec does below -
    // otherwise the vehicle just sits hovering in GUIDED on zero velocity
    // forever, waiting for a target that may never come back.
    double target_lost_land_sec = track.get_double_or("target_lost_land_sec", 10.0);

    // Proportional gains: forward speed reaches max_forward at
    // (distance - stop_distance) = max_forward / k_forward; yaw rate
    // reaches max_yaw_rate at a 1/k_yaw_px pixel offset from center. The
    // vehicle turns to face the target (instead of crab-walking sideways
    // while facing wherever it happened to be pointed) and only opens up
    // forward speed once roughly aligned - see the alignment taper below.
    const double k_forward = 0.5;   // 1/s
    const double k_yaw_px = 0.006;  // (rad/s) per pixel of x_px offset

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

    MavConnection& vehicle = drone::require_connection();
    request_message_interval(vehicle, MAVLINK_MSG_ID_SYS_STATUS, health_limit.poll_rate_hz);
    request_message_interval(vehicle, MAVLINK_MSG_ID_GPS_RAW_INT, health_limit.poll_rate_hz);
    request_message_interval(vehicle, MAVLINK_MSG_ID_EKF_STATUS_REPORT, health_limit.poll_rate_hz);
    // HEARTBEAT is broadcast on its own (~1 Hz) without needing a request.

    TargetRangeReceiver receiver(udp_port);
    FlightLogger logger(resolve_log_dir());
    std::cout << "타겟 접근 모드 시작 (UDP " << udp_port << ", 주기 " << cycle_ms << "ms, 최대 "
              << duration_sec << "초, 고도제한 " << alt_limit.soft_limit_m << "~"
              << alt_limit.hard_limit_m << "m)" << std::endl;

    // Zero velocity on the way out no matter how this function exits -
    // duration elapsed, target lost, or an exception - so the vehicle never
    // keeps coasting on the last command it was given.
    struct FinallyGuard {
        ~FinallyGuard() {
            try {
                drone::send_velocity_body(0.0, 0.0, 0.0, 0.0);
            } catch (...) {
            }
        }
    } finally_guard;

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

    while (true) {
        auto cycle_start = std::chrono::steady_clock::now();
        double elapsed_sec = std::chrono::duration<double>(cycle_start - start).count();
        if (duration_sec > 0 && elapsed_sec >= duration_sec) {
            std::cout << "접근 제한 시간(" << duration_sec << "초) 도달, 종료." << std::endl;
            break;
        }

        TargetRangeMsg msg;
        if (receiver.poll(msg)) {
            last = msg;
            last_valid = cycle_start;
        }
        double link_age_ms = std::chrono::duration<double, std::milli>(cycle_start - last_valid).count();
        bool link_fresh = link_age_ms <= link_stale_ms;
        if (last.altitude_valid) altitude_ever_valid = true;

        bool tracking = link_fresh && last.valid && last.found;
        if (tracking) target_lost_since = cycle_start;
        double target_lost_sec = std::chrono::duration<double>(cycle_start - target_lost_since).count();

        // Opportunistically pick up whatever vehicle-health telemetry has
        // arrived since the last tick (see kHealthPollTimeoutSec above).
        mavlink_message_t hmsg;
        if (vehicle.recv_match({MAVLINK_MSG_ID_SYS_STATUS, MAVLINK_MSG_ID_GPS_RAW_INT,
                                 MAVLINK_MSG_ID_HEARTBEAT, MAVLINK_MSG_ID_EKF_STATUS_REPORT},
                                hmsg, kHealthPollTimeoutSec)) {
            // Captured before apply_health_message() below overwrites it -
            // that function only touches health.armed on a HEARTBEAT
            // message, so for any other msgid this stays equal to
            // health.armed and the comparison after is a no-op.
            bool was_armed = health.have_armed && health.armed;
            apply_health_message(hmsg, health, cycle_start);

            // Unexpected disarm: this loop never disarms on its own
            // (arm_disarm(false) only ever happens outside
            // approach_target(), after it returns), so any true -> false
            // transition observed here is external - a crash, a firmware
            // failsafe cutting motors, physical damage, etc. Request LAND
            // immediately (best-effort; if it's truly unresponsive this
            // does nothing, but if it's still listening it can only help)
            // and stop - there is nothing else this loop can usefully
            // command with the motors off.
            if (was_armed && health.have_armed && !health.armed) {
                std::cout << "[EMERGENCY] 예상치 못한 disarm 감지 (armed: true -> false) - "
                             "긴급 LAND 명령 전송 후 종료"
                          << std::endl;
                logger.log(elapsed_sec, mode_string(health.custom_mode), false, false, last.distance_m,
                           0, 0, 0, last.altitude_m, "", health.battery_percent,
                           health.battery_voltage_v, health.fix_type, health.satellites,
                           health.prearm_healthy, health.system_status, true,
                           "unexpected disarm mid-flight", "UNEXPECTED_DISARM", health.lat,
                           health.lon, gps_moved_m);
                try {
                    drone::land();
                } catch (const std::exception& e) {
                    std::cerr << "LAND 명령 전송 실패: " << e.what() << std::endl;
                }
                return ApproachOutcome::kLanded;
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
            logger.log(elapsed_sec, mode_string(health.custom_mode), health.armed, false, last.distance_m, 0,
                       0, 0, last.altitude_m, "", health.battery_percent, health.battery_voltage_v,
                       health.fix_type, health.satellites, health.prearm_healthy, health.system_status, true,
                       "heartbeat gap exceeded land_gap_sec", "HEARTBEAT_LAND_ESCALATION", health.lat,
                       health.lon, gps_moved_m);
            try {
                drone::land();
            } catch (const std::exception& e) {
                std::cerr << "LAND 명령 전송 실패: " << e.what() << std::endl;
            }
            return ApproachOutcome::kLanded;
        }

        // Same escalation as heartbeat loss above, for the same reason:
        // without this, losing the target just means vx/vy/yaw_rate stay
        // at 0 forever (see the tracking block below) - the vehicle sits
        // hovering in GUIDED indefinitely instead of ever coming down.
        if (target_lost_sec > target_lost_land_sec) {
            if (!resume_mode.empty()) {
                std::cout << "[AUTO_INTERCEPT] 타겟 로스트 " << std::fixed << std::setprecision(1)
                          << target_lost_sec << "s > target_lost_land_sec " << target_lost_land_sec
                          << "s - " << resume_mode << " 로 복귀" << std::endl;
                logger.log(elapsed_sec, mode_string(health.custom_mode), health.armed, false,
                           last.distance_m, 0, 0, 0, last.altitude_m, "", health.battery_percent,
                           health.battery_voltage_v, health.fix_type, health.satellites,
                           health.prearm_healthy, health.system_status, false, "",
                           "TARGET_LOST_RESUME_" + resume_mode, health.lat, health.lon, gps_moved_m);
                try {
                    drone::set_mode(resume_mode);
                } catch (const std::exception& e) {
                    std::cerr << resume_mode << " 전환 명령 전송 실패: " << e.what() << std::endl;
                }
                return ApproachOutcome::kHandedBack;
            }
            std::cout << "[EMERGENCY] 타겟 로스트 " << std::fixed << std::setprecision(1)
                      << target_lost_sec << "s > target_lost_land_sec " << target_lost_land_sec
                      << "s - 긴급 LAND 명령 전송 후 종료" << std::endl;
            logger.log(elapsed_sec, mode_string(health.custom_mode), health.armed, false, last.distance_m, 0,
                       0, 0, last.altitude_m, "", health.battery_percent, health.battery_voltage_v,
                       health.fix_type, health.satellites, health.prearm_healthy, health.system_status, true,
                       "target lost exceeded target_lost_land_sec", "TARGET_LOST_LAND_ESCALATION",
                       health.lat, health.lon, gps_moved_m);
            try {
                drone::land();
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
        emergency_reason.clear();
        for (size_t i = 0; i < reasons.size(); ++i) {
            if (i > 0) emergency_reason += "; ";
            emergency_reason += reasons[i];
        }

        auto next_tick = cycle_start + std::chrono::duration<double>(cycle_ms / 1000.0);
        std::string mode_name = mode_string(health.custom_mode);

        if (emergency_active) {
            std::string event;
            if (!in_loiter) {
                std::cout << "[EMERGENCY] " << emergency_reason << ": LOITER로 전환" << std::endl;
                drone::set_mode("LOITER");
                in_loiter = true;
                event = "LOITER_ENTER";
            }
            loiter_since = cycle_start;  // keep refreshing while active
            logger.log(elapsed_sec, mode_name, health.armed, false, last.distance_m, 0, 0, 0,
                       last.altitude_m, "", health.battery_percent, health.battery_voltage_v,
                       health.fix_type, health.satellites, health.prearm_healthy, health.system_status,
                       true, emergency_reason, event, health.lat, health.lon, gps_moved_m);
            std::this_thread::sleep_until(next_tick);
            continue;
        }

        if (in_loiter) {
            double held_sec = std::chrono::duration<double>(cycle_start - loiter_since).count();
            if (held_sec < kMinLoiterHoldSec) {
                std::cout << std::fixed << std::setprecision(1) << "LOITER 최소 유지 중 (" << held_sec
                          << "/" << kMinLoiterHoldSec << "s)" << std::endl;
                logger.log(elapsed_sec, mode_name, health.armed, false, last.distance_m, 0, 0, 0,
                           last.altitude_m, "", health.battery_percent, health.battery_voltage_v,
                           health.fix_type, health.satellites, health.prearm_healthy, health.system_status,
                           false, "", "LOITER_HOLD", health.lat, health.lon, gps_moved_m);
                std::this_thread::sleep_until(next_tick);
                continue;
            }
            std::cout << "위험 상태 해제, 최소 유지시간 경과 - GUIDED로 복귀." << std::endl;
            drone::set_mode("GUIDED");
            in_loiter = false;
            logger.log(elapsed_sec, mode_name, health.armed, false, last.distance_m, 0, 0, 0,
                       last.altitude_m, "", health.battery_percent, health.battery_voltage_v,
                       health.fix_type, health.satellites, health.prearm_healthy, health.system_status,
                       false, "", "LOITER_EXIT", health.lat, health.lon, gps_moved_m);
        }

        double vx = 0.0, yaw_rate = 0.0;  // body frame: +x forward; yaw +clockwise (right)
        if (tracking) {
            double approach_distance = std::max(0.0, static_cast<double>(last.distance_m) - stop_distance);
            double px_offset = static_cast<double>(last.x_px);
            yaw_rate = std::clamp(k_yaw_px * px_offset, -max_yaw_rate, max_yaw_rate);
            // Turn to face the target first: forward speed tapers to 0 as
            // |px_offset| grows past yaw_align_px, and ramps back in as the
            // yaw correction above brings the target back toward center. A
            // hard on/off cutoff at the threshold would make the vehicle
            // stutter forward/stop right at the boundary, so this fades
            // linearly instead.
            double align = std::clamp(1.0 - std::abs(px_offset) / yaw_align_px, 0.0, 1.0);
            vx = std::min(max_forward, k_forward * approach_distance) * align;
        }

        double vz = 0.0;  // body frame: +z down
        std::string alt_status;
        if (link_fresh) {
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

        drone::send_velocity_body(vx, 0.0, vz, yaw_rate);

        std::cout << std::fixed << std::setprecision(2) << "t=" << elapsed_sec << "s "
                  << (tracking ? "TRACK" : "LOST ") << " dist=" << last.distance_m << "m vx=" << vx
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
                   "", "", health.lat, health.lon, gps_moved_m);

        std::this_thread::sleep_until(next_tick);
    }

    // Reached only via the duration_sec break at the top of the loop -
    // every early exit above already returned directly.
    if (!resume_mode.empty()) {
        std::cout << "[AUTO_INTERCEPT] 접근 제한 시간(" << duration_sec << "초) 도달 - " << resume_mode
                  << " 로 복귀" << std::endl;
        try {
            drone::set_mode(resume_mode);
        } catch (const std::exception& e) {
            std::cerr << resume_mode << " 전환 명령 전송 실패: " << e.what() << std::endl;
        }
    }
    return ApproachOutcome::kHandedBack;
}

// Blocks until the vehicle is armed and in AUTO mode - flying its own
// mission, uploaded separately (e.g. from a GCS), never something this
// program commands. Unlike wait_for_heartbeat() above, this has no overall
// timeout: run_auto_intercept() is meant to be started once and left
// running, patiently waiting for the operator to switch into AUTO whenever
// they're ready to begin the mission, not to give up after a fixed window.
// A heartbeat gap past max_heartbeat_gap_sec still throws - if the link
// itself is gone, waiting silently forever would just hide that instead of
// surfacing it.
void wait_for_auto_armed(MavConnection& vehicle, double max_heartbeat_gap_sec) {
    auto last_heartbeat = std::chrono::steady_clock::now();
    auto last_status_print = last_heartbeat - std::chrono::seconds(10);
    while (true) {
        auto now = std::chrono::steady_clock::now();
        double gap = std::chrono::duration<double>(now - last_heartbeat).count();
        if (gap > max_heartbeat_gap_sec) {
            throw std::runtime_error("AUTO 대기 중: heartbeat " + std::to_string(max_heartbeat_gap_sec) +
                                      "초 이상 끊김 - 픽스호크 응답 없음");
        }
        mavlink_message_t msg;
        if (vehicle.recv_match({MAVLINK_MSG_ID_HEARTBEAT}, msg, 0.5)) {
            mavlink_heartbeat_t hb;
            mavlink_msg_heartbeat_decode(&msg, &hb);
            last_heartbeat = now;
            if (is_armed_from_heartbeat(hb) && hb.custom_mode == copter_mode_mapping().at("AUTO")) {
                return;
            }
        }
        if (std::chrono::duration<double>(now - last_status_print).count() >= 5.0) {
            std::cout << "[AUTO_INTERCEPT] AUTO 전환/arm 대기 중..." << std::endl;
            last_status_print = now;
        }
    }
}

// Polls target_distance.cpp's UDP link and the vehicle's own telemetry
// while AUTO still has control, waiting for a target lock solid enough to
// justify taking over: lock_confirm_sec of continuous tracking (same
// `tracking` definition approach_target() uses - fresh link, valid, found)
// while current health is clean (evaluate_health_breach() empty) and the
// vehicle is still armed and in AUTO. A stray single-frame detection, or a
// lock that shows up mid health-breach, won't trigger a takeover.
//
// Returns false - without ever touching mode or sending a velocity command
// - if AUTO/armed drops out from under us first (operator switched modes,
// disarmed, landed, RTL'd, ...) or the heartbeat itself goes stale; the
// caller goes back to wait_for_auto_armed() either way rather than treating
// it as a fatal error - the operator is still flying, just not ready for a
// handoff yet.
bool wait_for_lock(MavConnection& vehicle, TargetRangeReceiver& receiver, const HealthLimit& health_limit,
                    double link_stale_ms, double lock_confirm_sec, HealthState& health) {
    constexpr double kPollTimeoutSec = 0.1;
    auto last_valid = std::chrono::steady_clock::now() - std::chrono::seconds(10);
    TargetRangeMsg last{};
    bool locking = false;
    auto lock_since = std::chrono::steady_clock::now();
    auto last_status_print = std::chrono::steady_clock::now() - std::chrono::seconds(10);

    while (true) {
        auto now = std::chrono::steady_clock::now();

        TargetRangeMsg msg;
        if (receiver.poll(msg)) {
            last = msg;
            last_valid = now;
        }
        double link_age_ms = std::chrono::duration<double, std::milli>(now - last_valid).count();
        bool tracking = link_age_ms <= link_stale_ms && last.valid && last.found;

        mavlink_message_t hmsg;
        if (vehicle.recv_match({MAVLINK_MSG_ID_SYS_STATUS, MAVLINK_MSG_ID_GPS_RAW_INT,
                                 MAVLINK_MSG_ID_HEARTBEAT, MAVLINK_MSG_ID_EKF_STATUS_REPORT},
                                hmsg, kPollTimeoutSec)) {
            apply_health_message(hmsg, health, now);
        }

        double heartbeat_gap = std::chrono::duration<double>(now - health.last_heartbeat).count();
        if (heartbeat_gap > health_limit.max_heartbeat_gap_sec) {
            std::cerr << "[AUTO_INTERCEPT] heartbeat gap " << std::fixed << std::setprecision(1)
                      << heartbeat_gap << "s > " << health_limit.max_heartbeat_gap_sec
                      << "s - 대기 중단, 연결 회복 대기." << std::endl;
            return false;
        }
        if (!health.have_armed || !health.armed ||
            health.custom_mode != copter_mode_mapping().at("AUTO")) {
            return false;
        }

        bool healthy = evaluate_health_breach(health, health_limit, now).empty();
        if (tracking && healthy) {
            if (!locking) {
                locking = true;
                lock_since = now;
            }
            double held = std::chrono::duration<double>(now - lock_since).count();
            if (held >= lock_confirm_sec) return true;
        } else {
            locking = false;
        }

        if (std::chrono::duration<double>(now - last_status_print).count() >= 5.0) {
            std::cout << "[AUTO_INTERCEPT] AUTO 비행 중 - 타겟 대기 (tracking="
                      << (tracking ? "Y" : "N") << " healthy=" << (healthy ? "Y" : "N") << ")"
                      << std::endl;
            last_status_print = now;
        }
    }
}

// Entry point for "픽스호크가 AUTO 모드로 미션을 날고 있다가, YOLO가 타겟을
// 확정 인식하면 그쪽으로 접근한다" (scripts/full_mission.sh, control
// --auto-intercept). Unlike the self-launched flow in main() below, this
// never arms or takes off itself - the vehicle is expected to already be
// flying an externally-uploaded AUTO mission by the time a target locks on.
//
// Loop:
//   1. wait_for_auto_armed() - sit and watch until AUTO + armed
//   2. wait_for_lock()       - keep watching AUTO's flight, now also
//                               polling target_distance.cpp, until a target
//                               lock is confirmed
//   3. switch to GUIDED, run approach_target() with resume_mode="AUTO" so
//      losing the target or running out of intercept time hands control
//      back to the mission instead of landing
//   4. on kHandedBack, cool down briefly then go back to step 1 (mode is
//      already AUTO again by then, so this re-enters immediately); on
//      kLanded (heartbeat lost / unexpected disarm), stop - the flight is
//      over.
void run_auto_intercept(const YamlValue& track, const AltitudeLimit& alt_limit,
                         const HealthLimit& health_limit) {
    double link_stale_ms = track.get_double_or("link_stale_ms", 500);
    double lock_confirm_sec = track.get_double_or("lock_confirm_sec", 1.0);
    double intercept_duration_sec = track.get_double_or("intercept_approach_duration_sec", 30.0);
    double reacquire_cooldown_sec = track.get_double_or("reacquire_cooldown_sec", 3.0);
    int udp_port = static_cast<int>(track.get_long_or("udp_port", 15020));
    const std::string kResumeMode = "AUTO";

    MavConnection& vehicle = drone::require_connection();
    request_message_interval(vehicle, MAVLINK_MSG_ID_SYS_STATUS, health_limit.poll_rate_hz);
    request_message_interval(vehicle, MAVLINK_MSG_ID_GPS_RAW_INT, health_limit.poll_rate_hz);
    request_message_interval(vehicle, MAVLINK_MSG_ID_EKF_STATUS_REPORT, health_limit.poll_rate_hz);

    TargetRangeReceiver receiver(udp_port);

    std::cout << "AUTO 요격 모드 시작 (UDP " << udp_port << ", 잠금 확인 " << lock_confirm_sec
              << "s, 접근 제한 " << intercept_duration_sec << "s, 재탐지 대기 " << reacquire_cooldown_sec
              << "s)" << std::endl;

    while (true) {
        wait_for_auto_armed(vehicle, health_limit.max_heartbeat_gap_sec);
        std::cout << "[AUTO_INTERCEPT] AUTO + armed 확인됨 - 타겟 확정 대기." << std::endl;

        HealthState health;
        health.last_heartbeat = std::chrono::steady_clock::now();
        if (!wait_for_lock(vehicle, receiver, health_limit, link_stale_ms, lock_confirm_sec, health)) {
            std::cout << "[AUTO_INTERCEPT] AUTO/armed 상태 이탈 - 대기 상태로 복귀." << std::endl;
            continue;
        }

        std::cout << "[AUTO_INTERCEPT] 타겟 확정 (" << lock_confirm_sec << "s 유지) - GUIDED로 전환합니다."
                  << std::endl;
        if (!drone::set_mode("GUIDED")) {
            std::cerr << "[AUTO_INTERCEPT] GUIDED 전환 명령 전송 실패 - 대기 상태로 복귀." << std::endl;
            continue;
        }
        try {
            wait_for_heartbeat(vehicle, 5.0, health_limit.max_heartbeat_gap_sec, "GUIDED 전환 (요격)",
                                [](const mavlink_heartbeat_t& hb) {
                                    return hb.custom_mode == copter_mode_mapping().at("GUIDED");
                                });
        } catch (const std::exception& e) {
            std::cerr << "[AUTO_INTERCEPT] " << e.what() << " - " << kResumeMode << " 복귀 시도."
                      << std::endl;
            try {
                drone::set_mode(kResumeMode);
            } catch (const std::exception& e2) {
                std::cerr << "[AUTO_INTERCEPT] " << kResumeMode << " 전환 명령 전송 실패: " << e2.what()
                          << std::endl;
            }
            continue;
        }

        ApproachOutcome outcome =
            approach_target(track, alt_limit, health_limit, intercept_duration_sec, kResumeMode);
        if (outcome == ApproachOutcome::kLanded) {
            std::cout << "[AUTO_INTERCEPT] 비상 착륙으로 종료 - 프로그램을 마칩니다." << std::endl;
            return;
        }

        std::cout << "[AUTO_INTERCEPT] " << kResumeMode << " 로 복귀함 - " << reacquire_cooldown_sec
                  << "s 후 재탐지 대기." << std::endl;
        std::this_thread::sleep_for(std::chrono::duration<double>(reacquire_cooldown_sec));
    }
}

}  // namespace

int main(int argc, char** argv) {
    bool auto_intercept = false;
    for (int i = 1; i < argc; ++i) {
        if (std::string(argv[i]) == "--auto-intercept") auto_intercept = true;
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

        YamlValue ports = drone::load_port_settings();
        std::string mav_address = with_port(settings["real"]["proxy_udp"]["address"].as_string(),
                                             ports.get_long_or("mavlink_control", 14550));
        drone::connect(mav_address);
        MavConnection& vehicle = drone::require_connection();

        if (auto_intercept) {
            // AUTO 모드로 이미 날고 있는 미션에 끼어드는 흐름 - 이 프로그램은
            // 여기서 arm/이륙을 직접 하지 않는다 (미션이 이미 하고 있음).
            run_auto_intercept(track, alt_limit, health_limit);
            return 0;
        }

        const double target_alt = 4.5;

        if (!drone::set_mode("GUIDED")) {
            throw std::runtime_error("GUIDED 모드 전환 명령 전송 실패 (지원하지 않는 모드)");
        }
        wait_for_heartbeat(vehicle, 5.0, health_limit.max_heartbeat_gap_sec, "GUIDED 모드 전환",
                            [](const mavlink_heartbeat_t& hb) {
                                return hb.custom_mode == copter_mode_mapping().at("GUIDED");
                            });
        std::cout << "GUIDED 모드 확인됨." << std::endl;

        drone::arm_disarm(true);
        wait_for_heartbeat(vehicle, 5.0, health_limit.max_heartbeat_gap_sec, "시동(arm) 확인",
                            [](const mavlink_heartbeat_t& hb) { return is_armed_from_heartbeat(hb); });
        std::cout << "시동 완료 확인됨." << std::endl;

        drone::takeoff(target_alt);
        wait_for_heartbeat(vehicle, 10.0, health_limit.max_heartbeat_gap_sec, "이륙 대기");

        ApproachOutcome outcome = approach_target(track, alt_limit, health_limit,
                                                   track.get_double_or("approach_duration_sec", 60));
        // kLanded means approach_target() already sent LAND itself
        // (heartbeat lost / unexpected disarm) - sending it again here
        // would just be a redundant no-op, but skipping it makes the one
        // remaining call site (the normal duration_sec-elapsed path)
        // unambiguous about who's responsible for landing.
        if (outcome != ApproachOutcome::kLanded) {
            drone::land();
            std::cout << "착륙 중..." << std::endl;
        }
        return 0;
    } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return 1;
    }
}
