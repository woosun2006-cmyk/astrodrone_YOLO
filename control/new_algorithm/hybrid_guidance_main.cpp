// Entry point for the coordinate-based approach guidance law
// (Document/algorithm-renewer.md section 20, 2026-08-14 revision -
// constant-speed cruise to a 5m shell, re-verify hold, exponential decel to
// a 1m hard stop). Self-launches like
// control.cpp's default (non-auto-intercept) flow: sets GUIDED, arms,
// takes off, then runs its own approach loop - it does not reuse
// control.cpp's run_auto_intercept()/wait_for_lock() path (control/README.md
// documents a HealthState race in there that this program sidesteps simply
// by not depending on it), so it should be launched instead of `control`,
// not alongside it - both would fight over the same target_track.udp_port
// receiver socket.
//
// Everything below either calls into control/'s unmodified public headers
// (drone_lib.hpp, target_link.hpp, yaml_settings.hpp) or hybrid_guidance.hpp
// in this folder. The small amount of duplication from control.cpp
// (request_message_interval, the health-breach checks, the altitude-limit
// descent law) is because those live in control.cpp's anonymous namespace,
// not in a shared header - see this folder's README.md.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits.h>
#include <string>
#include <thread>
#include <unistd.h>

#include "../drone_lib.hpp"
#include "../target_link.hpp"
#include "hybrid_guidance.hpp"

namespace {

void request_message_interval(MavConnection& connection, uint32_t message_id, double frequency_hz) {
    int64_t interval_us = static_cast<int64_t>(1'000'000 / frequency_hz);
    mavlink_message_t msg;
    mavlink_msg_command_long_pack(255, 0, &msg, connection.target_system(), connection.target_component(),
                                   MAV_CMD_SET_MESSAGE_INTERVAL, 0, static_cast<float>(message_id),
                                   static_cast<float>(interval_us), 0, 0, 0, 0, 0);
    connection.send(msg);
}

// Same lookup yaml_settings.cpp's (unexported) load_setting_file() uses for
// the built-in settings files, applied to this new file. This binary's
// build output directory (control/new_algorithm/CMakeLists.txt's
// RUNTIME_OUTPUT_DIRECTORY) is set to sit two levels below the repo root,
// same depth as control/build/, so this candidate list finds
// setting/hybrid_guidance.yaml the same way drone::load_mavlink_settings()
// etc. already find their files from that depth.
YamlValue load_hybrid_guidance_settings() {
    char exe_path[PATH_MAX];
    ssize_t len = readlink("/proc/self/exe", exe_path, sizeof(exe_path) - 1);
    std::string exe_dir = ".";
    if (len != -1) {
        exe_path[len] = '\0';
        std::string full(exe_path);
        exe_dir = full.substr(0, full.find_last_of('/'));
    }
    std::string candidates[] = {
        exe_dir + "/../../setting/hybrid_guidance.yaml",
        exe_dir + "/../setting/hybrid_guidance.yaml",
    };
    for (const auto& candidate : candidates) {
        std::ifstream probe(candidate);
        if (probe.good()) return parse_yaml_file(candidate);
    }
    throw std::runtime_error("hybrid_guidance.yaml not found relative to executable");
}

void wait_for_mode(MavConnection& vehicle, const std::string& mode, double timeout_sec) {
    uint32_t target_mode = copter_mode_mapping().at(mode);
    auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_sec);
    while (std::chrono::steady_clock::now() < deadline) {
        mavlink_message_t msg;
        if (vehicle.recv_match({MAVLINK_MSG_ID_HEARTBEAT}, msg, 0.2)) {
            mavlink_heartbeat_t hb;
            mavlink_msg_heartbeat_decode(&msg, &hb);
            if (hb.custom_mode == target_mode) return;
        }
    }
    throw std::runtime_error(mode + " 전환 확인 실패 (" + std::to_string(timeout_sec) + "초 초과)");
}

void wait_for_armed(MavConnection& vehicle, double timeout_sec) {
    auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_sec);
    while (std::chrono::steady_clock::now() < deadline) {
        mavlink_message_t msg;
        if (vehicle.recv_match({MAVLINK_MSG_ID_HEARTBEAT}, msg, 0.2)) {
            mavlink_heartbeat_t hb;
            mavlink_msg_heartbeat_decode(&msg, &hb);
            if (is_armed_from_heartbeat(hb)) return;
        }
    }
    throw std::runtime_error("시동(arm) 확인 실패 (" + std::to_string(timeout_sec) + "초 초과)");
}

// Compact stand-in for control.cpp's HealthState/evaluate_health_breach -
// same fields, same thresholds (both read from the unmodified
// setting/safety.yaml), just without the CSV flight logger, since that's
// not needed to validate this guidance law in Gazebo.
struct HealthSnapshot {
    std::chrono::steady_clock::time_point last_heartbeat;
    int battery_percent = -1;
    double battery_voltage_v = -1;
    uint8_t fix_type = 0;
    uint8_t satellites = 255;
    bool prearm_healthy = true;
    uint8_t system_status = MAV_STATE_STANDBY;
    bool armed = false;
};

void apply_health(const mavlink_message_t& msg, HealthSnapshot& health,
                   std::chrono::steady_clock::time_point now) {
    switch (msg.msgid) {
        case MAVLINK_MSG_ID_SYS_STATUS: {
            mavlink_sys_status_t s;
            mavlink_msg_sys_status_decode(&msg, &s);
            health.battery_percent = s.battery_remaining;
            health.battery_voltage_v = s.voltage_battery == 65535 ? -1 : s.voltage_battery / 1000.0;
            health.prearm_healthy = (s.onboard_control_sensors_health & MAV_SYS_STATUS_PREARM_CHECK) != 0;
            break;
        }
        case MAVLINK_MSG_ID_GPS_RAW_INT: {
            mavlink_gps_raw_int_t g;
            mavlink_msg_gps_raw_int_decode(&msg, &g);
            health.fix_type = g.fix_type;
            health.satellites = g.satellites_visible;
            break;
        }
        case MAVLINK_MSG_ID_HEARTBEAT: {
            mavlink_heartbeat_t hb;
            mavlink_msg_heartbeat_decode(&msg, &hb);
            health.system_status = hb.system_status;
            health.armed = is_armed_from_heartbeat(hb);
            health.last_heartbeat = now;
            break;
        }
        default:
            break;
    }
}

bool health_breach(const HealthSnapshot& h, const YamlValue& safety) {
    long min_battery_percent = safety["battery_limit"].get_long_or("min_percent", 20);
    double min_battery_voltage_v = safety["battery_limit"].get_double_or("min_voltage_v", 14.0);
    long min_fix_type = safety["gps_limit"].get_long_or("min_fix_type", 3);
    long min_satellites = safety["gps_limit"].get_long_or("min_satellites", 6);

    if (h.battery_percent >= 0 && h.battery_percent < min_battery_percent) return true;
    if (h.battery_voltage_v >= 0 && h.battery_voltage_v < min_battery_voltage_v) return true;
    if (h.satellites != 255 && (h.fix_type < min_fix_type || h.satellites < min_satellites)) return true;
    if (!h.prearm_healthy) return true;
    if (h.system_status == MAV_STATE_CRITICAL || h.system_status == MAV_STATE_EMERGENCY) return true;
    return false;
}

std::string mode_name(GuidanceMode mode) {
    switch (mode) {
        case GuidanceMode::kHold: return "HOLD";
        case GuidanceMode::kCruise: return "CRUISE";
        case GuidanceMode::kReverify: return "REVERIFY";
        case GuidanceMode::kDecel: return "DECEL";
        case GuidanceMode::kStopped: return "STOPPED";
    }
    return "?";
}

}  // namespace

int main() {
    try {
        YamlValue mav = drone::load_mavlink_settings();
        YamlValue safety = drone::load_safety_settings();
        YamlValue ports = drone::load_port_settings();
        YamlValue hg = load_hybrid_guidance_settings();
        YamlValue track = mav["target_track"];
        YamlValue alt_limit_cfg = safety["altitude_limit"];

        HybridGuidanceConfig cfg;
        cfg.shell_radius_m = hg.get_double_or("shell_radius_m", cfg.shell_radius_m);
        cfg.stop_radius_m = hg.get_double_or("stop_radius_m", cfg.stop_radius_m);
        cfg.cruise_speed_mps = hg.get_double_or("cruise_speed_mps", cfg.cruise_speed_mps);
        cfg.decel_rate_per_m = hg.get_double_or("decel_rate_per_m", cfg.decel_rate_per_m);
        cfg.reverify_hold_sec = hg.get_double_or("reverify_hold_sec", cfg.reverify_hold_sec);
        cfg.max_yaw_rate = hg.get_double_or("max_yaw_rate", cfg.max_yaw_rate);
        cfg.k_yaw_px = hg.get_double_or("k_yaw_px", cfg.k_yaw_px);
        // Reused, not duplicated: same calibration knob target_distance.cpp
        // already uses to build the TargetRangeMsg this program consumes.
        cfg.focal_length_px = track.get_double_or("pixel_focal_length_px", cfg.focal_length_px);

        double takeoff_altitude_m = hg.get_double_or("takeoff_altitude_m", 4.5);
        double approach_duration_sec = hg.get_double_or("approach_duration_sec", 60.0);
        double control_cycle_ms = hg.get_double_or("control_cycle_ms", 50.0);
        double link_stale_ms = hg.get_double_or("link_stale_ms", track.get_double_or("link_stale_ms", 500.0));
        double target_lost_land_sec = hg.get_double_or("target_lost_land_sec", 10.0);
        double heartbeat_timeout = mav.get_double_or("heartbeat_timeout", 20.0);
        double land_gap_sec = safety["heartbeat_limit"].get_double_or("land_gap_sec", 5.0);
        double poll_rate_hz = alt_limit_cfg.get_double_or("poll_rate_hz", 5.0);

        double soft_limit_m = alt_limit_cfg.get_double_or("soft_limit_m", 4.0);
        double hard_limit_m = alt_limit_cfg.get_double_or("hard_limit_m", 5.0);
        double descent_speed_mps = alt_limit_cfg.get_double_or("descent_speed_mps", 0.5);
        double recovery_margin_m = alt_limit_cfg.get_double_or("recovery_margin_m", 0.2);
        constexpr double kAltGain = 0.5;
        constexpr double kAltMinDescent = 0.1;

        std::string mav_address =
            with_port(mav["real"]["proxy_udp"]["address"].as_string(), ports.get_long_or("mavlink_control", 14550));
        drone::connect(mav_address, heartbeat_timeout);
        MavConnection& vehicle = drone::require_connection();

        request_message_interval(vehicle, MAVLINK_MSG_ID_SYS_STATUS, poll_rate_hz);
        request_message_interval(vehicle, MAVLINK_MSG_ID_GPS_RAW_INT, poll_rate_hz);

        if (!drone::set_mode("GUIDED")) throw std::runtime_error("GUIDED 모드 전환 명령 전송 실패");
        wait_for_mode(vehicle, "GUIDED", 5.0);
        std::cout << "GUIDED 모드 확인됨." << std::endl;

        drone::arm_disarm(true);
        wait_for_armed(vehicle, 5.0);
        std::cout << "시동 완료 확인됨." << std::endl;

        drone::takeoff(takeoff_altitude_m);
        std::this_thread::sleep_for(std::chrono::duration<double>(10.0));

        int udp_port = static_cast<int>(track.get_long_or("udp_port", 15020));
        TargetRangeReceiver receiver(udp_port);
        std::cout << "하이브리드 가이던스 시작 (UDP " << udp_port << ", 주기 " << control_cycle_ms
                  << "ms, shell=" << cfg.shell_radius_m << "m, stop=" << cfg.stop_radius_m << "m)"
                  << std::endl;

        HealthSnapshot health;
        auto start = std::chrono::steady_clock::now();
        health.last_heartbeat = start;
        GuidanceState guidance_state;
        bool in_loiter = false;
        auto loiter_since = start;
        auto last_valid = start - std::chrono::seconds(10);
        auto target_lost_since = start;
        TargetRangeMsg last{};
        bool alt_enforcing = false;
        constexpr double kMinLoiterHoldSec = 3.0;

        struct FinallyGuard {
            ~FinallyGuard() {
                try {
                    drone::send_velocity_body(0.0, 0.0, 0.0, 0.0);
                } catch (...) {
                }
            }
        } finally_guard;

        while (true) {
            auto cycle_start = std::chrono::steady_clock::now();
            double elapsed_sec = std::chrono::duration<double>(cycle_start - start).count();
            if (approach_duration_sec > 0 && elapsed_sec >= approach_duration_sec) {
                std::cout << "접근 제한 시간(" << approach_duration_sec << "초) 도달, 착륙." << std::endl;
                break;
            }
            TargetRangeMsg msg;
            if (receiver.poll(msg)) {
                last = msg;
                last_valid = cycle_start;
            }
            double link_age_ms = std::chrono::duration<double, std::milli>(cycle_start - last_valid).count();
            bool link_fresh = link_age_ms <= link_stale_ms;
            bool tracking = link_fresh && last.valid && last.found;
            if (tracking) target_lost_since = cycle_start;
            double target_lost_sec = std::chrono::duration<double>(cycle_start - target_lost_since).count();

            mavlink_message_t hmsg;
            if (vehicle.recv_match({MAVLINK_MSG_ID_SYS_STATUS, MAVLINK_MSG_ID_GPS_RAW_INT,
                                     MAVLINK_MSG_ID_HEARTBEAT},
                                    hmsg, 0.005) &&
                hmsg.sysid == vehicle.target_system()) {
                apply_health(hmsg, health, cycle_start);
            }

            double heartbeat_gap = std::chrono::duration<double>(cycle_start - health.last_heartbeat).count();
            if (heartbeat_gap > land_gap_sec) {
                std::cout << "[EMERGENCY] heartbeat gap " << heartbeat_gap << "s - LAND." << std::endl;
                drone::land();
                return 0;
            }
            if (target_lost_sec > target_lost_land_sec) {
                std::cout << "[EMERGENCY] 타겟 로스트 " << target_lost_sec << "s - LAND." << std::endl;
                drone::land();
                return 0;
            }

            auto next_tick = cycle_start + std::chrono::duration<double>(control_cycle_ms / 1000.0);

            if (health_breach(health, safety)) {
                if (!in_loiter) {
                    std::cout << "[EMERGENCY] health breach - LOITER." << std::endl;
                    drone::set_mode("LOITER");
                    in_loiter = true;
                }
                loiter_since = cycle_start;
                std::this_thread::sleep_until(next_tick);
                continue;
            }
            if (in_loiter) {
                double held = std::chrono::duration<double>(cycle_start - loiter_since).count();
                if (held < kMinLoiterHoldSec) {
                    std::this_thread::sleep_until(next_tick);
                    continue;
                }
                std::cout << "위험 상태 해제 - GUIDED 복귀." << std::endl;
                drone::set_mode("GUIDED");
                in_loiter = false;
            }

            // Altitude ceiling takes priority over normal guidance, same
            // proportional-with-floor law as control.cpp's approach_target().
            double alt = last.altitude_m;
            bool alt_override = false;
            double vz = 0.0;
            if (link_fresh) {
                if (alt > hard_limit_m) {
                    alt_enforcing = true;
                    double overshoot = alt - hard_limit_m;
                    vz = std::clamp(kAltGain * overshoot, kAltMinDescent, descent_speed_mps);
                    alt_override = true;
                } else if (alt_enforcing) {
                    if (alt <= hard_limit_m - recovery_margin_m) {
                        alt_enforcing = false;
                    } else {
                        vz = kAltMinDescent;
                        alt_override = true;
                    }
                } else if (alt > soft_limit_m) {
                    std::cout << "[SOFT-LIMIT] altitude " << alt << "m" << std::endl;
                }
            }

            if (alt_override) {
                drone::send_velocity_body(0.0, 0.0, vz, 0.0);
                std::cout << std::fixed << std::setprecision(2) << "t=" << elapsed_sec
                          << "s [ALT-LIMIT] alt=" << alt << "m vz=" << vz << std::endl;
            } else {
                GuidanceCommand cmd = compute_guidance(last, tracking, cfg, guidance_state, cycle_start);
                drone::send_velocity_body(cmd.vx, cmd.vy, 0.0, cmd.yaw_rate);
                std::cout << std::fixed << std::setprecision(2) << "t=" << elapsed_sec << "s "
                          << (tracking ? "TRACK" : "LOST ") << " mode=" << mode_name(cmd.mode)
                          << " dist=" << last.distance_m << "m vx=" << cmd.vx << " vy=" << cmd.vy
                          << " alt=" << alt << "m armed=" << (health.armed ? "Y" : "N") << std::endl;
            }

            std::this_thread::sleep_until(next_tick);
        }

        drone::send_velocity_body(0.0, 0.0, 0.0, 0.0);
        drone::land();
        std::cout << "착륙 중..." << std::endl;
        return 0;
    } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return 1;
    }
}
