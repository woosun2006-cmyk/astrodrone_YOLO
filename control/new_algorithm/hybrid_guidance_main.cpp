// Entry point for the coordinate-based approach guidance law
// (Document/algorithm-renewer.md section 20, 2026-08-14 revision -
// constant-speed cruise to a 5m shell, re-verify hold, exponential decel to
// a 1m hard stop). Does NOT arm or take off itself: the vehicle is expected
// to already be armed and flying its own externally-uploaded AUTO mission
// (waypoints, takeoff included) by the time this program is launched. This
// program watches that AUTO flight until a target locks on, then switches
// to GUIDED and takes over - same shape as control.cpp's
// run_auto_intercept()/wait_for_lock(), re-implemented here (not linked
// against control.cpp) because those are private functions in
// control.cpp's anonymous namespace. gazeboSim/README.md's "Resolved
// (2026-08-12)" section records a HealthState race that used to live in
// that path (a fresh HealthState created on every outer-loop pass, so
// wait_for_lock() checked health.have_armed before a HEARTBEAT had a
// chance to arrive and bailed out instantly). It was fixed in control.cpp
// on 2026-08-12 by commit 7d85d66 (Document/developinglogMJ.md) - but not
// by hoisting the HealthState out of the loop: run_auto_intercept() still
// declares a fresh one per pass (control.cpp:932-933). What closed the race
// there is wait_for_auto_armed() seeding the caller's HealthState from the
// HEARTBEAT it had already decoded, plus wait_for_lock() taking
// HealthState& by reference (control.cpp:829). This file closes the same
// race a different way: `health` below is declared exactly once and never
// recreated, so it always reflects whatever HEARTBEATs have actually been
// seen so far. (control/README.md has no such section - only
// gazeboSim/README.md ever carried it.)
//
// Should be launched instead of `control`, not alongside it - both would
// fight over the same target_track.udp_port receiver socket.
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
    // present/healthy mirror MAVLink's own two-bit convention
    // (onboard_control_sensors_present/_health): a sensor bit that isn't
    // present can't be assumed healthy *or* unhealthy - same "can't be
    // assumed safe or unsafe" rule this codebase already applies to an
    // unreported battery_percent/voltage (see target_distance.cpp/
    // control.cpp). Verified against this Gazebo/SITL setup: its
    // SYS_STATUS never sets PREARM_CHECK's present bit at all, so treating
    // health=0 there as "unhealthy" was a false LOITER trip - present=false
    // now short-circuits it instead.
    bool prearm_present = false;
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
            // 65535 is MAVLink's explicit "unknown"; 0 mV is what a flight
            // controller with no battery monitor configured actually sends.
            // Both mean "unreported", not "0 V" - mapping 0 to a real reading
            // made health_breach() below see a critically flat battery and
            // hand off to LOITER one cycle after taking over. That contradicts
            // the "can't be assumed safe or unsafe" rule this file already
            // applies to battery_remaining == -1 (see the struct comment above).
            health.battery_voltage_v = (s.voltage_battery == 65535 || s.voltage_battery == 0)
                                            ? -1
                                            : s.voltage_battery / 1000.0;
            health.prearm_present = (s.onboard_control_sensors_present & MAV_SYS_STATUS_PREARM_CHECK) != 0;
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

// Blocks until the vehicle is armed and in AUTO mode - flying its own
// mission, uploaded separately, never something this program commands.
// Keeps updating the SAME `health` the caller passes in (never a fresh one
// per call), so health.armed/health.last_heartbeat stay valid once this
// returns - the structural fix for the HealthState race documented in
// control/README.md's wait_for_lock()/run_auto_intercept(). No overall
// timeout: this is meant to be started once and left running, patiently
// waiting for the operator to switch into AUTO. A heartbeat gap past
// max_heartbeat_gap_sec still throws - if the link itself is gone, waiting
// silently forever would just hide that.
void wait_for_auto_armed(MavConnection& vehicle, HealthSnapshot& health, double max_heartbeat_gap_sec) {
    uint32_t auto_mode = copter_mode_mapping().at("AUTO");
    auto last_status_print = std::chrono::steady_clock::now() - std::chrono::seconds(10);
    uint32_t custom_mode = 0;
    while (true) {
        auto now = std::chrono::steady_clock::now();
        double gap = std::chrono::duration<double>(now - health.last_heartbeat).count();
        if (gap > max_heartbeat_gap_sec) {
            throw std::runtime_error("AUTO 대기 중: heartbeat " + std::to_string(max_heartbeat_gap_sec) +
                                      "초 이상 끊김 - 픽스호크 응답 없음");
        }
        mavlink_message_t msg;
        if (vehicle.recv_match({MAVLINK_MSG_ID_HEARTBEAT}, msg, 0.5) && msg.sysid == vehicle.target_system()) {
            mavlink_heartbeat_t hb;
            mavlink_msg_heartbeat_decode(&msg, &hb);
            custom_mode = hb.custom_mode;
            apply_health(msg, health, now);
            if (health.armed && custom_mode == auto_mode) return;
        }
        if (std::chrono::duration<double>(now - last_status_print).count() >= 5.0) {
            std::cout << "[HYBRID_GUIDANCE] AUTO 전환/arm 대기 중..." << std::endl;
            last_status_print = now;
        }
    }
}

// Polls target_distance.cpp's UDP link while AUTO still has control,
// waiting for a target lock solid enough to justify taking over:
// confirm_hold_sec of continuous tracking while the vehicle is still armed
// and in AUTO. Returns false - without sending any command - if AUTO/armed
// drops out from under us first (operator switched modes, disarmed,
// landed, ...) or the heartbeat goes stale; the caller goes back to
// wait_for_auto_armed() either way.
bool wait_for_target_lock(MavConnection& vehicle, TargetRangeReceiver& receiver, HealthSnapshot& health,
                           double max_heartbeat_gap_sec, double link_stale_ms, double confirm_hold_sec) {
    uint32_t auto_mode = copter_mode_mapping().at("AUTO");
    auto last_valid = std::chrono::steady_clock::now() - std::chrono::seconds(10);
    TargetRangeMsg last{};
    bool locking = false;
    auto lock_since = std::chrono::steady_clock::now();
    auto last_status_print = std::chrono::steady_clock::now() - std::chrono::seconds(10);
    uint32_t custom_mode = auto_mode;

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
        if (vehicle.recv_match({MAVLINK_MSG_ID_HEARTBEAT}, hmsg, 0.1) && hmsg.sysid == vehicle.target_system()) {
            mavlink_heartbeat_t hb;
            mavlink_msg_heartbeat_decode(&hmsg, &hb);
            custom_mode = hb.custom_mode;
            apply_health(hmsg, health, now);
        }

        double heartbeat_gap = std::chrono::duration<double>(now - health.last_heartbeat).count();
        if (heartbeat_gap > max_heartbeat_gap_sec) return false;
        if (!health.armed || custom_mode != auto_mode) return false;

        if (tracking) {
            if (!locking) {
                locking = true;
                lock_since = now;
            }
            if (std::chrono::duration<double>(now - lock_since).count() >= confirm_hold_sec) return true;
        } else {
            locking = false;
        }

        if (std::chrono::duration<double>(now - last_status_print).count() >= 5.0) {
            std::cout << "[HYBRID_GUIDANCE] AUTO 비행 중 - 타겟 대기 (tracking=" << (tracking ? "Y" : "N") << ")"
                      << std::endl;
            last_status_print = now;
        }
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
    if (h.prearm_present && !h.prearm_healthy) return true;
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

        double search_confirm_hold_sec = hg.get_double_or("search_confirm_hold_sec", 1.0);
        double stop_hover_sec = hg.get_double_or("stop_hover_sec", 5.0);
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

        int udp_port = static_cast<int>(track.get_long_or("udp_port", 15020));
        TargetRangeReceiver receiver(udp_port);
        std::cout << "하이브리드 가이던스 시작 (UDP " << udp_port << ", 주기 " << control_cycle_ms
                  << "ms, shell=" << cfg.shell_radius_m << "m, stop=" << cfg.stop_radius_m << "m)"
                  << std::endl;

        // health is declared exactly once, here, and every wait/loop below
        // updates this same instance (never a fresh one per iteration) -
        // see the top-of-file comment on why that matters.
        HealthSnapshot health;
        health.last_heartbeat = std::chrono::steady_clock::now();  // drone::connect() already saw one to get here

        // Sit and watch until the vehicle is armed and flying its own AUTO
        // mission, then wait for a target lock solid enough to take over.
        // If AUTO/armed drops out before a lock happens (operator switched
        // modes, disarmed, ...), go back to watching rather than treating
        // it as fatal - the operator is still flying, just not ready for a
        // handoff yet.
        while (true) {
            wait_for_auto_armed(vehicle, health, land_gap_sec);
            std::cout << "[HYBRID_GUIDANCE] AUTO + armed 확인됨 - 타겟 확정 대기." << std::endl;
            if (wait_for_target_lock(vehicle, receiver, health, land_gap_sec, link_stale_ms,
                                      search_confirm_hold_sec)) {
                break;
            }
            std::cout << "[HYBRID_GUIDANCE] AUTO/armed 상태 이탈 - 대기 상태로 복귀." << std::endl;
        }

        std::cout << "[HYBRID_GUIDANCE] 타겟 확정 - GUIDED로 전환합니다." << std::endl;
        if (!drone::set_mode("GUIDED")) throw std::runtime_error("GUIDED 모드 전환 명령 전송 실패");
        wait_for_mode(vehicle, "GUIDED", 5.0);
        std::cout << "GUIDED 모드 확인됨." << std::endl;

        auto start = std::chrono::steady_clock::now();
        GuidanceState guidance_state;
        bool stopped_timer_running = false;
        auto stopped_since = start;
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
                drone::send_velocity_body(cmd.vx, cmd.vy, cmd.vz, cmd.yaw_rate);
                std::cout << std::fixed << std::setprecision(2) << "t=" << elapsed_sec << "s "
                          << (tracking ? "TRACK" : "LOST ") << " mode=" << mode_name(cmd.mode)
                          << " dist=" << last.distance_m << "m vx=" << cmd.vx << " vy=" << cmd.vy
                          << " vz=" << cmd.vz << " alt=" << alt << "m armed=" << (health.armed ? "Y" : "N")
                          << std::endl;

                // Document/algorithm.md's "1m 하드정지 이후 동작" open item:
                // hover at the stop point for stop_hover_sec, then land. The
                // timer only runs while still in kStopped - if the vehicle
                // drifts back out past stop_radius_m (mode changes), the
                // hover-then-land countdown resets rather than landing from
                // wherever it happened to be.
                if (cmd.mode == GuidanceMode::kStopped) {
                    if (!stopped_timer_running) {
                        stopped_timer_running = true;
                        stopped_since = cycle_start;
                    }
                    double stopped_held = std::chrono::duration<double>(cycle_start - stopped_since).count();
                    if (stopped_held >= stop_hover_sec) {
                        std::cout << "[HYBRID_GUIDANCE] 1m 정지 " << stop_hover_sec << "초 호버링 완료 - 착륙."
                                  << std::endl;
                        drone::land();
                        return 0;
                    }
                } else {
                    stopped_timer_running = false;
                }
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
