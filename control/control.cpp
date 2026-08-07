#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <thread>

#include "drone_lib.hpp"
#include "target_link.hpp"

namespace {

// Stage 2+3 of the target-approach pipeline: read the latest range sample
// target_distance.cpp published (stage 1), turn it into a body-frame
// velocity command (proportional guidance + safety clamps), and send it to
// the Pixhawk. See target_distance.cpp / setting/MAVLink.yaml's
// target_track.cycle_ms for why the loop runs at that rate.
void approach_target(const YamlValue& track, double duration_sec) {
    int udp_port = static_cast<int>(track.get_long_or("udp_port", 15020));
    double cycle_ms = track.get_double_or("cycle_ms", 100);
    double max_forward = track.get_double_or("max_forward_speed", 1.5);
    double max_lateral = track.get_double_or("max_lateral_speed", 1.0);
    double stop_distance = track.get_double_or("stop_distance", 2.0);
    double link_stale_ms = track.get_double_or("link_stale_ms", 500);

    // Proportional gains: forward speed reaches max_forward at
    // (distance - stop_distance) = max_forward / k_forward; lateral speed
    // reaches max_lateral at a 1/k_lateral_px pixel offset from center.
    const double k_forward = 0.5;       // 1/s
    const double k_lateral_px = 0.01;   // (m/s) per pixel of x_px offset

    TargetRangeReceiver receiver(udp_port);
    std::cout << "타겟 접근 모드 시작 (UDP " << udp_port << ", 주기 " << cycle_ms << "ms, 최대 "
              << duration_sec << "초)" << std::endl;

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

        double vx = 0.0, vy = 0.0;  // body frame: +x forward, +y right
        bool tracking = link_age_ms <= link_stale_ms && last.valid && last.found;
        if (tracking) {
            double approach_distance = std::max(0.0, static_cast<double>(last.distance_m) - stop_distance);
            vx = std::min(max_forward, k_forward * approach_distance);
            vy = std::clamp(k_lateral_px * static_cast<double>(last.x_px), -max_lateral, max_lateral);
        }

        drone::send_velocity_body(vx, vy, 0.0, 0.0);

        std::cout << std::fixed << std::setprecision(2) << "t=" << elapsed_sec << "s "
                  << (tracking ? "TRACK" : "LOST ") << " dist=" << last.distance_m
                  << "m vx=" << vx << " vy=" << vy << std::endl;

        auto next_tick = cycle_start + std::chrono::duration<double>(cycle_ms / 1000.0);
        std::this_thread::sleep_until(next_tick);
    }
}

}  // namespace

int main() {
    try {
        YamlValue settings = drone::load_mavlink_settings();
        YamlValue track = settings["target_track"];
        drone::connect(settings["real"]["proxy_udp"]["address"].as_string());
        const double target_alt = 10;

        drone::set_mode("GUIDED");
        std::this_thread::sleep_for(std::chrono::seconds(2));

        drone::arm_disarm(true);
        std::cout << "시동 완료." << std::endl;

        drone::takeoff(target_alt);
        std::this_thread::sleep_for(std::chrono::seconds(10));

        approach_target(track, track.get_double_or("approach_duration_sec", 60));

        drone::land();
        std::cout << "착륙 중..." << std::endl;
        return 0;
    } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return 1;
    }
}
