// Real-time altitude-hold monitor. Reads MAVLink ALTITUDE messages
// (altitude_relative, meters above home - same reference drone::takeoff()
// uses) and reports how tightly the vehicle is holding around a reference
// altitude: current deviation, min/max, stddev, and % of time within
// tolerance. Also checks each reading against setting/safety.yaml's
// altitude_limit (soft_limit_m / hard_limit_m) and flags SOFT/HARD breaches.
// Read-only, sends no arm/motor commands -- emergency.cpp is what actually
// enforces the hard limit by commanding the vehicle back down.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <string>

#include "drone_lib.hpp"

namespace {

// Ask the flight controller to emit a MAVLink message periodically.
void request_message_interval(MavConnection& connection, uint32_t message_id, double frequency_hz) {
    int64_t interval_us = static_cast<int64_t>(1'000'000 / frequency_hz);
    mavlink_message_t msg;
    mavlink_msg_command_long_pack(255, 0, &msg, connection.target_system(),
                                   connection.target_component(), MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                                   static_cast<float>(message_id), static_cast<float>(interval_us),
                                   0, 0, 0, 0, 0);
    connection.send(msg);
}

struct Args {
    std::string address;
    int baud;
    double heartbeat_timeout;
    double alt_timeout = 15;
    double rate = 5;
    double tolerance = 0.5;  // meters; |deviation| <= tolerance counts as "holding"
    double target = NAN;     // NAN = use the first reading as the reference
    double duration = 0;     // seconds; 0 = run until interrupted
    double soft_limit_m = 4.0;  // setting/safety.yaml altitude_limit.soft_limit_m
    double hard_limit_m = 5.0;  // setting/safety.yaml altitude_limit.hard_limit_m
};

enum class LimitStatus { kOk, kSoft, kHard };

LimitStatus classify_altitude(double alt, double soft_limit_m, double hard_limit_m) {
    if (alt >= hard_limit_m) return LimitStatus::kHard;
    if (alt >= soft_limit_m) return LimitStatus::kSoft;
    return LimitStatus::kOk;
}

const char* limit_status_label(LimitStatus status) {
    switch (status) {
        case LimitStatus::kHard: return "HARD-LIMIT";
        case LimitStatus::kSoft: return "SOFT-LIMIT";
        default: return "ok";
    }
}

}  // namespace

int run(int argc, char** argv) {
    YamlValue settings = drone::load_mavlink_settings();
    YamlValue serial = settings["real"]["serial"];

    Args args;
    args.address = serial["address"].as_string();
    args.baud = static_cast<int>(serial["baud"].as_long());
    args.heartbeat_timeout = settings.get_double_or("heartbeat_timeout", 20);

    try {
        YamlValue alt_limit = drone::load_safety_settings()["altitude_limit"];
        args.soft_limit_m = alt_limit.get_double_or("soft_limit_m", args.soft_limit_m);
        args.hard_limit_m = alt_limit.get_double_or("hard_limit_m", args.hard_limit_m);
    } catch (const std::exception& e) {
        std::cerr << "Warning: could not load setting/safety.yaml altitude_limit (" << e.what()
                   << "), using defaults soft=" << args.soft_limit_m << "m hard=" << args.hard_limit_m
                   << "m." << std::endl;
    }

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        auto next = [&](const char* name) -> std::string {
            if (i + 1 >= argc) throw std::runtime_error(std::string(name) + " requires a value");
            return argv[++i];
        };
        if (arg == "--address") {
            args.address = next("--address");
        } else if (arg == "--baud") {
            args.baud = std::stoi(next("--baud"));
        } else if (arg == "--heartbeat-timeout") {
            args.heartbeat_timeout = std::stod(next("--heartbeat-timeout"));
        } else if (arg == "--alt-timeout") {
            args.alt_timeout = std::stod(next("--alt-timeout"));
        } else if (arg == "--rate") {
            args.rate = std::stod(next("--rate"));
        } else if (arg == "--tolerance") {
            args.tolerance = std::stod(next("--tolerance"));
        } else if (arg == "--target") {
            args.target = std::stod(next("--target"));
        } else if (arg == "--duration") {
            args.duration = std::stod(next("--duration"));
        }
    }

    if (!std::isfinite(args.rate) || args.rate <= 0) {
        std::cerr << "--rate must be greater than zero" << std::endl;
        return 2;
    }
    if (!std::isfinite(args.tolerance) || args.tolerance <= 0) {
        std::cerr << "--tolerance must be greater than zero" << std::endl;
        return 2;
    }

    auto connection = open_connection(args.address, args.baud);
    std::cout << "Waiting for Pixhawk heartbeat on " << args.address << " at " << args.baud
              << " baud..." << std::endl;
    if (!connection->wait_heartbeat(args.heartbeat_timeout)) {
        std::cerr << "No Pixhawk heartbeat received. Check the USB/serial connection, device "
                     "name, permissions, and baud rate."
                  << std::endl;
        return 1;
    }
    std::cout << "Connected to MAVLink system " << static_cast<int>(connection->target_system())
              << ", component " << static_cast<int>(connection->target_component()) << "."
              << std::endl;

    request_message_interval(*connection, MAVLINK_MSG_ID_ALTITUDE, args.rate);

    bool have_target = std::isfinite(args.target);
    double target = args.target;

    long sample_count = 0;
    double alt_min = 0, alt_max = 0, sum = 0, sumsq = 0;
    double hold_time = 0, total_time = 0;
    auto start_time = std::chrono::steady_clock::now();
    auto last_time = start_time;
    auto deadline = args.duration > 0 ? start_time + std::chrono::duration<double>(args.duration)
                                       : std::chrono::steady_clock::time_point::max();

    std::cout << "Streaming ALTITUDE at " << args.rate << " Hz, tolerance=+/-" << args.tolerance
              << "m" << (have_target ? "" : " (reference = first reading)") << ". Ctrl+C to stop."
              << std::endl;
    std::cout << "Altitude limit: soft=" << args.soft_limit_m << "m hard=" << args.hard_limit_m
              << "m (setting/safety.yaml altitude_limit)." << std::endl;

    LimitStatus last_limit_status = LimitStatus::kOk;
    long soft_samples = 0, hard_samples = 0;

    while (std::chrono::steady_clock::now() < deadline) {
        mavlink_message_t msg;
        bool got = connection->recv_match({MAVLINK_MSG_ID_ALTITUDE}, msg, args.alt_timeout);
        if (!got) {
            std::cerr << "Pixhawk is connected, but no ALTITUDE message arrived. Check that "
                         "the flight controller's SR params / message intervals allow it through."
                      << std::endl;
            return 1;
        }

        mavlink_altitude_t altitude_msg;
        mavlink_msg_altitude_decode(&msg, &altitude_msg);
        double alt = altitude_msg.altitude_relative;

        auto now = std::chrono::steady_clock::now();

        if (!have_target) {
            target = alt;
            have_target = true;
            std::cout << "Reference altitude set to " << std::fixed << std::setprecision(2)
                      << target << " m (first reading)." << std::endl;
        }

        double deviation = alt - target;
        bool holding = std::fabs(deviation) <= args.tolerance;

        LimitStatus limit_status = classify_altitude(alt, args.soft_limit_m, args.hard_limit_m);
        if (limit_status == LimitStatus::kSoft) ++soft_samples;
        if (limit_status == LimitStatus::kHard) ++hard_samples;
        if (limit_status != last_limit_status) {
            if (limit_status == LimitStatus::kHard) {
                std::cerr << "[BREACH] altitude " << std::fixed << std::setprecision(2) << alt
                          << "m exceeded hard_limit_m " << args.hard_limit_m << "m." << std::endl;
            } else if (limit_status == LimitStatus::kSoft) {
                std::cerr << "[WARN] altitude " << std::fixed << std::setprecision(2) << alt
                          << "m exceeded soft_limit_m " << args.soft_limit_m << "m." << std::endl;
            } else if (last_limit_status != LimitStatus::kOk) {
                std::cerr << "Altitude back under soft_limit_m (" << args.soft_limit_m << "m)."
                          << std::endl;
            }
            last_limit_status = limit_status;
        }

        if (sample_count == 0) {
            alt_min = alt_max = alt;
        } else {
            double dt = std::chrono::duration<double>(now - last_time).count();
            alt_min = std::min(alt_min, alt);
            alt_max = std::max(alt_max, alt);
            total_time += dt;
            if (holding) hold_time += dt;
        }
        sum += alt;
        sumsq += alt * alt;
        ++sample_count;
        last_time = now;

        double mean = sum / sample_count;
        double variance = std::max(0.0, sumsq / sample_count - mean * mean);
        double hold_pct = total_time > 0 ? (hold_time / total_time) * 100.0 : 100.0;
        double elapsed = std::chrono::duration<double>(now - start_time).count();

        std::cout << std::fixed << std::setprecision(2) << "t=" << elapsed << "s alt=" << alt
                  << "m target=" << target << "m dev=" << std::showpos << deviation
                  << std::noshowpos << "m [" << (holding ? "HOLD " : "DRIFT") << "] min=" << alt_min
                  << " max=" << alt_max << " std=" << std::sqrt(variance) << " held=" << hold_pct
                  << "% limit=" << limit_status_label(limit_status) << std::endl;
    }

    std::cout << "---" << std::endl;
    std::cout << std::fixed << std::setprecision(2) << "Samples=" << sample_count
              << ", mean=" << (sample_count > 0 ? sum / sample_count : 0.0) << "m, min=" << alt_min
              << "m, max=" << alt_max << "m, held="
              << (total_time > 0 ? (hold_time / total_time) * 100.0 : 100.0) << "% within +/-"
              << args.tolerance << "m of " << target << "m" << std::endl;
    std::cout << "Altitude limit: soft_samples=" << soft_samples << ", hard_samples=" << hard_samples
              << " (soft_limit_m=" << args.soft_limit_m << ", hard_limit_m=" << args.hard_limit_m
              << ")" << std::endl;

    return 0;
}

int main(int argc, char** argv) {
    try {
        return run(argc, argv);
    } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return 1;
    }
}
