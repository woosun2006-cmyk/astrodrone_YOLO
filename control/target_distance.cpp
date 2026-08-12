// Stage 1 of the target-approach pipeline ("1.거리계산"): computes the
// drone's straight-line distance to a YOLO-detected target and publishes it
// over loopback UDP for control.cpp to consume (stage 2/3: velocity calc +
// send to Pixhawk - see control.cpp).
//
// Combines two legs with the Pythagorean theorem each cycle:
//   - the target's pixel offset from the image center (YOLO_MODEL/
//     yolo_live.py's /target, in the setting/cam_sets.yaml coord_origin
//     frame), scaled to meters by pos_calculator.cpp's pinhole-camera model
//   - the vehicle's altitude above home from MAVLink ALTITUDE
//     (altitude_relative), the same reading check_alt.cpp reports
//
// Read-only on the MAVLink side (sends no arm/motor commands) plus an HTTP
// poll of yolo_live.py. Runs its own MAVLink connection, separate from
// control.cpp's - the two are linked only by the UDP socket on
// target_track.udp_port.
#include <arpa/inet.h>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <netinet/in.h>
#include <stdexcept>
#include <string>
#include <sys/socket.h>
#include <unistd.h>

#include "drone_lib.hpp"
#include "pos_calculator.hpp"
#include "target_json.hpp"
#include "target_link.hpp"

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

// Minimal blocking HTTP/1.0 GET over loopback TCP - just enough to read
// yolo_live.py's small flat JSON /target response. No keep-alive, no
// chunked transfer: yolo_live.py's Handler sends Content-Length and closes
// the connection.
std::string http_get(const std::string& host, int port, const std::string& path,
                      double timeout_sec) {
    int fd = ::socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) throw std::runtime_error("create HTTP socket failed");

    timeval tv{};
    tv.tv_sec = static_cast<long>(timeout_sec);
    tv.tv_usec = static_cast<long>((timeout_sec - static_cast<double>(tv.tv_sec)) * 1'000'000);
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<uint16_t>(port));
    if (inet_pton(AF_INET, host.c_str(), &addr.sin_addr) != 1) {
        ::close(fd);
        throw std::runtime_error("invalid YOLO host: " + host);
    }

    if (::connect(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
        ::close(fd);
        throw std::runtime_error("connect to " + host + ":" + std::to_string(port) + " failed");
    }

    std::string request =
        "GET " + path + " HTTP/1.0\r\nHost: " + host + "\r\nConnection: close\r\n\r\n";
    if (::send(fd, request.data(), request.size(), 0) < 0) {
        ::close(fd);
        throw std::runtime_error("send HTTP request failed");
    }

    std::string response;
    char buf[2048];
    ssize_t n;
    while ((n = ::recv(fd, buf, sizeof(buf), 0)) > 0) {
        response.append(buf, static_cast<size_t>(n));
    }
    ::close(fd);

    size_t body_start = response.find("\r\n\r\n");
    if (body_start == std::string::npos) throw std::runtime_error("malformed HTTP response");
    return response.substr(body_start + 4);
}

struct Args {
    std::string mav_address;
    int mav_baud;
    double heartbeat_timeout;
    std::string yolo_host = "127.0.0.1";
    int yolo_port = 8002;
    int udp_port = 15020;
    double focal_length_px = 530;
    double target_stale_ms = 400;
    double altitude_stale_ms = 500;
    // Sanity bounds on the computed distance_m, both placeholders to tune
    // against this vehicle's real operating range - a bad pixel detection
    // or a corrupted altitude reading can otherwise turn into a huge
    // straight-line distance that flows straight into control.cpp's
    // forward-speed calc.
    double max_plausible_distance_m = 50.0;   // absolute cap on any single reading
    double max_distance_jump_m = 5.0;         // cap on change from the last accepted reading
    // Paced to setting/rate.yaml's sense_cycle_ms, not control.cpp's
    // control_cycle_ms (MAVLink.yaml) - this loop's natural rate is how
    // often YOLO_MODEL/yolo_live.py is expected to have a fresh detection
    // on hand, not the Pixhawk send rate.
    double sense_cycle_ms = 100;
};

}  // namespace

int run(int argc, char** argv) {
    YamlValue settings = drone::load_mavlink_settings();
    YamlValue track = settings["target_track"];

    Args args;
    // Default to the proxy_udp endpoint, but on the "sensor" port rather
    // than control.cpp's "control" port (not real.serial): the two programs
    // run concurrently, and mav_transport.cpp's UdpTransport binds its port,
    // so two processes sharing one port would steal each other's packets
    // instead of both receiving the stream - see setting/port.yaml.
    YamlValue ports = drone::load_port_settings();
    args.mav_address = with_port(settings["real"]["proxy_udp"]["address"].as_string(),
                                  ports.get_long_or("mavlink_sensor", 14551));
    args.mav_baud = 115200;
    args.heartbeat_timeout = settings.get_double_or("heartbeat_timeout", 20);
    args.yolo_host = track["yolo_host"].as_string();
    args.yolo_port = static_cast<int>(track.get_long_or("yolo_port", 8002));
    args.udp_port = static_cast<int>(track.get_long_or("udp_port", 15020));
    args.focal_length_px = track.get_double_or("pixel_focal_length_px", 530);
    args.target_stale_ms = track.get_double_or("target_stale_ms", 400);
    args.altitude_stale_ms = track.get_double_or("altitude_stale_ms", 500);
    args.max_plausible_distance_m = track.get_double_or("max_plausible_distance_m", 50.0);
    args.max_distance_jump_m = track.get_double_or("max_distance_jump_m", 5.0);
    // Shared with YOLO_MODEL/yolo_live.py's infer_max_fps - see
    // setting/rate.yaml for why the two live together.
    args.sense_cycle_ms = drone::load_rate_settings().get_double_or("sense_cycle_ms", 100);

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        auto next = [&](const char* name) -> std::string {
            if (i + 1 >= argc) throw std::runtime_error(std::string(name) + " requires a value");
            return argv[++i];
        };
        if (arg == "--address") {
            args.mav_address = next("--address");
        } else if (arg == "--baud") {
            args.mav_baud = std::stoi(next("--baud"));
        } else if (arg == "--heartbeat-timeout") {
            args.heartbeat_timeout = std::stod(next("--heartbeat-timeout"));
        } else if (arg == "--yolo-host") {
            args.yolo_host = next("--yolo-host");
        } else if (arg == "--yolo-port") {
            args.yolo_port = std::stoi(next("--yolo-port"));
        } else if (arg == "--udp-port") {
            args.udp_port = std::stoi(next("--udp-port"));
        } else if (arg == "--focal-length-px") {
            args.focal_length_px = std::stod(next("--focal-length-px"));
        } else if (arg == "--altitude-stale-ms") {
            args.altitude_stale_ms = std::stod(next("--altitude-stale-ms"));
        } else if (arg == "--max-plausible-distance-m") {
            args.max_plausible_distance_m = std::stod(next("--max-plausible-distance-m"));
        } else if (arg == "--max-distance-jump-m") {
            args.max_distance_jump_m = std::stod(next("--max-distance-jump-m"));
        } else if (arg == "--sense-cycle-ms") {
            args.sense_cycle_ms = std::stod(next("--sense-cycle-ms"));
        }
    }

    if (args.sense_cycle_ms <= 0) {
        std::cerr << "--sense-cycle-ms must be greater than zero" << std::endl;
        return 2;
    }

    auto connection = open_connection(args.mav_address, args.mav_baud);
    std::cout << "Waiting for Pixhawk heartbeat on " << args.mav_address << "..." << std::endl;
    if (!connection->wait_heartbeat(args.heartbeat_timeout)) {
        std::cerr << "No Pixhawk heartbeat received." << std::endl;
        return 1;
    }
    std::cout << "Connected. Polling YOLO at http://" << args.yolo_host << ":" << args.yolo_port
              << "/target, publishing range on UDP 127.0.0.1:" << args.udp_port << " every "
              << args.sense_cycle_ms << " ms." << std::endl;

    request_message_interval(*connection, MAVLINK_MSG_ID_ALTITUDE, 1000.0 / args.sense_cycle_ms);

    TargetRangeSender sender(args.udp_port);
    double altitude_m = 0.0;
    bool have_altitude = false;  // an ALTITUDE reading has arrived at least once
    bool altitude_was_fresh = false;
    auto last_altitude_time = std::chrono::steady_clock::now();
    uint32_t seq = 0;

    // Outlier tracking for distance_m - see the rejection check below.
    bool have_last_distance = false;
    double last_accepted_distance_m = 0.0;

    // recv_match's timeout paces this loop (same pattern as check_alt.cpp):
    // ALTITUDE is requested at exactly this cycle's rate above, so blocking
    // here for up to one cycle is how we wait for "the next tick" instead of
    // a separate sleep. If the flight controller hiccups for a tick, the
    // last known altitude carries over below rather than the cycle stalling.
    double cycle_sec = args.sense_cycle_ms / 1000.0;

    while (true) {
        mavlink_message_t msg;
        // GLOBAL_POSITION_INT is accepted alongside ALTITUDE because ArduPilot
        // denies the SET_MESSAGE_INTERVAL request for ALTITUDE (msgid 141) and
        // never streams it, while relative_alt carries the same altitude above
        // home and is always available. ALTITUDE still wins when a vehicle does
        // send it.
        if (connection->recv_match({MAVLINK_MSG_ID_ALTITUDE,
                                     MAVLINK_MSG_ID_GLOBAL_POSITION_INT},
                                    msg, cycle_sec)) {
            if (msg.msgid == MAVLINK_MSG_ID_ALTITUDE) {
                mavlink_altitude_t alt;
                mavlink_msg_altitude_decode(&msg, &alt);
                altitude_m = alt.altitude_relative;
            } else {
                mavlink_global_position_int_t gpos;
                mavlink_msg_global_position_int_decode(&msg, &gpos);
                altitude_m = gpos.relative_alt / 1000.0;  // mm -> m
            }
            have_altitude = true;
            last_altitude_time = std::chrono::steady_clock::now();
        }

        // altitude_m above keeps the last known reading purely so the
        // console line and CSV-adjacent logging below stay informative -
        // out.altitude_valid is what tells control.cpp whether that number
        // is still trustworthy. Past altitude_stale_ms with no new reading,
        // it isn't: control.cpp must not act on a frozen value as if it
        // were current (e.g. altitude-limit enforcement going blind while
        // still reporting the last altitude it saw).
        double altitude_age_ms = std::chrono::duration<double, std::milli>(
                                      std::chrono::steady_clock::now() - last_altitude_time)
                                      .count();
        bool altitude_fresh = have_altitude && altitude_age_ms <= args.altitude_stale_ms;
        if (altitude_was_fresh && !altitude_fresh) {
            std::cerr << "[EMERGENCY] ALTITUDE stale: no reading for " << std::fixed
                      << std::setprecision(0) << altitude_age_ms << "ms (> " << args.altitude_stale_ms
                      << "ms) - reporting altitude as invalid until it recovers." << std::endl;
        } else if (!altitude_was_fresh && altitude_fresh) {
            std::cout << "ALTITUDE recovered." << std::endl;
        }
        altitude_was_fresh = altitude_fresh;

        TargetRangeMsg out{};
        out.seq = seq++;
        out.altitude_m = static_cast<float>(altitude_m);
        out.altitude_valid = altitude_fresh ? 1 : 0;

        bool have_target = false;
        try {
            std::string body = http_get(args.yolo_host, args.yolo_port, "/target", 0.2);
            bool found = target_json::json_bool(body, "found").value_or(false);
            // "found" is recency-only (this single frame saw something);
            // "confirmed" is yolo_live.py's TARGET_CONFIRM_FRAMES-in-a-row
            // stability check. Require both so a one-frame misdetection
            // can't turn into a velocity command.
            bool confirmed = target_json::json_bool(body, "confirmed").value_or(false);
            double age_ms = target_json::json_number(body, "age_ms").value_or(NAN);
            if (found && confirmed && !std::isnan(age_ms) && age_ms <= args.target_stale_ms) {
                out.x_px = static_cast<float>(target_json::json_number(body, "x_px").value_or(NAN));
                out.y_px = static_cast<float>(target_json::json_number(body, "y_px").value_or(NAN));
                have_target = std::isfinite(out.x_px) && std::isfinite(out.y_px);
            }
        } catch (const std::exception& e) {
            std::cerr << "YOLO /target poll failed: " << e.what() << std::endl;
        }

        double ground_offset = 0.0, distance = 0.0;
        if (have_target) {
            double pixel_offset = std::hypot(out.x_px, out.y_px);
            ground_offset = pixel_offset_to_ground_m(pixel_offset, altitude_m, args.focal_length_px);
            distance = std::hypot(ground_offset, altitude_m);

            // Outlier rejection: a bad pixel detection or a corrupted
            // altitude reading can produce a distance that's either way
            // outside any plausible operating range, or a sudden jump from
            // the last reading we trusted. Either way, don't let it reach
            // control.cpp's velocity calc - treat this cycle the same as
            // "no target found" instead of clamping and sending it anyway.
            bool plausible = distance <= args.max_plausible_distance_m;
            bool jump_ok = !have_last_distance ||
                           std::fabs(distance - last_accepted_distance_m) <= args.max_distance_jump_m;
            if (!plausible || !jump_ok) {
                std::cerr << "[EMERGENCY] distance outlier rejected: " << std::fixed
                          << std::setprecision(2) << distance << "m ("
                          << (!plausible ? "exceeds max_plausible_distance_m="
                                               + std::to_string(args.max_plausible_distance_m) + "m"
                                         : "jumped " +
                                               std::to_string(std::fabs(distance - last_accepted_distance_m)) +
                                               "m from last accepted " +
                                               std::to_string(last_accepted_distance_m) + "m")
                          << ") - treating this cycle as target-not-found." << std::endl;
                have_target = false;
            } else {
                last_accepted_distance_m = distance;
                have_last_distance = true;
            }
        }

        out.found = have_target ? 1 : 0;
        out.valid = (altitude_fresh && have_target) ? 1 : 0;
        if (have_target) {
            out.ground_offset_m = static_cast<float>(ground_offset);
            out.distance_m = static_cast<float>(distance);
        }

        sender.send(out);

        std::cout << std::fixed << std::setprecision(2) << "alt=" << out.altitude_m
                  << "m (valid=" << static_cast<int>(out.altitude_valid) << ") target="
                  << (out.found ? "yes" : "no ") << " x_px=" << out.x_px << " y_px=" << out.y_px
                  << " ground=" << out.ground_offset_m << "m dist=" << out.distance_m
                  << "m valid=" << static_cast<int>(out.valid) << std::endl;
    }
}

int main(int argc, char** argv) {
    try {
        return run(argc, argv);
    } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return 1;
    }
}
