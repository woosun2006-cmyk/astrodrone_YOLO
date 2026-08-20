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
// poll of yolo_live.py. Its endpoint is selected as a telemetry subscriber by
// runtime_transport.cpp. On real hardware this must be the shared UDP output
// of the single serial owner, never the Pixhawk serial device itself.
#include <arpa/inet.h>
#include <chrono>
#include <cmath>
#include <cstdio>
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
#include "autopilot/runtime_transport.hpp"
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
    double focal_x_px = 530;
    double focal_y_px = 530;
    double principal_x_px = 320;
    double principal_y_px = 240;
    double frame_width_px = 640;
    double frame_height_px = 480;
    double target_stale_ms = 400;
    double altitude_stale_ms = 500;
    bool fixed_target = false;
    double fixed_target_north_m = 0.0;
    double fixed_target_east_m = 0.0;
    double fixed_target_min_alt_m = 0.0;
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
    const autopilot::RuntimeTarget runtime_target =
        autopilot::runtime_target_from_environment();
    const autopilot::RuntimeTransportConfig transport =
        autopilot::load_runtime_transport(runtime_target,
                                           autopilot::TransportRole::TelemetrySubscriber);
    args.mav_address = transport.endpoint;
    args.mav_baud = transport.baud;
    args.heartbeat_timeout = settings.get_double_or("heartbeat_timeout", 20);
    args.yolo_host = track["yolo_host"].as_string();
    args.yolo_port = static_cast<int>(track.get_long_or("yolo_port", 8002));
    args.udp_port = static_cast<int>(track.get_long_or("udp_port", 15020));
    args.focal_length_px = track.get_double_or("pixel_focal_length_px", 530);
    args.focal_x_px = track.get_double_or("pixel_focal_length_x_px", args.focal_length_px);
    args.focal_y_px = track.get_double_or("pixel_focal_length_y_px", args.focal_length_px);
    args.principal_x_px = track.get_double_or("pixel_principal_point_x_px", 320);
    args.principal_y_px = track.get_double_or("pixel_principal_point_y_px", 240);
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
            args.focal_x_px = args.focal_length_px;
            args.focal_y_px = args.focal_length_px;
        } else if (arg == "--focal-x-px") {
            args.focal_x_px = std::stod(next("--focal-x-px"));
        } else if (arg == "--focal-y-px") {
            args.focal_y_px = std::stod(next("--focal-y-px"));
        } else if (arg == "--principal-x-px") {
            args.principal_x_px = std::stod(next("--principal-x-px"));
        } else if (arg == "--principal-y-px") {
            args.principal_y_px = std::stod(next("--principal-y-px"));
        } else if (arg == "--altitude-stale-ms") {
            args.altitude_stale_ms = std::stod(next("--altitude-stale-ms"));
        } else if (arg == "--max-plausible-distance-m") {
            args.max_plausible_distance_m = std::stod(next("--max-plausible-distance-m"));
        } else if (arg == "--max-distance-jump-m") {
            args.max_distance_jump_m = std::stod(next("--max-distance-jump-m"));
        } else if (arg == "--sense-cycle-ms") {
            args.sense_cycle_ms = std::stod(next("--sense-cycle-ms"));
        } else if (arg == "--fixed-target-ned") {
            args.fixed_target = true;
            args.fixed_target_north_m = std::stod(next("--fixed-target-ned NORTH"));
            args.fixed_target_east_m = std::stod(next("--fixed-target-ned EAST"));
        } else if (arg == "--fixed-target-min-alt") {
            args.fixed_target_min_alt_m = std::stod(next("--fixed-target-min-alt"));
        }
    }

    if (runtime_target == autopilot::RuntimeTarget::Real &&
        autopilot::endpoint_is_serial(args.mav_address)) {
        throw std::runtime_error(
            "real target-distance is telemetry-only and cannot open a Pixhawk serial endpoint");
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
    if (args.fixed_target) {
        std::cout << "Connected. Fixed target test mode: LOCAL_NED north="
                  << args.fixed_target_north_m << "m east=" << args.fixed_target_east_m
                  << "m, active at altitude >= " << args.fixed_target_min_alt_m
                  << "m. YOLO is bypassed." << std::endl;
    } else {
        std::cout << "Connected. Polling actual YOLO /target at http://" << args.yolo_host << ":"
                  << args.yolo_port << "/target." << std::endl;
    }
    std::cout << "Publishing range on UDP 127.0.0.1:" << args.udp_port << " every "
              << args.sense_cycle_ms << " ms." << std::endl;

    if (transport.allow_telemetry_configuration) {
        request_message_interval(*connection, MAVLINK_MSG_ID_ALTITUDE,
                                  1000.0 / args.sense_cycle_ms);
        // ArduCopter SITL does not always emit the common-dialect ALTITUDE
        // message. GLOBAL_POSITION_INT.relative_alt is the reliable fallback.
        request_message_interval(*connection, MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
                                 1000.0 / args.sense_cycle_ms);
        if (args.fixed_target) {
            request_message_interval(*connection, MAVLINK_MSG_ID_LOCAL_POSITION_NED,
                                     1000.0 / args.sense_cycle_ms);
            request_message_interval(*connection, MAVLINK_MSG_ID_ATTITUDE,
                                     1000.0 / args.sense_cycle_ms);
        }
    } else {
        std::cout << "Telemetry configuration writes disabled by runtime command mode."
                  << std::endl;
    }

    TargetRangeSender sender(args.udp_port);
    double altitude_m = 0.0;
    bool have_altitude = false;  // an ALTITUDE reading has arrived at least once
    bool altitude_was_fresh = false;
    auto last_altitude_time = std::chrono::steady_clock::now();
    double local_north_m = 0.0;
    double local_east_m = 0.0;
    double yaw_rad = 0.0;
    bool have_local_position = false;
    bool have_attitude = false;
    bool fixed_target_activated = false;
    auto last_local_position_time = std::chrono::steady_clock::now();
    auto last_attitude_time = std::chrono::steady_clock::now();
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
        auto cycle_deadline = std::chrono::steady_clock::now() +
                              std::chrono::duration<double>(cycle_sec);
        do {
            double remaining_sec = std::chrono::duration<double>(
                                       cycle_deadline - std::chrono::steady_clock::now())
                                       .count();
            if (remaining_sec <= 0.0) break;

            mavlink_message_t msg;
            std::vector<uint32_t> ids{
                MAVLINK_MSG_ID_ALTITUDE,
                MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
            };
            if (args.fixed_target) {
                ids.push_back(MAVLINK_MSG_ID_LOCAL_POSITION_NED);
                ids.push_back(MAVLINK_MSG_ID_ATTITUDE);
            }
            if (!connection->recv_match(ids, msg, remaining_sec)) break;

            auto now = std::chrono::steady_clock::now();
            if (msg.msgid == MAVLINK_MSG_ID_ALTITUDE) {
                mavlink_altitude_t alt;
                mavlink_msg_altitude_decode(&msg, &alt);
                altitude_m = alt.altitude_relative;
                have_altitude = true;
                last_altitude_time = now;
            } else if (msg.msgid == MAVLINK_MSG_ID_GLOBAL_POSITION_INT) {
                mavlink_global_position_int_t pos;
                mavlink_msg_global_position_int_decode(&msg, &pos);
                altitude_m = static_cast<double>(pos.relative_alt) / 1000.0;
                have_altitude = true;
                last_altitude_time = now;
            } else if (msg.msgid == MAVLINK_MSG_ID_LOCAL_POSITION_NED) {
                mavlink_local_position_ned_t pos;
                mavlink_msg_local_position_ned_decode(&msg, &pos);
                local_north_m = pos.x;
                local_east_m = pos.y;
                have_local_position = true;
                last_local_position_time = now;
                // In fixed-target SITL mode LOCAL_POSITION_NED.z is metres
                // down from the home origin, so -z is relative altitude.
                // Use it as another fallback on links that omit ALTITUDE.
                if (args.fixed_target) {
                    altitude_m = std::max(0.0, -static_cast<double>(pos.z));
                    have_altitude = true;
                    last_altitude_time = now;
                }
            } else if (msg.msgid == MAVLINK_MSG_ID_ATTITUDE) {
                mavlink_attitude_t attitude;
                mavlink_msg_attitude_decode(&msg, &attitude);
                yaw_rad = attitude.yaw;
                have_attitude = true;
                last_attitude_time = now;
            }
        } while (std::chrono::steady_clock::now() < cycle_deadline);

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
        bool confirmed_observation = false;
        if (args.fixed_target) {
            if (!fixed_target_activated && altitude_fresh &&
                altitude_m >= args.fixed_target_min_alt_m) {
                fixed_target_activated = true;
                std::cout << "Fixed target activated at altitude " << std::fixed
                          << std::setprecision(2) << altitude_m
                          << "m; it will remain active during descent." << std::endl;
            }
            auto now = std::chrono::steady_clock::now();
            double position_age_ms = std::chrono::duration<double, std::milli>(
                                         now - last_local_position_time)
                                         .count();
            double attitude_age_ms = std::chrono::duration<double, std::milli>(
                                         now - last_attitude_time)
                                         .count();
            bool pose_fresh = have_local_position && have_attitude &&
                              position_age_ms <= args.altitude_stale_ms &&
                              attitude_age_ms <= args.altitude_stale_ms;
            if (fixed_target_activated && pose_fresh && altitude_fresh) {
                DownwardTargetPixels pixels = fixed_ned_target_to_pixels(
                    args.fixed_target_north_m, args.fixed_target_east_m,
                    local_north_m, local_east_m, yaw_rad, altitude_m,
                    args.focal_x_px, args.focal_y_px);
                out.x_px = static_cast<float>(pixels.x_px);
                out.y_px = static_cast<float>(pixels.y_px);
                have_target = true;
            }
        } else try {
            std::string body = http_get(args.yolo_host, args.yolo_port, "/target", 0.2);
            bool found = target_json::json_bool(body, "found").value_or(false);
            // "found" is recency-only (this single frame saw something);
            // "confirmed" is yolo_live.py's TARGET_CONFIRM_FRAMES-in-a-row
            // stability check. Require both so a one-frame misdetection
            // can't turn into a velocity command.
            bool confirmed = target_json::json_bool(body, "confirmed").value_or(false);
            confirmed_observation = confirmed;
            double age_ms = target_json::json_number(body, "age_ms").value_or(NAN);
            std::string source = target_json::json_string(body, "source").value_or("");
            std::string target_name = target_json::json_string(body, "name").value_or("");
            double target_conf = target_json::json_number(body, "conf").value_or(NAN);
            double detection_count = target_json::json_number(body, "detections").value_or(0.0);
            double frame_sequence = target_json::json_number(body, "frame_sequence").value_or(-1.0);
            double frame_timestamp_ns = target_json::json_number(body, "frame_timestamp_ns").value_or(-1.0);
            double bbox_x_px = target_json::json_number(body, "bbox_x_px").value_or(0.0);
            double bbox_y_px = target_json::json_number(body, "bbox_y_px").value_or(0.0);
            double bbox_width_px = target_json::json_number(body, "bbox_width_px").value_or(0.0);
            double bbox_height_px = target_json::json_number(body, "bbox_height_px").value_or(0.0);
            args.frame_width_px = target_json::json_number(body, "width").value_or(args.frame_width_px);
            args.frame_height_px = target_json::json_number(body, "height").value_or(args.frame_height_px);
            out.observation_age_ms = std::isfinite(age_ms) ? static_cast<float>(age_ms) : -1.0F;
            out.target_confidence = std::isfinite(target_conf) ? static_cast<float>(target_conf) : -1.0F;
            out.frame_sequence = frame_sequence >= 0 ? static_cast<uint64_t>(frame_sequence) : 0;
            out.frame_timestamp_ns = frame_timestamp_ns >= 0 ? static_cast<int64_t>(frame_timestamp_ns) : -1;
            out.frame_width = static_cast<uint32_t>(std::max(1.0, args.frame_width_px));
            out.frame_height = static_cast<uint32_t>(std::max(1.0, args.frame_height_px));
            out.bbox_x_px = static_cast<float>(bbox_x_px);
            out.bbox_y_px = static_cast<float>(bbox_y_px);
            out.bbox_width_px = static_cast<float>(bbox_width_px);
            out.bbox_height_px = static_cast<float>(bbox_height_px);
            std::snprintf(out.class_name, sizeof(out.class_name), "%s", target_name.c_str());
            std::snprintf(out.source, sizeof(out.source), "%s", source.c_str());
            if (!source.empty()) {
                std::cout << "[target-observation] source=" << source
                          << " frame_sequence=" << std::fixed << std::setprecision(0) << frame_sequence
                          << " frame_timestamp_ns=" << frame_timestamp_ns
                          << " age_ms=" << std::setprecision(1) << age_ms
                          << " name=" << (target_name.empty() ? "-" : target_name)
                          << " conf=" << std::setprecision(3) << target_conf
                          << " detections=" << std::setprecision(0) << detection_count
                          << " found=" << (found ? 1 : 0)
                          << " confirmed=" << (confirmed ? 1 : 0) << std::endl;
            }
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
            const PixelCalibration calibration{
                args.focal_x_px, args.focal_y_px, args.principal_x_px, args.principal_y_px};
            const auto angles = downward_pixel_to_angles(out.x_px, out.y_px, calibration);
            const auto body_offset = downward_pixel_to_body_offset(
                out.x_px, out.y_px, altitude_m, calibration);
            out.theta_forward_rad = static_cast<float>(angles.forward_rad);
            out.theta_right_rad = static_cast<float>(angles.right_rad);
            ground_offset = std::hypot(body_offset.forward_m, body_offset.right_m);
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
                out.normalized_x = static_cast<float>(out.x_px / (args.frame_width_px * 0.5));
                out.normalized_y = static_cast<float>(out.y_px / (args.frame_height_px * 0.5));
                out.body_forward_m = static_cast<float>(body_offset.forward_m);
                out.body_right_m = static_cast<float>(body_offset.right_m);
            }
        }

        out.found = have_target ? 1 : 0;
        out.confirmed = confirmed_observation ? 1 : 0;
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
