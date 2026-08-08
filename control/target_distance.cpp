// Stage 1 of the target-approach pipeline ("1.거리계산"): computes the
// drone's straight-line distance to a YOLO-detected target and publishes it
// over loopback UDP for control.cpp to consume (stage 2/3: velocity calc +
// send to Pixhawk - see control.cpp).
//
// Combines two legs with the Pythagorean theorem each cycle:
//   - the target's pixel offset from the image center (YOLO_MODEL/
//     yolo_live.py's /target, in the setting/cam_sets.yaml coord_origin
//     frame), scaled to meters by target_track.pixel_to_meter
//   - the vehicle's altitude above home from MAVLink ALTITUDE
//     (altitude_relative), the same reading check_alt.cpp reports
//
// Read-only on the MAVLink side (sends no arm/motor commands) plus an HTTP
// poll of yolo_live.py. Runs its own MAVLink connection, separate from
// control.cpp's - the two are linked only by the UDP socket on
// target_track.udp_port.
#include <arpa/inet.h>
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

// Pulls a numeric field out of yolo_live.py's flat /target JSON (no nested
// objects/arrays, so a substring search is enough - see the endpoint in
// YOLO_MODEL/yolo_live.py). Returns NaN if the key is absent or null.
double json_number(const std::string& body, const std::string& key) {
    std::string needle = "\"" + key + "\":";
    size_t pos = body.find(needle);
    if (pos == std::string::npos) return NAN;
    pos += needle.size();
    size_t end = body.find_first_of(",}", pos);
    std::string token = body.substr(pos, end - pos);
    try {
        return std::stod(token);
    } catch (...) {
        return NAN;
    }
}

bool json_bool(const std::string& body, const std::string& key) {
    std::string needle = "\"" + key + "\":";
    size_t pos = body.find(needle);
    if (pos == std::string::npos) return false;
    return body.compare(pos + needle.size(), 4, "true") == 0;
}

struct Args {
    std::string mav_address;
    int mav_baud;
    double heartbeat_timeout;
    std::string yolo_host = "127.0.0.1";
    int yolo_port = 8002;
    int udp_port = 15020;
    double pixel_to_meter = 0.01;
    double target_stale_ms = 400;
    // Paced to target_track.sense_cycle_ms, not control.cpp's
    // control_cycle_ms - this loop's natural rate is YOLO's actual
    // inference throughput (~10 fps), not the Pixhawk send rate.
    double sense_cycle_ms = 100;
};

}  // namespace

int run(int argc, char** argv) {
    YamlValue settings = drone::load_mavlink_settings();
    YamlValue track = settings["target_track"];

    Args args;
    // Default to the same proxy_udp endpoint control.cpp connects through
    // (not real.serial): the two programs run concurrently, and only the
    // UDP proxy is a shared multi-listener endpoint - two processes opening
    // the raw serial port at once would interleave/corrupt each other's
    // parsed MAVLink stream.
    args.mav_address = settings["real"]["proxy_udp"]["address"].as_string();
    args.mav_baud = 115200;
    args.heartbeat_timeout = settings.get_double_or("heartbeat_timeout", 20);
    args.yolo_host = track["yolo_host"].as_string();
    args.yolo_port = static_cast<int>(track.get_long_or("yolo_port", 8002));
    args.udp_port = static_cast<int>(track.get_long_or("udp_port", 15020));
    args.pixel_to_meter = track.get_double_or("pixel_to_meter", 0.01);
    args.target_stale_ms = track.get_double_or("target_stale_ms", 400);
    args.sense_cycle_ms = track.get_double_or("sense_cycle_ms", 100);

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
        } else if (arg == "--pixel-to-meter") {
            args.pixel_to_meter = std::stod(next("--pixel-to-meter"));
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
    bool have_altitude = false;
    uint32_t seq = 0;

    // recv_match's timeout paces this loop (same pattern as check_alt.cpp):
    // ALTITUDE is requested at exactly this cycle's rate above, so blocking
    // here for up to one cycle is how we wait for "the next tick" instead of
    // a separate sleep. If the flight controller hiccups for a tick, the
    // last known altitude carries over below rather than the cycle stalling.
    double cycle_sec = args.sense_cycle_ms / 1000.0;

    while (true) {
        mavlink_message_t msg;
        if (connection->recv_match({MAVLINK_MSG_ID_ALTITUDE}, msg, cycle_sec)) {
            mavlink_altitude_t alt;
            mavlink_msg_altitude_decode(&msg, &alt);
            altitude_m = alt.altitude_relative;
            have_altitude = true;
        }

        TargetRangeMsg out{};
        out.seq = seq++;
        out.altitude_m = static_cast<float>(altitude_m);

        bool have_target = false;
        try {
            std::string body = http_get(args.yolo_host, args.yolo_port, "/target", 0.2);
            bool found = json_bool(body, "found");
            // "found" is recency-only (this single frame saw something);
            // "confirmed" is yolo_live.py's TARGET_CONFIRM_FRAMES-in-a-row
            // stability check. Require both so a one-frame misdetection
            // can't turn into a velocity command.
            bool confirmed = json_bool(body, "confirmed");
            double age_ms = json_number(body, "age_ms");
            if (found && confirmed && !std::isnan(age_ms) && age_ms <= args.target_stale_ms) {
                out.x_px = static_cast<float>(json_number(body, "x_px"));
                out.y_px = static_cast<float>(json_number(body, "y_px"));
                have_target = std::isfinite(out.x_px) && std::isfinite(out.y_px);
            }
        } catch (const std::exception& e) {
            std::cerr << "YOLO /target poll failed: " << e.what() << std::endl;
        }

        out.found = have_target ? 1 : 0;
        out.valid = (have_altitude && have_target) ? 1 : 0;

        if (have_target) {
            double ground_offset = std::hypot(out.x_px, out.y_px) * args.pixel_to_meter;
            out.ground_offset_m = static_cast<float>(ground_offset);
            out.distance_m = static_cast<float>(std::hypot(ground_offset, altitude_m));
        }

        sender.send(out);

        std::cout << std::fixed << std::setprecision(2) << "alt=" << out.altitude_m
                  << "m target=" << (out.found ? "yes" : "no ") << " x_px=" << out.x_px
                  << " y_px=" << out.y_px << " ground=" << out.ground_offset_m
                  << "m dist=" << out.distance_m << "m valid=" << static_cast<int>(out.valid)
                  << std::endl;
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
