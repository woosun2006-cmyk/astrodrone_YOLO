// Print the vehicle GPS location received from a Pixhawk over MAVLink.
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <string>

#include "drone_lib.hpp"
#include "app/runtime_config.hpp"

namespace {

const std::map<int, std::string> kFixNames = {
    {0, "no GPS"}, {1, "no fix"}, {2, "2D fix"},   {3, "3D fix"},  {4, "DGPS"},
    {5, "RTK float"}, {6, "RTK fixed"}, {7, "static"}, {8, "PPP"},
};

std::string fix_name(int fix_type) {
    auto it = kFixNames.find(fix_type);
    if (it != kFixNames.end()) return it->second;
    return "type " + std::to_string(fix_type);
}

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

// Formats MAVLink fields whose all-ones value means unknown.
std::string format_optional(uint16_t value, double scale, const std::string& suffix, int decimals = 1) {
    if (value == 65535) return "unknown";
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(decimals) << (value / scale) << suffix;
    return oss.str();
}

struct Args {
    std::string address;
    int baud;
    double heartbeat_timeout;
    double gps_timeout = 15;
    double rate = 1;
    bool once = false;
};

}  // namespace

int run(int argc, char** argv) {
    YamlValue settings = drone::load_mavlink_settings();
    YamlValue serial = settings["real"]["serial"];

    Args args;
    args.address = serial["address"].as_string();
    args.baud = static_cast<int>(serial["baud"].as_long());
    args.heartbeat_timeout = settings.get_double_or("heartbeat_timeout", 20);
    bool allow_telemetry_configuration = true;

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
        } else if (arg == "--gps-timeout") {
            args.gps_timeout = std::stod(next("--gps-timeout"));
        } else if (arg == "--rate") {
            args.rate = std::stod(next("--rate"));
        } else if (arg == "--once") {
            args.once = true;
        }
    }

    const bool runtime_selected = std::getenv("ASTRODRONE_TARGET") != nullptr ||
                                  std::getenv("DRONE_TARGET") != nullptr;
    if (runtime_selected) {
        const auto target = app::runtime_target_from_environment();
        const auto runtime = app::load_runtime_config(
            target, app::TransportRole::TelemetrySubscriber);
        args.address = runtime.endpoint;
        args.baud = runtime.baud;
        allow_telemetry_configuration = runtime.allow_telemetry_configuration;
    }
    if (!runtime_selected || app::endpoint_is_serial(args.address)) {
        throw std::runtime_error(
            "diagnostic requires ASTRODRONE_TARGET=sitl|real and a runtime loopback UDP endpoint; direct serial is disabled");
    }

    if (!std::isfinite(args.rate) || args.rate <= 0) {
        std::cerr << "--rate must be greater than zero" << std::endl;
        return 2;
    }
    if (args.gps_timeout <= 0) {
        std::cerr << "--gps-timeout must be greater than zero" << std::endl;
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
    if (allow_telemetry_configuration) {
        request_message_interval(*connection, MAVLINK_MSG_ID_GPS_RAW_INT, args.rate);
    }

    while (true) {
        mavlink_message_t msg;
        bool got = connection->recv_match({MAVLINK_MSG_ID_GPS_RAW_INT}, msg, args.gps_timeout);
        if (!got) {
            std::cerr << "Pixhawk is connected, but no GPS_RAW_INT message arrived. Check that "
                         "the GPS is connected to the Pixhawk GPS/I2C port and enabled in the "
                         "flight-controller configuration."
                      << std::endl;
            return 1;
        }

        mavlink_gps_raw_int_t gps;
        mavlink_msg_gps_raw_int_decode(&msg, &gps);

        int fix_type = gps.fix_type;
        std::string fix = fix_name(fix_type);
        int satellites = gps.satellites_visible;

        if (fix_type < 2) {
            std::cout << "Waiting for GPS fix: " << fix << ", satellites=" << satellites << ". "
                         "Move the antenna outdoors with a clear view of the sky."
                      << std::endl;
            continue;
        }

        double latitude = gps.lat / 10'000'000.0;
        double longitude = gps.lon / 10'000'000.0;
        double altitude_msl = gps.alt / 1000.0;
        std::string hdop = format_optional(gps.eph, 100.0, "", 2);

        std::time_t now = std::time(nullptr);
        char timestamp[32];
        std::strftime(timestamp, sizeof(timestamp), "%Y-%m-%d %H:%M:%S", std::localtime(&now));

        std::cout << timestamp << " | lat=" << std::fixed << std::setprecision(7) << latitude
                  << ", lon=" << longitude << ", alt_msl=" << std::setprecision(2) << altitude_msl
                  << " m, fix=" << fix << ", satellites=" << satellites << ", hdop=" << hdop
                  << std::endl;

        if (args.once) break;
    }

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
