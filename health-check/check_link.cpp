// Read-only MAVLink link/status check. Sends no arm or motor commands.
#include <chrono>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <string>

#include "drone_lib.hpp"

namespace {

std::string mode_string(uint32_t custom_mode) {
    for (const auto& entry : copter_mode_mapping()) {
        if (entry.second == custom_mode) return entry.first;
    }
    return "Mode(" + std::to_string(custom_mode) + ")";
}

std::string statustext_to_string(const mavlink_statustext_t& st) {
    return std::string(st.text, strnlen(st.text, sizeof(st.text)));
}

}  // namespace

int run(int argc, char** argv) {
    YamlValue settings = drone::load_mavlink_settings();
    YamlValue real_serial = settings["real"]["serial"];

    std::string address = real_serial["address"].as_string();
    int baud = static_cast<int>(real_serial["baud"].as_long());
    double heartbeat_timeout = 20;
    double listen_sec = 8;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        auto next = [&](const char* name) -> std::string {
            if (i + 1 >= argc) throw std::runtime_error(std::string(name) + " requires a value");
            return argv[++i];
        };
        if (arg == "--address") {
            address = next("--address");
        } else if (arg == "--baud") {
            baud = std::stoi(next("--baud"));
        } else if (arg == "--heartbeat-timeout") {
            heartbeat_timeout = std::stod(next("--heartbeat-timeout"));
        } else if (arg == "--listen") {
            listen_sec = std::stod(next("--listen"));
        }
    }

    auto master = open_connection(address, baud);
    std::cout << "MAVLink heartbeat waiting: " << address << ", baud=" << baud
              << ", timeout=" << heartbeat_timeout << "s" << std::endl;

    mavlink_heartbeat_t heartbeat{};
    if (!master->wait_heartbeat(heartbeat_timeout, &heartbeat)) {
        throw std::runtime_error("No heartbeat received. Check cable, port, baud rate, and power.");
    }

    std::cout << "Connected: system=" << static_cast<int>(master->target_system())
              << ", component=" << static_cast<int>(master->target_component())
              << ", mode=" << mode_string(heartbeat.custom_mode)
              << ", armed=" << (is_armed_from_heartbeat(heartbeat) ? "true" : "false")
              << std::endl;

    auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(listen_sec);
    while (std::chrono::steady_clock::now() < deadline) {
        mavlink_message_t msg;
        bool got = master->recv_match(
            {MAVLINK_MSG_ID_HEARTBEAT, MAVLINK_MSG_ID_SYS_STATUS, MAVLINK_MSG_ID_GPS_RAW_INT,
             MAVLINK_MSG_ID_STATUSTEXT},
            msg, 1.0);
        if (!got) continue;

        switch (msg.msgid) {
            case MAVLINK_MSG_ID_HEARTBEAT: {
                mavlink_heartbeat_t hb;
                mavlink_msg_heartbeat_decode(&msg, &hb);
                std::cout << "HEARTBEAT: mode=" << mode_string(hb.custom_mode)
                          << ", armed=" << (is_armed_from_heartbeat(hb) ? "true" : "false")
                          << std::endl;
                break;
            }
            case MAVLINK_MSG_ID_SYS_STATUS: {
                mavlink_sys_status_t sys_status;
                mavlink_msg_sys_status_decode(&msg, &sys_status);
                std::cout << std::fixed << std::setprecision(2)
                          << "SYS_STATUS: voltage=" << sys_status.voltage_battery / 1000.0 << "V, "
                          << "battery=" << static_cast<int>(sys_status.battery_remaining) << "%"
                          << std::endl;
                break;
            }
            case MAVLINK_MSG_ID_GPS_RAW_INT: {
                mavlink_gps_raw_int_t gps;
                mavlink_msg_gps_raw_int_decode(&msg, &gps);
                std::cout << "GPS_RAW_INT: fix_type=" << static_cast<int>(gps.fix_type)
                          << ", satellites=" << static_cast<int>(gps.satellites_visible)
                          << std::endl;
                break;
            }
            case MAVLINK_MSG_ID_STATUSTEXT: {
                mavlink_statustext_t st;
                mavlink_msg_statustext_decode(&msg, &st);
                std::cout << "STATUSTEXT: " << statustext_to_string(st) << std::endl;
                break;
            }
        }
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
