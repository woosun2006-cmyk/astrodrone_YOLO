// Standalone vehicle-health diagnostic. Streams MAVLink SYS_STATUS /
// GPS_RAW_INT / HEARTBEAT / EKF_STATUS_REPORT (read-only, sends no MAVLink
// commands - same posture as check_alt.cpp) and checks each against
// setting/safety.yaml's battery_limit / heartbeat_limit / gps_limit /
// vehicle_health_limit / ekf_limit, printing WARN/BREACH transitions to
// the terminal.
//
// NOTE: this program is NOT wired into the automatic safety response.
// control.cpp evaluates these exact same checks itself (see its
// HealthState/HealthLimit) and is the one that actually switches to
// LOITER/LAND on a breach, because control.cpp is the only process that
// ever sends MAVLink commands to the vehicle - having a separate process
// relay "emergency" over a side channel turned out to be an unnecessary
// extra hop (and a blind spot if that process wasn't running). This binary
// is just a convenient way for a human to watch vehicle health in a
// terminal, independent of a flight.
#include <chrono>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "drone_lib.hpp"

namespace {

void request_message_interval(MavConnection& connection, uint32_t message_id, double frequency_hz) {
    int64_t interval_us = static_cast<int64_t>(1'000'000 / frequency_hz);
    mavlink_message_t msg;
    mavlink_msg_command_long_pack(255, 0, &msg, connection.target_system(),
                                   connection.target_component(), MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                                   static_cast<float>(message_id), static_cast<float>(interval_us),
                                   0, 0, 0, 0, 0);
    connection.send(msg);
}

// safety.yaml stores booleans as the bare strings "true"/"false" --
// YamlValue has no bool accessor of its own (nothing else in this repo has
// needed one yet), so parse it locally.
bool get_bool_or(const YamlValue& parent, const std::string& key, bool default_value) {
    try {
        return parent[key].as_string() == "true";
    } catch (const std::exception&) {
        return default_value;
    }
}

struct Args {
    std::string address;
    int baud = 115200;
    double heartbeat_timeout = 20;
    double poll_rate_hz = 5;

    long min_battery_percent = 20;
    double min_battery_voltage_v = 14.8;
    double max_heartbeat_gap_sec = 3;
    long min_fix_type = 3;
    long min_satellites = 6;
    bool require_prearm_healthy = true;
    bool require_normal_state = true;
    double ekf_pos_horiz_variance_max = 1.0;
    double ekf_velocity_variance_max = 1.0;
};

}  // namespace

int run(int argc, char** argv) {
    YamlValue mav_settings = drone::load_mavlink_settings();

    Args args;
    args.address = mav_settings["real"]["proxy_udp"]["address"].as_string();
    args.heartbeat_timeout = mav_settings.get_double_or("heartbeat_timeout", 20);

    try {
        YamlValue safety = drone::load_safety_settings();

        YamlValue alt_limit = safety["altitude_limit"];
        args.poll_rate_hz = alt_limit.get_double_or("poll_rate_hz", args.poll_rate_hz);

        YamlValue battery_limit = safety["battery_limit"];
        args.min_battery_percent = battery_limit.get_long_or("min_percent", args.min_battery_percent);
        args.min_battery_voltage_v =
            battery_limit.get_double_or("min_voltage_v", args.min_battery_voltage_v);

        YamlValue heartbeat_limit = safety["heartbeat_limit"];
        args.max_heartbeat_gap_sec =
            heartbeat_limit.get_double_or("max_gap_sec", args.max_heartbeat_gap_sec);

        YamlValue gps_limit = safety["gps_limit"];
        args.min_fix_type = gps_limit.get_long_or("min_fix_type", args.min_fix_type);
        args.min_satellites = gps_limit.get_long_or("min_satellites", args.min_satellites);

        YamlValue health_limit = safety["vehicle_health_limit"];
        args.require_prearm_healthy =
            get_bool_or(health_limit, "require_prearm_healthy", args.require_prearm_healthy);
        args.require_normal_state =
            get_bool_or(health_limit, "require_normal_state", args.require_normal_state);

        YamlValue ekf_limit = safety["ekf_limit"];
        args.ekf_pos_horiz_variance_max =
            ekf_limit.get_double_or("pos_horiz_variance_max", args.ekf_pos_horiz_variance_max);
        args.ekf_velocity_variance_max =
            ekf_limit.get_double_or("velocity_variance_max", args.ekf_velocity_variance_max);
    } catch (const std::exception& e) {
        std::cerr << "Warning: could not fully load setting/safety.yaml (" << e.what()
                  << "), missing sections fall back to built-in defaults." << std::endl;
    }

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        auto next = [&](const char* name) -> std::string {
            if (i + 1 >= argc) throw std::runtime_error(std::string(name) + " requires a value");
            return argv[++i];
        };
        if (arg == "--address") {
            args.address = next("--address");
        } else if (arg == "--heartbeat-timeout") {
            args.heartbeat_timeout = std::stod(next("--heartbeat-timeout"));
        }
    }

    // Read-only connection: this program never sends MAVLink commands, so
    // it doesn't need drone::connect()'s command-sending singleton - same
    // posture as check_alt.cpp.
    auto connection = open_connection(args.address, args.baud);
    std::cout << "Waiting for Pixhawk heartbeat on " << args.address << "..." << std::endl;
    if (!connection->wait_heartbeat(args.heartbeat_timeout)) {
        std::cerr << "No Pixhawk heartbeat received. Check the connection." << std::endl;
        return 1;
    }
    std::cout << "Connected to MAVLink system " << static_cast<int>(connection->target_system())
              << "." << std::endl;

    request_message_interval(*connection, MAVLINK_MSG_ID_SYS_STATUS, args.poll_rate_hz);
    request_message_interval(*connection, MAVLINK_MSG_ID_GPS_RAW_INT, args.poll_rate_hz);
    request_message_interval(*connection, MAVLINK_MSG_ID_EKF_STATUS_REPORT, args.poll_rate_hz);
    // HEARTBEAT is broadcast on its own (~1 Hz) without needing a request.

    std::cout << "Health watch (read-only, diagnostic only - see top-of-file note): battery>="
              << args.min_battery_percent << "% / " << args.min_battery_voltage_v
              << "V; heartbeat gap<" << args.max_heartbeat_gap_sec << "s; gps fix>="
              << args.min_fix_type << " sats>=" << args.min_satellites
              << "; prearm_healthy=" << (args.require_prearm_healthy ? "required" : "ignored")
              << "; normal_state=" << (args.require_normal_state ? "required" : "ignored")
              << "; ekf pos_horiz_variance<" << args.ekf_pos_horiz_variance_max << " velocity_variance<"
              << args.ekf_velocity_variance_max << std::endl;

    using clock = std::chrono::steady_clock;
    double poll_interval_sec = 1.0 / args.poll_rate_hz;

    auto last_heartbeat = clock::now();  // wait_heartbeat() above already saw one

    bool have_battery = false;
    int battery_percent = -1;
    double battery_voltage_v = -1;

    bool have_gps = false;
    uint8_t fix_type = 0;
    uint8_t satellites = 255;  // 255 == unknown, per MAVLink convention

    bool have_sys_status = false;
    bool prearm_healthy = true;
    uint8_t system_status = MAV_STATE_STANDBY;
    bool armed = false;

    bool have_ekf = false;
    double ekf_pos_horiz_variance = 0;
    double ekf_velocity_variance = 0;

    bool was_active = false;

    while (true) {
        mavlink_message_t msg;
        bool got = connection->recv_match(
            {MAVLINK_MSG_ID_SYS_STATUS, MAVLINK_MSG_ID_GPS_RAW_INT, MAVLINK_MSG_ID_HEARTBEAT,
             MAVLINK_MSG_ID_EKF_STATUS_REPORT},
            msg, poll_interval_sec);
        auto now = clock::now();

        if (got) {
            switch (msg.msgid) {
                case MAVLINK_MSG_ID_SYS_STATUS: {
                    mavlink_sys_status_t s;
                    mavlink_msg_sys_status_decode(&msg, &s);
                    battery_percent = s.battery_remaining;
                    battery_voltage_v = s.voltage_battery == 65535 ? -1 : s.voltage_battery / 1000.0;
                    prearm_healthy = (s.onboard_control_sensors_health & MAV_SYS_STATUS_PREARM_CHECK) != 0;
                    have_battery = true;
                    have_sys_status = true;
                    break;
                }
                case MAVLINK_MSG_ID_GPS_RAW_INT: {
                    mavlink_gps_raw_int_t g;
                    mavlink_msg_gps_raw_int_decode(&msg, &g);
                    fix_type = g.fix_type;
                    satellites = g.satellites_visible;
                    have_gps = true;
                    break;
                }
                case MAVLINK_MSG_ID_HEARTBEAT: {
                    mavlink_heartbeat_t hb;
                    mavlink_msg_heartbeat_decode(&msg, &hb);
                    system_status = hb.system_status;
                    armed = is_armed_from_heartbeat(hb);
                    last_heartbeat = now;
                    break;
                }
                case MAVLINK_MSG_ID_EKF_STATUS_REPORT: {
                    mavlink_ekf_status_report_t e;
                    mavlink_msg_ekf_status_report_decode(&msg, &e);
                    ekf_pos_horiz_variance = e.pos_horiz_variance;
                    ekf_velocity_variance = e.velocity_variance;
                    have_ekf = true;
                    break;
                }
                default:
                    break;
            }
        }

        // Evaluated every tick (not just when a message arrives) so a
        // stalled link is caught by the heartbeat-gap check even though no
        // message is ever going to arrive to trigger it.
        std::vector<std::string> reasons;

        double heartbeat_gap = std::chrono::duration<double>(now - last_heartbeat).count();
        if (heartbeat_gap > args.max_heartbeat_gap_sec) {
            std::ostringstream oss;
            oss << "heartbeat gap " << std::fixed << std::setprecision(1) << heartbeat_gap << "s > "
                << args.max_heartbeat_gap_sec << "s";
            reasons.push_back(oss.str());
        }
        if (have_battery && battery_percent >= 0 && battery_percent < args.min_battery_percent) {
            reasons.push_back("battery " + std::to_string(battery_percent) + "% < " +
                               std::to_string(args.min_battery_percent) + "%");
        }
        if (have_battery && battery_voltage_v >= 0 && battery_voltage_v < args.min_battery_voltage_v) {
            std::ostringstream oss;
            oss << "battery " << std::fixed << std::setprecision(2) << battery_voltage_v << "V < "
                << args.min_battery_voltage_v << "V";
            reasons.push_back(oss.str());
        }
        if (have_gps && satellites != 255 &&
            (fix_type < args.min_fix_type || satellites < args.min_satellites)) {
            reasons.push_back("gps fix_type=" + std::to_string(static_cast<int>(fix_type)) +
                               " satellites=" + std::to_string(static_cast<int>(satellites)));
        }
        if (have_sys_status && args.require_prearm_healthy && !prearm_healthy) {
            reasons.push_back("prearm check unhealthy");
        }
        if (have_sys_status && args.require_normal_state &&
            (system_status == MAV_STATE_CRITICAL || system_status == MAV_STATE_EMERGENCY)) {
            reasons.push_back("system_status=" + std::to_string(static_cast<int>(system_status)) +
                               " (CRITICAL/EMERGENCY - ArduPilot internal failsafe, e.g. EKF)");
        }
        if (have_ekf && (ekf_pos_horiz_variance > args.ekf_pos_horiz_variance_max ||
                         ekf_velocity_variance > args.ekf_velocity_variance_max)) {
            std::ostringstream oss;
            oss << "ekf pos_horiz_variance=" << std::fixed << std::setprecision(2) << ekf_pos_horiz_variance
                << " velocity_variance=" << ekf_velocity_variance;
            reasons.push_back(oss.str());
        }

        bool active = !reasons.empty();
        std::string reason_text;
        for (size_t i = 0; i < reasons.size(); ++i) {
            if (i > 0) reason_text += "; ";
            reason_text += reasons[i];
        }

        if (active && !was_active) {
            std::cout << "[BREACH] " << reason_text << std::endl;
        } else if (!active && was_active) {
            std::cout << "Health checks recovered." << std::endl;
        }
        was_active = active;

        std::cout << std::fixed << std::setprecision(2) << "battery=" << battery_percent << "% "
                  << battery_voltage_v << "V gps=fix" << static_cast<int>(fix_type) << "/"
                  << static_cast<int>(satellites) << "sat armed=" << (armed ? "Y" : "N")
                  << " prearm_healthy=" << (prearm_healthy ? "Y" : "N") << " ekf_pos_var="
                  << ekf_pos_horiz_variance << " ekf_vel_var=" << ekf_velocity_variance << " ["
                  << (active ? "BREACH" : "ok") << "]" << std::endl;
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
