// Reads real vehicle/target state and pushes it through FecEncoder ->
// UdpSender at rate_hz. See gcs/PLAN.md for the wire-format/FEC design.
//
// mode/armed come straight from MAVLink HEARTBEAT, read over
// setting/MAVLink.yaml's real.proxy_udp address on the mavlink_gcs port
// (setting/port.yaml) -- like control.cpp/target_distance.cpp, this does
// NOT open the Pixhawk's serial port directly. That used to be a real
// problem (see git history/control/README.md): opening the serial port
// directly, the same way check_link.cpp/check_alt.cpp do, meant only one
// process could reliably own it at a time, so this would contend with
// control.cpp/target_distance.cpp during an actual flight. Fixed by running
// control/mavlink_proxy.cpp - see its top comment - which owns the serial
// port alone and fans it out to mavlink_control/mavlink_sensor/mavlink_gcs.
//
// altitude + target-detection state are NOT read independently here --
// they're read from target_distance.cpp's existing TargetRangeMsg loopback
// broadcast (control/target_link.hpp), which already fuses YOLO's /target
// HTTP endpoint with a fresh ALTITUDE reading for control.cpp's own use.
// Reusing that avoids a second HTTP client and a second ALTITUDE stream
// request; it does mean this telemetry alt/target fields go stale if
// target_distance.cpp isn't running.

#include <unistd.h>
#include <limits.h>

#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <thread>

#include "drone_lib.hpp"
#include "target_link.hpp"
#include "fec_encoder.hpp"
#include "udp_sender.hpp"
#include "yaml_settings.hpp"

namespace {

volatile std::sig_atomic_t g_stop = 0;
void on_sigint(int) { g_stop = 1; }

// Mirrors yaml_settings.cpp's private load_setting_file() path resolution
// (exe_dir/../../setting/<file>), duplicated here instead of exported from
// that shared file so other executables' build stays untouched.
YamlValue load_gcs_yaml() {
    char exe_path[PATH_MAX];
    ssize_t len = readlink("/proc/self/exe", exe_path, sizeof(exe_path) - 1);
    std::string exe_dir = ".";
    if (len != -1) {
        exe_path[len] = '\0';
        std::string full(exe_path);
        exe_dir = full.substr(0, full.find_last_of('/'));
    }
    const std::string candidates[] = {
        exe_dir + "/../../setting/gcs.yaml",
        exe_dir + "/../setting/gcs.yaml",
    };
    for (const auto& c : candidates) {
        std::ifstream probe(c);
        if (probe.good()) return parse_yaml_file(c);
    }
    throw std::runtime_error("gcs.yaml not found relative to executable");
}

// Same helper as check_alt.cpp's anonymous-namespace request_message_interval
// -- not exported from that file, so duplicated here.
void request_message_interval(MavConnection& connection, uint32_t message_id, double frequency_hz) {
    int64_t interval_us = static_cast<int64_t>(1'000'000 / frequency_hz);
    mavlink_message_t msg;
    mavlink_msg_command_long_pack(255, 0, &msg, connection.target_system(),
                                   connection.target_component(), MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                                   static_cast<float>(message_id), static_cast<float>(interval_us),
                                   0, 0, 0, 0, 0);
    connection.send(msg);
}

// Same helper as check_link.cpp's anonymous-namespace mode_string.
std::string mode_string(uint32_t custom_mode) {
    for (const auto& entry : copter_mode_mapping()) {
        if (entry.second == custom_mode) return entry.first;
    }
    return "Mode(" + std::to_string(custom_mode) + ")";
}

struct VehicleState {
    std::string mode = "UNKNOWN";
    bool armed = false;
    bool have_heartbeat = false;
};

// Retries open_connection()+wait_heartbeat() until one succeeds or g_stop
// is set (returns nullptr in that case). Both can throw (e.g. mavlink_proxy
// not running yet, so nothing is bound on the other end) as well as just
// time out without a heartbeat; either way this is a telemetry/debug feed,
// not the flight-control link, so it should keep trying rather than take
// the whole process down. Also called mid-run to reconnect after a read
// error further down.
std::unique_ptr<MavConnection> connect_mavlink(const std::string& address, double heartbeat_timeout) {
    while (!g_stop) {
        try {
            std::fprintf(stderr, "telem_sender: waiting for Pixhawk heartbeat on %s...\n", address.c_str());
            auto mav = open_connection(address);
            if (mav->wait_heartbeat(heartbeat_timeout)) {
                std::fprintf(stderr, "telem_sender: connected (system=%d component=%d)\n",
                             mav->target_system(), mav->target_component());
                return mav;
            }
            std::fprintf(stderr, "telem_sender: no heartbeat within %.0fs, retrying...\n",
                         heartbeat_timeout);
        } catch (const std::exception& e) {
            std::fprintf(stderr, "telem_sender: MAVLink connect failed (%s), retrying in 3s...\n",
                         e.what());
            std::this_thread::sleep_for(std::chrono::seconds(3));
        }
    }
    return nullptr;
}

std::string make_telemetry_json(uint32_t seq, const VehicleState& vehicle,
                                 const TargetRangeMsg& target, bool have_target) {
    using namespace std::chrono;
    const auto ts_ms =
        duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
    std::ostringstream os;
    os << "{\"seq\":" << seq << ",\"ts_ms\":" << ts_ms << ",\"mode\":\"" << vehicle.mode << "\""
       << ",\"armed\":" << (vehicle.armed ? "true" : "false")
       << ",\"link_ok\":" << (vehicle.have_heartbeat ? "true" : "false");
    if (have_target && target.altitude_valid) {
        os << ",\"alt_m\":" << target.altitude_m;
    } else {
        os << ",\"alt_m\":null";
    }
    if (have_target && target.valid) {
        os << ",\"target_detected\":" << (target.found ? "true" : "false")
           << ",\"target_x_px\":" << target.x_px << ",\"target_y_px\":" << target.y_px
           << ",\"target_distance_m\":" << target.distance_m;
    } else {
        os << ",\"target_detected\":null";
    }
    os << "}";
    return os.str();
}

}  // namespace

int main() {
    std::signal(SIGINT, on_sigint);

    YamlValue cfg = load_gcs_yaml();
    const YamlValue& telem = cfg["telemetry"];
    const std::string dest_host = telem["dest_host"].as_string();
    const long dest_port = telem["dest_port"].as_long();
    const long lanes = telem.get_long_or("lanes", 3);
    const long group_size = telem.get_long_or("group_size", 4);
    const long rate_hz = telem.get_long_or("rate_hz", 10);

    YamlValue mav_cfg = drone::load_mavlink_settings();
    const double heartbeat_timeout = mav_cfg.get_double_or("heartbeat_timeout", 20);
    const long target_udp_port = mav_cfg["target_track"].get_long_or("udp_port", 15020);

    YamlValue port_cfg = drone::load_port_settings();
    const long mavlink_gcs_port = port_cfg.get_long_or("mavlink_gcs", 14552);
    const std::string mav_address =
        with_port(mav_cfg["real"]["proxy_udp"]["address"].as_string(), mavlink_gcs_port);

    std::fprintf(stderr, "telem_sender: -> %s:%ld lanes=%ld group_size=%ld rate=%ldhz\n",
                 dest_host.c_str(), dest_port, lanes, group_size, rate_hz);

    auto mav = connect_mavlink(mav_address, heartbeat_timeout);
    if (!mav) {
        std::fprintf(stderr, "telem_sender: stopped before a Pixhawk connection was made\n");
        return 0;
    }
    request_message_interval(*mav, MAVLINK_MSG_ID_ALTITUDE, static_cast<double>(rate_hz));

    TargetRangeReceiver target_rx(static_cast<int>(target_udp_port));

    gcs::UdpSender udp(dest_host, static_cast<uint16_t>(dest_port));
    gcs::FecEncoder enc(static_cast<uint8_t>(lanes), static_cast<uint8_t>(group_size),
                         [&udp](const uint8_t* data, size_t len) { udp.send(data, len); });

    const auto period = std::chrono::microseconds(1000000 / rate_hz);
    const double poll_timeout_s = std::min(0.02, 1.0 / static_cast<double>(rate_hz) / 4.0);

    VehicleState vehicle;
    TargetRangeMsg target{};
    bool have_target = false;

    uint32_t seq = 0;
    uint32_t sent_count = 0;
    while (!g_stop) {
        const auto next_tick = std::chrono::steady_clock::now() + period;

        // Opportunistic, short-timeout drain -- doesn't block the telemetry
        // rate waiting for MAVLink; a message not seen this tick is just
        // picked up next tick (this feed is for status/debugging, not
        // control, so a tick or two of staleness is fine).
        //
        // recv_match can throw if the underlying transport read fails (the
        // serial device node disappearing mid-run -- USB unplug/reset is
        // the real-world case this guards against). Reconnecting here
        // rather than letting the exception escape is the whole point of
        // this loop: a telemetry/debug feed should degrade (link_ok=false
        // downstream) and keep trying, not take the process down.
        try {
            mavlink_message_t msg;
            if (mav->recv_match({MAVLINK_MSG_ID_HEARTBEAT, MAVLINK_MSG_ID_ALTITUDE}, msg, poll_timeout_s)) {
                if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
                    mavlink_heartbeat_t hb;
                    mavlink_msg_heartbeat_decode(&msg, &hb);
                    vehicle.mode = mode_string(hb.custom_mode);
                    vehicle.armed = is_armed_from_heartbeat(hb);
                    vehicle.have_heartbeat = true;
                }
                // ALTITUDE is requested above so target_distance.cpp (if
                // also running) isn't the only consumer, but we don't
                // decode it here -- target.altitude_m via TargetRangeMsg
                // below already carries it, computed from the same stream.
            }
        } catch (const std::exception& e) {
            std::fprintf(stderr, "telem_sender: MAVLink read error (%s), reconnecting...\n", e.what());
            vehicle.have_heartbeat = false;
            mav = connect_mavlink(mav_address, heartbeat_timeout);
            if (!mav) break;  // g_stop was set while reconnecting
            request_message_interval(*mav, MAVLINK_MSG_ID_ALTITUDE, static_cast<double>(rate_hz));
        }
        if (target_rx.poll(target)) {
            have_target = true;
        }

        const std::string payload = make_telemetry_json(seq, vehicle, target, have_target);
        enc.submit(reinterpret_cast<const uint8_t*>(payload.data()),
                   static_cast<uint16_t>(payload.size()));

        ++sent_count;
        if (sent_count % (static_cast<uint32_t>(rate_hz) * 5) == 0) {
            std::fprintf(stderr, "telem_sender: sent %u messages (mode=%s armed=%d)\n", sent_count,
                         vehicle.mode.c_str(), vehicle.armed);
        }

        ++seq;
        std::this_thread::sleep_until(next_tick);
    }

    enc.flush();
    std::fprintf(stderr, "telem_sender: stopped after %u messages\n", sent_count);
    return 0;
}
