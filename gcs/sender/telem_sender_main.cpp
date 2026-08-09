// Phase-1 test harness: generates synthetic telemetry JSON at a fixed rate
// and pushes it through FecEncoder -> UdpSender. Proves the FEC/transport
// layer end to end; wiring real yolo_live.cpp/MAVLink data in is a later
// phase (see gcs/PLAN.md).

#include <unistd.h>
#include <limits.h>

#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <thread>

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

std::string make_telemetry_json(uint32_t seq, double alt_m, bool target_detected,
                                 const std::string& mode, bool armed) {
    using namespace std::chrono;
    const auto ts_ms =
        duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
    std::ostringstream os;
    os << "{\"seq\":" << seq << ",\"ts_ms\":" << ts_ms << ",\"alt_m\":" << alt_m
       << ",\"target_detected\":" << (target_detected ? "true" : "false") << ",\"mode\":\""
       << mode << "\"" << ",\"armed\":" << (armed ? "true" : "false") << "}";
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

    std::fprintf(stderr, "telem_sender: -> %s:%ld lanes=%ld group_size=%ld rate=%ldhz\n",
                 dest_host.c_str(), dest_port, lanes, group_size, rate_hz);

    gcs::UdpSender udp(dest_host, static_cast<uint16_t>(dest_port));
    gcs::FecEncoder enc(static_cast<uint8_t>(lanes), static_cast<uint8_t>(group_size),
                         [&udp](const uint8_t* data, size_t len) { udp.send(data, len); });

    const auto period = std::chrono::microseconds(1000000 / rate_hz);
    uint32_t seq = 0;
    uint32_t sent_count = 0;
    while (!g_stop) {
        const auto next_tick = std::chrono::steady_clock::now() + period;

        const double alt_m = 10.0 + 2.0 * std::sin(seq * 0.05);
        const bool target = (seq % 37) < 5;
        const std::string payload = make_telemetry_json(seq, alt_m, target, "GUIDED", true);

        enc.submit(reinterpret_cast<const uint8_t*>(payload.data()),
                   static_cast<uint16_t>(payload.size()));

        ++sent_count;
        if (sent_count % (static_cast<uint32_t>(rate_hz) * 5) == 0) {
            std::fprintf(stderr, "telem_sender: sent %u messages\n", sent_count);
        }

        ++seq;
        std::this_thread::sleep_until(next_tick);
    }

    std::fprintf(stderr, "telem_sender: stopped after %u messages\n", sent_count);
    return 0;
}
