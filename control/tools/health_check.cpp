#include "../autopilot/autopilot_mavlink_adapter.hpp"
#include "../app/runtime_config.hpp"
#include "../drone_lib.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <deque>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {

using Clock = autopilot::AutopilotState::Clock;
constexpr double kFreshnessSec = 3.0;
constexpr std::size_t kRecentStatusLimit = 8;

struct Options {
    app::RuntimeTarget target = app::RuntimeTarget::Sitl;
    std::string endpoint;
    double duration_sec = 10.0;
    int period_ms = 500;
    bool json = false;
    bool terminal_output = false;
    bool telemetry_bench = false;
};

struct Freshness {
    bool available = false;
    bool fresh = false;
    double age_sec = -1.0;
};

struct Counters {
    Clock::time_point heartbeat_timestamp{};
    Clock::time_point gps_timestamp{};
    Clock::time_point battery_timestamp{};
    Clock::time_point ekf_timestamp{};
    Clock::time_point rc_timestamp{};
    Clock::time_point status_timestamp{};
    std::size_t heartbeat_count = 0;
    uint64_t heartbeat_observation_count = 0;
    std::size_t gps_count = 0;
    std::size_t battery_count = 0;
    std::size_t ekf_count = 0;
    std::size_t rc_count = 0;
    std::size_t status_count = 0;
    std::size_t stale_count = 0;
    double max_heartbeat_gap_sec = 0.0;
    bool stale_now = false;
};

long long monotonic_ms() {
    const auto now = Clock::now();
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               now.time_since_epoch())
        .count();
}

long long wall_clock_ms() {
    const auto now = std::chrono::system_clock::now();
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               now.time_since_epoch())
        .count();
}

std::string health_trace_path() {
    const char* explicit_path = std::getenv("HEALTH_CHECK_TRACE_LOG");
    if (explicit_path != nullptr && *explicit_path != '\0') return explicit_path;
    const char* directory = std::getenv("LOG_DIR");
    if (directory == nullptr || *directory == '\0') return {};
    return std::string(directory) + "/health-check-heartbeat-trace.log";
}

std::string json_escape(const std::string& value) {
    std::ostringstream out;
    for (const char character : value) {
        switch (character) {
            case '\\': out << "\\\\"; break;
            case '"': out << "\\\""; break;
            case '\n': out << "\\n"; break;
            case '\r': out << "\\r"; break;
            case '\t': out << "\\t"; break;
            default: out << character; break;
        }
    }
    return out.str();
}

std::string mode_name(uint32_t custom_mode) {
    for (const auto& entry : copter_mode_mapping()) {
        if (entry.second == custom_mode) return entry.first;
    }
    return "UNKNOWN(" + std::to_string(custom_mode) + ")";
}

Freshness freshness(bool have, Clock::time_point timestamp, Clock::time_point now) {
    if (!have || timestamp == Clock::time_point{}) return {};
    const double age = std::max(0.0, std::chrono::duration<double>(now - timestamp).count());
    return {true, age <= kFreshnessSec, age};
}

std::string freshness_label(const Freshness& value) {
    if (!value.available) return "UNAVAILABLE";
    return value.fresh ? "FRESH" : "STALE";
}

std::string age_text(const Freshness& value) {
    if (!value.available) return "age=n/a";
    std::ostringstream out;
    out << "age=" << std::fixed << std::setprecision(2) << value.age_sec << "s";
    return out.str();
}

bool loopback_endpoint(const std::string& endpoint) {
    const bool network = endpoint.rfind("udp:", 0) == 0 || endpoint.rfind("tcp:", 0) == 0;
    if (!network) return false;
    const std::size_t separator = endpoint.find_last_of(':');
    if (separator == std::string::npos || endpoint.substr(0, separator).find("127.0.0.1") == std::string::npos) {
        return false;
    }
    try {
        const int port = std::stoi(endpoint.substr(separator + 1));
        return port > 0 && port <= 65535;
    } catch (const std::exception&) {
        return false;
    }
}

Options parse_options(int argc, char** argv) {
    Options options;
    bool have_target = false;
    bool have_endpoint = false;
    for (int index = 1; index < argc; ++index) {
        const std::string argument = argv[index];
        auto next_value = [&](const char* name) {
            if (index + 1 >= argc) throw std::runtime_error(std::string(name) + " requires a value");
            return std::string(argv[++index]);
        };
        if (argument == "--target") {
            const std::string value = next_value("--target");
            if (value == "sitl") options.target = app::RuntimeTarget::Sitl;
            else if (value == "real") options.target = app::RuntimeTarget::Real;
            else throw std::runtime_error("--target must be sitl or real");
            have_target = true;
        } else if (argument == "--connect") {
            options.endpoint = next_value("--connect");
            have_endpoint = true;
        } else if (argument == "--duration-sec") {
            options.duration_sec = std::stod(next_value("--duration-sec"));
        } else if (argument == "--period-ms") {
            options.period_ms = std::stoi(next_value("--period-ms"));
        } else if (argument == "--json") {
            options.json = true;
        } else if (argument == "--health-check-terminal") {
            options.terminal_output = true;
        } else if (argument == "--telemetry-bench") {
            options.telemetry_bench = true;
        } else if (argument == "--help" || argument == "-h") {
            std::cout << "Usage: health_check --target sitl|real --connect ENDPOINT "
                         "[--duration-sec SEC] [--period-ms MS] [--json] "
                         "[--health-check-terminal] [--telemetry-bench]\n";
            std::exit(0);
        } else {
            throw std::runtime_error("unknown option: " + argument);
        }
    }
    if (!have_target || !have_endpoint) {
        throw std::runtime_error("--target and --connect are required");
    }
    if (!std::isfinite(options.duration_sec) || options.duration_sec < 0.0) {
        throw std::runtime_error("--duration-sec must be zero or positive");
    }
    if (options.period_ms <= 0) throw std::runtime_error("--period-ms must be positive");
    (void)options.terminal_output;
    if (options.target == app::RuntimeTarget::Real) {
        if (!app::serial_endpoint_is_allowed(options.endpoint) &&
            !loopback_endpoint(options.endpoint)) {
            throw std::runtime_error(
                "real health_check requires /dev/serial/by-id/... or a loopback telemetry fan-out endpoint");
        }
    } else if (!loopback_endpoint(options.endpoint)) {
        throw std::runtime_error("sitl health_check requires udp/tcp 127.0.0.1 endpoint");
    }
    return options;
}

void update_counters(const autopilot::AutopilotState& state, Counters& counters,
                     Clock::time_point now) {
    auto update = [](Clock::time_point current, Clock::time_point& previous,
                     std::size_t& count) {
        if (current == Clock::time_point{} || current == previous) return;
        previous = current;
        ++count;
    };
    if (state.last_heartbeat != Clock::time_point{} &&
        state.last_heartbeat != counters.heartbeat_timestamp) {
        if (counters.heartbeat_timestamp != Clock::time_point{}) {
            counters.max_heartbeat_gap_sec = std::max(
                counters.max_heartbeat_gap_sec,
                std::chrono::duration<double>(state.last_heartbeat - counters.heartbeat_timestamp).count());
        }
        counters.heartbeat_timestamp = state.last_heartbeat;
        const uint64_t observation_count = state.heartbeat_observation_count;
        if (observation_count > counters.heartbeat_observation_count) {
            counters.heartbeat_count += static_cast<std::size_t>(
                observation_count - counters.heartbeat_observation_count);
            counters.heartbeat_observation_count = observation_count;
        } else {
            ++counters.heartbeat_count;
        }
    }
    update(state.gps_updated_at, counters.gps_timestamp, counters.gps_count);
    update(state.battery_updated_at, counters.battery_timestamp, counters.battery_count);
    update(state.ekf_updated_at, counters.ekf_timestamp, counters.ekf_count);
    update(state.rc_updated_at, counters.rc_timestamp, counters.rc_count);
    update(state.status_text_updated_at, counters.status_timestamp, counters.status_count);

    const auto heartbeat = freshness(state.last_heartbeat != Clock::time_point{}, state.last_heartbeat, now);
    const auto gps = freshness(state.have_gps, state.gps_updated_at, now);
    const auto battery = freshness(state.have_battery, state.battery_updated_at, now);
    const auto ekf = freshness(state.have_ekf, state.ekf_updated_at, now);
    const bool stale = (heartbeat.available && !heartbeat.fresh) ||
                       (gps.available && !gps.fresh) ||
                       (battery.available && !battery.fresh) ||
                       (ekf.available && !ekf.fresh);
    if (stale && !counters.stale_now) ++counters.stale_count;
    counters.stale_now = stale;
}

std::string overall_status(const autopilot::AutopilotState& state, Clock::time_point now) {
    const Freshness heartbeat = freshness(state.last_heartbeat != Clock::time_point{}, state.last_heartbeat, now);
    const Freshness gps = freshness(state.have_gps, state.gps_updated_at, now);
    const Freshness battery = freshness(state.have_battery, state.battery_updated_at, now);
    const Freshness ekf = freshness(state.have_ekf, state.ekf_updated_at, now);
    if (!heartbeat.available || !gps.available || !battery.available || !ekf.available) return "WAITING";
    if (!heartbeat.fresh || !gps.fresh || !battery.fresh || !ekf.fresh) return "STALE";
    return "FRESH";
}

void print_text(const Options& options, const autopilot::AutopilotState& state,
                const Counters& counters, Clock::time_point started,
                Clock::time_point now) {
    const auto heartbeat = freshness(state.last_heartbeat != Clock::time_point{}, state.last_heartbeat, now);
    const auto mode = freshness(state.have_armed, state.mode_updated_at, now);
    const auto gps = freshness(state.have_gps, state.gps_updated_at, now);
    const auto battery = freshness(state.have_battery, state.battery_updated_at, now);
    const auto altitude = freshness(state.have_altitude, state.altitude_updated_at, now);
    const auto local = freshness(state.have_local_position, state.local_position_updated_at, now);
    const auto ekf = freshness(state.have_ekf, state.ekf_updated_at, now);
    const auto rc = freshness(state.have_rc, state.rc_updated_at, now);
    const double elapsed = std::chrono::duration<double>(now - started).count();
    std::cout << "[HEALTH] t=" << std::fixed << std::setprecision(2) << elapsed
              << "s status=" << overall_status(state, now) << " target="
              << app::runtime_target_name(options.target) << '\n';
    std::cout << "  HEARTBEAT: " << freshness_label(heartbeat) << ' ' << age_text(heartbeat)
              << " system=" << static_cast<int>(state.heartbeat_system_id)
              << " component=" << static_cast<int>(state.heartbeat_component_id)
              << " count=" << counters.heartbeat_count << '\n';
    std::cout << "  MODE: " << (state.have_armed ? mode_name(state.custom_mode) : "UNAVAILABLE")
              << " (" << freshness_label(mode) << ' ' << age_text(mode) << ")\n";
    std::cout << "  ARMED: " << (state.have_armed ? (state.armed ? "YES" : "NO") : "UNAVAILABLE")
              << " (" << freshness_label(mode) << ' ' << age_text(mode) << ")\n";
    const bool bench_gps_unavailable = options.telemetry_bench &&
        (!state.have_gps || state.fix_type < 2 || state.satellites == 0 ||
         state.satellites == 255);
    if (bench_gps_unavailable) {
        std::cout << "  GPS: UNAVAILABLE (no GPS fix) " << age_text(gps)
                  << " fix=" << static_cast<int>(state.fix_type)
                  << " satellites=" << static_cast<int>(state.satellites) << '\n';
    } else {
        std::cout << "  GPS: " << freshness_label(gps) << ' ' << age_text(gps)
                  << " fix=" << static_cast<int>(state.fix_type)
                  << " satellites=" << static_cast<int>(state.satellites) << '\n';
    }
    std::cout << "  ALTITUDE: " << freshness_label(altitude) << ' ' << age_text(altitude)
              << " relative=" << state.altitude_m << "m\n";
    std::cout << "  LOCAL_NED: " << freshness_label(local) << ' ' << age_text(local)
              << " x=" << state.local_x_m << " y=" << state.local_y_m
              << " z=" << state.local_z_m << "m\n";
    std::cout << "  VELOCITY_NED: " << freshness_label(local) << ' ' << age_text(local)
              << " vx=" << state.local_vx_mps << " vy=" << state.local_vy_mps
              << " vz=" << state.local_vz_mps << "m/s\n";
    std::cout << "  BATTERY: " << freshness_label(battery) << ' ' << age_text(battery)
              << " valid=" << (state.battery_valid ? "YES" : "NO")
              << " voltage=" << state.battery_voltage_v << "V current="
              << state.battery_current_a << "A remaining=" << state.battery_percent << "%\n";
    std::cout << "  RC: " << freshness_label(rc) << ' ' << age_text(rc)
              << " channels=" << static_cast<int>(state.rc_channel_count)
              << " rssi=" << static_cast<int>(state.rc_rssi) << "% values=";
    for (uint8_t index = 0; index < state.rc_channel_count && index < state.rc_channels.size(); ++index) {
        if (index != 0) std::cout << ',';
        std::cout << state.rc_channels[index];
    }
    std::cout << '\n';
    std::cout << "  EKF: " << freshness_label(ekf) << ' ' << age_text(ekf)
              << " flags=0x" << std::hex << state.ekf_flags << std::dec
              << " pos_horiz=" << state.ekf_pos_horiz_variance
              << " pos_vert=" << state.ekf_pos_vert_variance
              << " velocity=" << state.ekf_velocity_variance << '\n';
    const auto status = freshness(state.have_status_text, state.status_text_updated_at, now);
    std::cout << "  STATUSTEXT: " << freshness_label(status) << ' ' << age_text(status) << " value=\""
              << (state.have_status_text ? state.last_status_text : "UNAVAILABLE") << "\"\n";
    std::cout << std::flush;
}

void print_json(const Options& options, const autopilot::AutopilotState& state,
                const Counters& counters, Clock::time_point started,
                Clock::time_point now, const std::deque<std::string>& recent_status) {
    const auto heartbeat = freshness(state.last_heartbeat != Clock::time_point{}, state.last_heartbeat, now);
    const auto mode = freshness(state.have_armed, state.mode_updated_at, now);
    const auto gps = freshness(state.have_gps, state.gps_updated_at, now);
    const auto battery = freshness(state.have_battery, state.battery_updated_at, now);
    const auto altitude = freshness(state.have_altitude, state.altitude_updated_at, now);
    const auto local = freshness(state.have_local_position, state.local_position_updated_at, now);
    const auto ekf = freshness(state.have_ekf, state.ekf_updated_at, now);
    const auto rc = freshness(state.have_rc, state.rc_updated_at, now);
    const auto status = freshness(state.have_status_text, state.status_text_updated_at, now);
    const double elapsed = std::chrono::duration<double>(now - started).count();
    std::cout << std::fixed << std::setprecision(3)
              << "{\"health\":\"" << overall_status(state, now)
              << "\",\"target\":\"" << app::runtime_target_name(options.target)
              << "\",\"elapsed_sec\":" << elapsed
              << ",\"heartbeat\":{\"status\":\"" << freshness_label(heartbeat)
              << "\",\"age_sec\":" << heartbeat.age_sec << ",\"count\":" << counters.heartbeat_count
              << ",\"system\":" << static_cast<int>(state.heartbeat_system_id)
              << ",\"component\":" << static_cast<int>(state.heartbeat_component_id) << "}"
              << ",\"mode\":{\"status\":\"" << freshness_label(mode)
              << "\",\"age_sec\":" << mode.age_sec << ",\"value\":\""
              << (state.have_armed ? mode_name(state.custom_mode) : "UNAVAILABLE") << "\"}"
              << ",\"armed\":{\"status\":\"" << freshness_label(mode)
              << "\",\"age_sec\":" << mode.age_sec << ",\"value\":"
              << (state.have_armed && state.armed ? "true" : "false") << "}"
              << ",\"gps\":{\"status\":\"" << freshness_label(gps)
              << "\",\"age_sec\":" << gps.age_sec << ",\"fix_type\":" << static_cast<int>(state.fix_type)
              << ",\"satellites\":" << static_cast<int>(state.satellites) << "}"
              << ",\"altitude\":{\"status\":\"" << freshness_label(altitude)
              << "\",\"age_sec\":" << altitude.age_sec << ",\"relative_m\":" << state.altitude_m << "}"
              << ",\"local_ned\":{\"status\":\"" << freshness_label(local)
              << "\",\"age_sec\":" << local.age_sec << ",\"x_m\":" << state.local_x_m << ",\"y_m\":" << state.local_y_m
              << ",\"z_m\":" << state.local_z_m << ",\"vx_mps\":" << state.local_vx_mps
              << ",\"vy_mps\":" << state.local_vy_mps << ",\"vz_mps\":" << state.local_vz_mps << "}"
              << ",\"battery\":{\"status\":\"" << freshness_label(battery)
              << "\",\"age_sec\":" << battery.age_sec << ",\"valid\":"
              << (state.battery_valid ? "true" : "false") << ",\"voltage_v\":" << state.battery_voltage_v
              << ",\"current_a\":" << state.battery_current_a << ",\"remaining_pct\":" << state.battery_percent << "}"
              << ",\"ekf\":{\"status\":\"" << freshness_label(ekf) << "\",\"age_sec\":" << ekf.age_sec
              << ",\"flags\":" << state.ekf_flags << ",\"pos_horiz_variance\":" << state.ekf_pos_horiz_variance
              << ",\"pos_vert_variance\":" << state.ekf_pos_vert_variance
              << ",\"velocity_variance\":" << state.ekf_velocity_variance << "}"
              << ",\"rc\":{\"status\":\"" << freshness_label(rc) << "\",\"age_sec\":" << rc.age_sec
              << ",\"channels\":" << static_cast<int>(state.rc_channel_count)
              << ",\"rssi\":" << static_cast<int>(state.rc_rssi) << "}"
              << ",\"status_text\":{\"status\":\"" << freshness_label(status)
              << "\",\"age_sec\":" << status.age_sec << ",\"value\":\""
              << json_escape(state.have_status_text ? state.last_status_text : "") << "\"}"
              << ",\"recent_status_text\":[";
    for (std::size_t index = 0; index < recent_status.size(); ++index) {
        if (index != 0) std::cout << ',';
        std::cout << "\"" << json_escape(recent_status[index]) << "\"";
    }
    std::cout << "],\"tx_packets\":0}\n" << std::flush;
}

void print_summary(const Counters& counters, bool json) {
    if (json) {
        std::cout << "{\"summary\":{\"heartbeat_count\":" << counters.heartbeat_count
                  << ",\"max_heartbeat_gap_sec\":" << counters.max_heartbeat_gap_sec
                  << ",\"gps_count\":" << counters.gps_count
                  << ",\"battery_count\":" << counters.battery_count
                  << ",\"ekf_count\":" << counters.ekf_count
                  << ",\"rc_count\":" << counters.rc_count
                  << ",\"statustext_count\":" << counters.status_count
                  << ",\"stale_count\":" << counters.stale_count
                  << ",\"mavlink_tx_packets\":0}}\n";
        return;
    }
    std::cout << "[HEALTH SUMMARY]\n"
              << "  HEARTBEAT count=" << counters.heartbeat_count
              << " max_gap=" << counters.max_heartbeat_gap_sec << "s\n"
              << "  GPS count=" << counters.gps_count
              << " battery count=" << counters.battery_count
              << " EKF count=" << counters.ekf_count
              << " RC count=" << counters.rc_count
              << " STATUSTEXT count=" << counters.status_count << '\n'
              << "  stale_events=" << counters.stale_count
              << " mavlink_tx_packets=0\n" << std::flush;
}

}  // namespace

int main(int argc, char** argv) {
    try {
        const Options options = parse_options(argc, argv);
        if (options.telemetry_bench) {
            std::cout << "[BENCH] read-only telemetry bench; flight disabled\n";
        }
        app::RuntimeConfig config{};
        config.target = options.target;
        config.role = app::TransportRole::TelemetrySubscriber;
        config.endpoint = options.endpoint;
        config.serial_endpoint = options.target == app::RuntimeTarget::Real ? options.endpoint : "";
        config.command_mode = app::CommandMode::Observe;
        config.commands_enabled = false;
        config.allow_mavlink_writes = false;
        config.allow_vehicle_commands = false;
        config.allow_arm = false;
        config.allow_telemetry_configuration = false;

        // open_connection delegates endpoint validation, serial flock, and
        // MAVLink byte transport ownership to the existing C++ stack.
        auto adapter = std::make_unique<autopilot::AutopilotMavlinkAdapter>(
            open_connection(options.endpoint, 115200), config);

        Counters counters;
        std::deque<std::string> recent_status;
        std::ofstream heartbeat_trace(health_trace_path(),
                                      std::ios::out | std::ios::app);
        uint64_t last_traced_heartbeat_count = 0;
        long long last_health_heartbeat_mono_ms = -1;
        std::string last_seen_status;
        const Clock::time_point started = Clock::now();
        Clock::time_point next_report = started;
        const bool run_forever = options.duration_sec == 0.0;
        const Clock::time_point deadline = started +
            std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(options.duration_sec));
        while (run_forever || Clock::now() < deadline) {
            adapter->poll(0.05);
            if (adapter->tx_packet_count() != 0) {
                throw std::runtime_error("read-only health_check observed MAVLink transmission");
            }
            const Clock::time_point now = Clock::now();
            const auto& state = adapter->state();
            update_counters(state, counters, now);
            if (state.heartbeat_observation_count != 0 &&
                state.heartbeat_observation_count != last_traced_heartbeat_count) {
                const auto health_receive_mono_ms = monotonic_ms();
                const auto adapter_observed_mono_ms =
                    std::chrono::duration_cast<std::chrono::milliseconds>(
                        state.last_heartbeat_observed_at.time_since_epoch()).count();
                const auto adapter_updated_mono_ms =
                    std::chrono::duration_cast<std::chrono::milliseconds>(
                        state.last_heartbeat_state_updated_at.time_since_epoch()).count();
                const long long gap_ms = last_health_heartbeat_mono_ms < 0
                                             ? -1
                                             : adapter_observed_mono_ms -
                                                   last_health_heartbeat_mono_ms;
                if (heartbeat_trace.is_open()) {
                    heartbeat_trace
                        << "source=HEALTH_CHECK_HEARTBEAT mono_ms="
                        << health_receive_mono_ms << " wall_ms=" << wall_clock_ms()
                        << " event=HEALTH_CHECK_HEARTBEAT"
                        << " health_receive_mono_ms=" << health_receive_mono_ms
                        << " adapter_observed_mono_ms=" << adapter_observed_mono_ms
                        << " adapter_state_updated_mono_ms=" << adapter_updated_mono_ms
                        << " previous_heartbeat_gap_ms=" << gap_ms
                        << " mav_seq=" << static_cast<unsigned>(state.last_heartbeat_mav_seq)
                        << " system=" << static_cast<unsigned>(state.heartbeat_system_id)
                        << " component=" << static_cast<unsigned>(state.heartbeat_component_id)
                        << " observation_count=" << state.heartbeat_observation_count
                        << " tx_packets=" << adapter->tx_packet_count() << '\n';
                    heartbeat_trace.flush();
                }
                last_traced_heartbeat_count = state.heartbeat_observation_count;
                last_health_heartbeat_mono_ms = adapter_observed_mono_ms;
            }
            if (state.have_status_text && state.last_status_text != last_seen_status &&
                !state.last_status_text.empty()) {
                last_seen_status = state.last_status_text;
                if (std::find(recent_status.begin(), recent_status.end(), last_seen_status) == recent_status.end()) {
                    recent_status.push_back(last_seen_status);
                    while (recent_status.size() > kRecentStatusLimit) recent_status.pop_front();
                }
                if (!options.json) std::cout << "[STATUSTEXT] " << last_seen_status << '\n' << std::flush;
            }
            if (now >= next_report) {
                if (options.json) print_json(options, state, counters, started, now, recent_status);
                else print_text(options, state, counters, started, now);
                next_report += std::chrono::milliseconds(options.period_ms);
            }
        }
        if (adapter->tx_packet_count() != 0) {
            throw std::runtime_error("read-only health_check observed MAVLink transmission");
        }
        print_summary(counters, options.json);
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "health_check: " << error.what() << '\n';
        return 2;
    }
}
