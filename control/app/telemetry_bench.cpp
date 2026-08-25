#include "../autopilot/autopilot_mavlink_adapter.hpp"
#include "../drone_lib.hpp"
#include "../yaml_settings.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

namespace {

std::atomic<bool> stopping{false};

void handle_signal(int) {
    stopping.store(true);
}

struct Options {
    std::string endpoint;
    double duration_sec = 0.0;
    int baud = 115200;
    std::string telemetry_endpoint;
    std::string gcs_telemetry_endpoint;
    std::string health_telemetry_endpoint;
};

std::string next_value(int& index, int argc, char** argv, const char* option) {
    if (index + 1 >= argc) {
        throw std::runtime_error(std::string(option) + " requires a value");
    }
    return argv[++index];
}

Options parse_options(int argc, char** argv) {
    Options options;
    for (int index = 1; index < argc; ++index) {
        const std::string argument = argv[index];
        if (argument == "--connect") {
            options.endpoint = next_value(index, argc, argv, "--connect");
        } else if (argument == "--duration-sec") {
            options.duration_sec = std::stod(next_value(index, argc, argv, "--duration-sec"));
        } else if (argument == "--baud") {
            options.baud = std::stoi(next_value(index, argc, argv, "--baud"));
        } else if (argument == "--telemetry-endpoint") {
            options.telemetry_endpoint = next_value(index, argc, argv, "--telemetry-endpoint");
        } else if (argument == "--gcs-telemetry-endpoint") {
            options.gcs_telemetry_endpoint = next_value(index, argc, argv, "--gcs-telemetry-endpoint");
        } else if (argument == "--health-telemetry-endpoint") {
            options.health_telemetry_endpoint = next_value(index, argc, argv, "--health-telemetry-endpoint");
        } else if (argument == "--help" || argument == "-h") {
            std::cout << "Usage: telemetry_bench --connect /dev/serial/by-id/... "
                         "[--duration-sec SEC] [--baud BAUD] "
                         "[--telemetry-endpoint udp:127.0.0.1:PORT] "
                         "[--gcs-telemetry-endpoint udp:127.0.0.1:PORT] "
                         "[--health-telemetry-endpoint udp:127.0.0.1:PORT]\n";
            std::exit(0);
        } else {
            throw std::runtime_error("unknown telemetry-bench option: " + argument);
        }
    }

    if (!app::serial_endpoint_is_allowed(options.endpoint)) {
        throw std::runtime_error(
            "telemetry-bench requires an explicit /dev/serial/by-id/... endpoint");
    }
    if (options.baud <= 0) throw std::runtime_error("--baud must be positive");
    if (options.duration_sec < 0.0) {
        throw std::runtime_error("--duration-sec must be zero or positive");
    }
    return options;
}

bool loopback_udp(const std::string& endpoint) {
    return endpoint.rfind("udp:127.0.0.1:", 0) == 0;
}

}  // namespace

int main(int argc, char** argv) {
    try {
        std::signal(SIGINT, handle_signal);
        std::signal(SIGTERM, handle_signal);
        const Options options = parse_options(argc, argv);

        const YamlValue settings = load_runtime_settings("real");
        const YamlValue transport = settings["transport"];
        const std::string telemetry_endpoint = options.telemetry_endpoint.empty()
            ? transport.get_string_or("telemetry_endpoint", "udp:127.0.0.1:14551")
            : options.telemetry_endpoint;
        const std::string gcs_endpoint = options.gcs_telemetry_endpoint.empty()
            ? transport.get_string_or("gcs_telemetry_endpoint", "udp:127.0.0.1:14553")
            : options.gcs_telemetry_endpoint;
        const std::string health_endpoint = options.health_telemetry_endpoint.empty()
            ? transport.get_string_or("health_telemetry_endpoint", "udp:127.0.0.1:14554")
            : options.health_telemetry_endpoint;
        if (!loopback_udp(telemetry_endpoint) || !loopback_udp(gcs_endpoint) ||
            !loopback_udp(health_endpoint)) {
            throw std::runtime_error(
                "telemetry-bench fan-out endpoints must use udp:127.0.0.1:<port>");
        }

        app::RuntimeConfig config{};
        config.target = app::RuntimeTarget::Real;
        config.role = app::TransportRole::TelemetrySubscriber;
        config.endpoint = options.endpoint;
        config.serial_endpoint = options.endpoint;
        config.telemetry_endpoint = telemetry_endpoint;
        config.gcs_telemetry_endpoint = gcs_endpoint;
        config.health_telemetry_endpoint = health_endpoint;
        config.telemetry_fanout_endpoints = {
            telemetry_endpoint, gcs_endpoint, health_endpoint};
        config.baud = options.baud;
        config.command_mode = app::CommandMode::Observe;
        config.commands_enabled = false;
        config.allow_mavlink_writes = false;
        config.allow_vehicle_commands = false;
        config.allow_arm = false;
        config.allow_telemetry_configuration = false;

        std::cout << "[BENCH] read-only telemetry bench; flight disabled\n"
                  << "[BENCH] Pixhawk serial=" << options.endpoint
                  << " baud=" << options.baud << "\n"
                  << "[BENCH] telemetry fan-out=" << telemetry_endpoint << ','
                  << gcs_endpoint << ',' << health_endpoint << std::endl;

        auto adapter = std::make_unique<autopilot::AutopilotMavlinkAdapter>(
            open_connection(options.endpoint, options.baud), config);
        const auto started = std::chrono::steady_clock::now();
        const bool run_forever = options.duration_sec == 0.0;
        const auto deadline = started + std::chrono::duration_cast<
            std::chrono::steady_clock::duration>(
                std::chrono::duration<double>(options.duration_sec));

        while (!stopping.load() &&
               (run_forever || std::chrono::steady_clock::now() < deadline)) {
            adapter->poll(0.05);
            if (adapter->tx_packet_count() != 0) {
                throw std::runtime_error(
                    "telemetry-bench observed an unexpected MAVLink transmission");
            }
        }

        std::cout << "[BENCH] mavlink_tx_packets=" << adapter->tx_packet_count()
                  << "\n[BENCH] telemetry bench stopped; flight disabled" << std::endl;
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "telemetry_bench: " << error.what() << std::endl;
        return 1;
    }
}
