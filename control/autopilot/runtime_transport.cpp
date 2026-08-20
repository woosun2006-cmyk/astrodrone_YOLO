#include "autopilot/runtime_transport.hpp"

#include "yaml_settings.hpp"

#include <cstdlib>
#include <stdexcept>

namespace autopilot {
namespace {

bool is_serial_endpoint(const std::string& endpoint) {
    return endpoint.rfind("udp:", 0) != 0 && endpoint.rfind("tcp:", 0) != 0 &&
           endpoint.rfind("fake:", 0) != 0;
}

bool is_loopback_network_endpoint(const std::string& endpoint) {
    const bool network = endpoint.rfind("udp:", 0) == 0 || endpoint.rfind("tcp:", 0) == 0;
    return !network || endpoint.find("127.0.0.1:") != std::string::npos;
}

RuntimeTarget parse_target(const char* value) {
    const std::string target = value == nullptr || *value == '\0' ? "sitl" : value;
    if (target == "sitl") return RuntimeTarget::Sitl;
    if (target == "real") return RuntimeTarget::Real;
    throw std::runtime_error("ASTRODRONE_TARGET must be 'sitl' or 'real'");
}

CommandMode parse_command_mode(const char* value, RuntimeTarget target) {
    const std::string mode = value == nullptr || *value == '\0'
        ? (target == RuntimeTarget::Real ? "observe" : "flight")
        : value;
    if (mode == "observe") return CommandMode::Observe;
    if (mode == "shadow") return CommandMode::Shadow;
    if (mode == "flight") return CommandMode::Flight;
    throw std::runtime_error(
        "ASTRODRONE_COMMAND_MODE must be 'observe', 'shadow', or 'flight'");
}

bool env_flag(const char* name, bool fallback) {
    const char* value = std::getenv(name);
    if (value == nullptr || *value == '\0') return fallback;
    return std::string(value) == "1" || std::string(value) == "true" ||
           std::string(value) == "TRUE" || std::string(value) == "yes";
}

}  // namespace

bool endpoint_is_serial(const std::string& endpoint) {
    return is_serial_endpoint(endpoint);
}

RuntimeTarget runtime_target_from_environment() {
    const char* value = std::getenv("ASTRODRONE_TARGET");
    if (value == nullptr) value = std::getenv("DRONE_TARGET");
    return parse_target(value);
}

const char* runtime_target_name(RuntimeTarget target) {
    return target == RuntimeTarget::Real ? "real" : "sitl";
}

const char* command_mode_name(CommandMode mode) {
    switch (mode) {
        case CommandMode::Observe: return "observe";
        case CommandMode::Shadow: return "shadow";
        case CommandMode::Flight: return "flight";
    }
    return "unknown";
}

CommandMode command_mode_from_environment(RuntimeTarget target) {
    const char* value = std::getenv("ASTRODRONE_COMMAND_MODE");
    if (value == nullptr) value = std::getenv("DRONE_COMMAND_MODE");
    return parse_command_mode(value, target);
}

RuntimeTransportConfig load_runtime_transport(RuntimeTarget target, TransportRole role) {
    const YamlValue settings = load_runtime_settings(runtime_target_name(target));
    const YamlValue transport = settings["transport"];
    const std::string role_key = role == TransportRole::CommandOwner
        ? "command_endpoint"
        : "telemetry_endpoint";

    RuntimeTransportConfig config;
    config.target = target;
    config.role = role;
    config.endpoint = transport[role_key].as_string();
    config.serial_endpoint = transport.get_string_or("serial_endpoint", "");
    config.baud = static_cast<int>(transport.get_long_or("baud", 115200));
    config.serial_owner = transport.get_string_or("serial_owner", "none");
    config.camera_source = settings["camera"].get_string_or("source", "unknown");
    const CommandMode default_mode = target == RuntimeTarget::Real
        ? CommandMode::Observe
        : CommandMode::Flight;
    config.command_mode = default_mode;
    try {
        const std::string configured_mode = settings["command"]["mode"].as_string();
        config.command_mode = parse_command_mode(configured_mode.c_str(), target);
    } catch (const std::exception&) {
        try {
            const std::string configured_mode = settings["flight"]["command_mode"].as_string();
            config.command_mode = parse_command_mode(configured_mode.c_str(), target);
        } catch (const std::exception&) {
            // Preserve the safe target-specific default for older files.
        }
    }
    const char* mode_env = std::getenv("ASTRODRONE_COMMAND_MODE");
    if (mode_env == nullptr) mode_env = std::getenv("DRONE_COMMAND_MODE");
    if (mode_env != nullptr && *mode_env != '\0') {
        config.command_mode = parse_command_mode(mode_env, target);
    }

    const YamlValue flight = settings["flight"];
    config.commands_enabled = flight.get_bool_or("commands_enabled", target == RuntimeTarget::Sitl);
    config.requires_allow_arm = flight.get_bool_or("requires_allow_arm", target == RuntimeTarget::Real);
    config.requires_real_confirmation = flight.get_bool_or(
        "requires_confirm_real_flight", target == RuntimeTarget::Real);

    const bool configured_writes = flight.get_bool_or("allow_mavlink_writes",
                                                       config.commands_enabled);
    const bool configured_vehicle = flight.get_bool_or("allow_vehicle_commands",
                                                        config.commands_enabled);
    const bool configured_arm = flight.get_bool_or("allow_arm", false);
    config.allow_mavlink_writes = env_flag("ASTRODRONE_ALLOW_MAVLINK_WRITES",
                                            configured_writes);
    config.allow_vehicle_commands = env_flag("ASTRODRONE_ALLOW_VEHICLE_COMMANDS",
                                              configured_vehicle);
    config.allow_arm = env_flag("ASTRODRONE_ALLOW_ARM", configured_arm);
    config.allow_telemetry_configuration = flight.get_bool_or(
        "allow_telemetry_configuration", config.allow_mavlink_writes);
    if (std::getenv("ASTRODRONE_ALLOW_TELEMETRY_CONFIGURATION") != nullptr) {
        config.allow_telemetry_configuration = env_flag(
            "ASTRODRONE_ALLOW_TELEMETRY_CONFIGURATION", false);
    }

    // Observe and shadow are receive/calculation modes. They can never be
    // turned into a write-capable mode by an old YAML boolean.
    if (config.command_mode != CommandMode::Flight) {
        config.allow_mavlink_writes = false;
        config.allow_vehicle_commands = false;
        config.allow_arm = false;
        config.allow_telemetry_configuration = false;
        config.commands_enabled = false;
    }
    if (config.command_mode == CommandMode::Flight) {
        config.commands_enabled = config.allow_vehicle_commands;
    }

    // A telemetry subscriber must never open the Pixhawk serial device. The
    // serial owner, either an external router or a future broker, fans out
    // MAVLink to the configured UDP telemetry endpoint.
    if (role == TransportRole::TelemetrySubscriber && is_serial_endpoint(config.endpoint)) {
        throw std::runtime_error(
            "telemetry subscriber cannot open a serial endpoint; configure a shared UDP telemetry endpoint");
    }

    if (target == RuntimeTarget::Real && config.serial_owner != "external-router") {
        throw std::runtime_error(
            "real runtime requires serial_owner=external-router; the application never owns Pixhawk serial");
    }

    if (target == RuntimeTarget::Real && is_serial_endpoint(config.endpoint)) {
        throw std::runtime_error(
            "real runtime accepts loopback UDP/TCP only; direct serial endpoint is forbidden");
    }

    if (!is_loopback_network_endpoint(config.endpoint)) {
        throw std::runtime_error("runtime MAVLink network endpoint must use 127.0.0.1");
    }

    return config;
}

}  // namespace autopilot
