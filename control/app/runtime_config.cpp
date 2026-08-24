#include "runtime_config.hpp"

#include "yaml_settings.hpp"

#include <cstdlib>
#include <stdexcept>

namespace app {
namespace {

bool is_serial_endpoint(const std::string& endpoint) {
    return endpoint.rfind("udp:", 0) != 0 && endpoint.rfind("tcp:", 0) != 0 &&
           endpoint.rfind("fake:", 0) != 0;
}

bool is_allowed_serial_endpoint(const std::string& endpoint) {
    static const std::string prefix = "/dev/serial/by-id/";
    return endpoint.rfind(prefix, 0) == 0 && endpoint.size() > prefix.size();
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

bool serial_endpoint_is_allowed(const std::string& endpoint) {
    return is_allowed_serial_endpoint(endpoint);
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

RuntimeConfig load_runtime_config(RuntimeTarget target, TransportRole role) {
    const YamlValue settings = load_runtime_settings(runtime_target_name(target));
    const YamlValue transport = settings["transport"];
    const std::string role_key = role == TransportRole::CommandOwner
        ? "command_endpoint"
        : "telemetry_endpoint";

    RuntimeConfig config;
    config.target = target;
    config.role = role;
    config.serial_endpoint = transport.get_string_or("serial_endpoint", "");
    config.telemetry_endpoint = transport.get_string_or(
        "telemetry_endpoint", "udp:127.0.0.1:14551");
    config.gcs_telemetry_endpoint = transport.get_string_or(
        "gcs_telemetry_endpoint", "udp:127.0.0.1:14553");
    config.health_telemetry_endpoint = transport.get_string_or(
        "health_telemetry_endpoint", "udp:127.0.0.1:14554");
    config.endpoint = transport[role_key].as_string();
    if (role == TransportRole::CommandOwner) {
        config.telemetry_fanout_endpoints = {
            config.telemetry_endpoint,
            config.gcs_telemetry_endpoint,
            config.health_telemetry_endpoint};
    }

    if (target == RuntimeTarget::Real && role == TransportRole::CommandOwner) {
        const char* serial_override = std::getenv("ASTRODRONE_SERIAL_ENDPOINT");
        config.endpoint = serial_override != nullptr && *serial_override != '\0'
                              ? serial_override
                              : config.serial_endpoint;
    }
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

    // Fan-out consumers are receive-only. They must never send telemetry
    // configuration requests back to the adapter's UDP destination.
    if (role == TransportRole::TelemetrySubscriber) {
        config.commands_enabled = false;
        config.allow_mavlink_writes = false;
        config.allow_vehicle_commands = false;
        config.allow_arm = false;
        config.allow_telemetry_configuration = false;
    }

    if (target == RuntimeTarget::Real && role == TransportRole::CommandOwner) {
        if (config.serial_owner != "onboard") {
            throw std::runtime_error(
                "real runtime requires serial_owner=onboard; the adapter owns Pixhawk serial");
        }
        if (!is_allowed_serial_endpoint(config.endpoint)) {
            throw std::runtime_error(
                "real command owner requires /dev/serial/by-id/... serial endpoint");
        }
    }

    // Telemetry subscribers never open the Pixhawk serial device. They only
    // consume the adapter's loopback UDP fan-out.
    if (role == TransportRole::TelemetrySubscriber && is_serial_endpoint(config.endpoint)) {
        throw std::runtime_error(
            "telemetry subscriber cannot open a serial endpoint; configure a shared UDP telemetry endpoint");
    }

    if (!is_serial_endpoint(config.endpoint) && !is_loopback_network_endpoint(config.endpoint)) {
        throw std::runtime_error("runtime MAVLink network endpoint must use 127.0.0.1");
    }
    if (!is_loopback_network_endpoint(config.telemetry_endpoint) ||
        !is_loopback_network_endpoint(config.gcs_telemetry_endpoint) ||
        !is_loopback_network_endpoint(config.health_telemetry_endpoint)) {
        throw std::runtime_error("telemetry fan-out endpoints must use 127.0.0.1");
    }

    return config;
}


namespace {
bool loopback_endpoint(const std::string& endpoint) {
    const bool network = endpoint.rfind("udp:", 0) == 0 ||
                         endpoint.rfind("tcp:", 0) == 0;
    return network && endpoint.find("127.0.0.1:") != std::string::npos;
}
}

RealFlightDecision validate_real_flight(const RuntimeConfig& config,
                                        const RealFlightOptions& options) {
    if (config.target != RuntimeTarget::Real) {
        return {false, "real-flight guard requires target=real"};
    }
    if (config.command_mode != CommandMode::Flight) {
        return {false, "command mode is not flight"};
    }
    if (config.serial_owner != "onboard") {
        return {false, "real flight requires serial_owner=onboard"};
    }
    if (!config.commands_enabled || !config.allow_mavlink_writes ||
        !config.allow_vehicle_commands || !config.allow_arm) {
        return {false, "runtime policy does not enable all flight command permissions"};
    }
    if (!options.allow_arm) return {false, "--allow-arm is required"};
    if (!options.confirm_real_flight) return {false, "--confirm-real-flight is required"};
    if (!options.commands_enabled) return {false, "commands_enabled must be explicitly enabled"};
    if (!options.serial_owner_confirmed) {
        return {false, "onboard serial ownership is not confirmed"};
    }
    if (!serial_endpoint_is_allowed(options.serial_endpoint) ||
        options.serial_endpoint.find("REPLACE_WITH") != std::string::npos) {
        return {false, "serial must be an explicit /dev/serial/by-id path"};
    }
    if (config.endpoint != options.serial_endpoint) {
        return {false, "configured adapter serial and --connect path differ"};
    }
    if (!loopback_endpoint(options.telemetry_endpoint)) {
        return {false, "telemetry endpoint must use 127.0.0.1"};
    }
    return {true, "explicit real-flight policy accepted"};
}

}  // namespace app
