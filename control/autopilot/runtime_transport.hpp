#pragma once

#include <string>

namespace autopilot {

enum class RuntimeTarget {
    Sitl,
    Real,
};

enum class TransportRole {
    CommandOwner,
    TelemetrySubscriber,
};

enum class CommandMode {
    Observe,
    Shadow,
    Flight,
};

struct RuntimeTransportConfig {
    RuntimeTarget target;
    TransportRole role;
    std::string endpoint;
    std::string serial_endpoint;
    int baud = 115200;
    std::string serial_owner;
    std::string camera_source;
    CommandMode command_mode = CommandMode::Observe;
    bool commands_enabled = false;
    bool allow_mavlink_writes = false;
    bool allow_vehicle_commands = false;
    bool allow_arm = false;
    bool allow_telemetry_configuration = false;
    bool requires_allow_arm = false;
    bool requires_real_confirmation = false;
};

// Defaults to SITL. Real mode is opt-in through ASTRODRONE_TARGET=real.
RuntimeTarget runtime_target_from_environment();
const char* runtime_target_name(RuntimeTarget target);
const char* command_mode_name(CommandMode mode);
CommandMode command_mode_from_environment(RuntimeTarget target);
bool endpoint_is_serial(const std::string& endpoint);
RuntimeTransportConfig load_runtime_transport(RuntimeTarget target,
                                               TransportRole role);

}  // namespace autopilot
