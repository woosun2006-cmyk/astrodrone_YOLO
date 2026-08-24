#pragma once

#include <string>
#include <vector>

namespace app {

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

struct RuntimeConfig {
    RuntimeTarget target;
    TransportRole role;
    std::string endpoint;
    std::string serial_endpoint;
    std::string telemetry_endpoint;
    std::string gcs_telemetry_endpoint;
    std::string health_telemetry_endpoint;
    std::vector<std::string> telemetry_fanout_endpoints;
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
bool serial_endpoint_is_allowed(const std::string& endpoint);
RuntimeConfig load_runtime_config(RuntimeTarget target,
                                               TransportRole role);


struct RealFlightOptions {
    std::string serial_endpoint;
    std::string telemetry_endpoint;
    bool allow_arm = false;
    bool confirm_real_flight = false;
    bool commands_enabled = false;
    bool serial_owner_confirmed = false;
};

struct RealFlightDecision {
    bool allowed = false;
    std::string reason;
};

RealFlightDecision validate_real_flight(const RuntimeConfig& config,
                                        const RealFlightOptions& options);

}  // namespace app
