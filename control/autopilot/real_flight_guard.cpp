#include "real_flight_guard.hpp"

namespace autopilot {
namespace {

bool loopback_endpoint(const std::string& endpoint) {
    const bool network = endpoint.rfind("udp:", 0) == 0 ||
                         endpoint.rfind("tcp:", 0) == 0;
    return network && endpoint.find("127.0.0.1:") != std::string::npos;
}

}  // namespace

RealFlightDecision validate_real_flight(const RuntimeTransportConfig& config,
                                        const RealFlightOptions& options) {
    if (config.target != RuntimeTarget::Real) {
        return {false, "real-flight guard requires target=real"};
    }
    if (config.command_mode != CommandMode::Flight) {
        return {false, "command mode is not flight"};
    }
    if (config.serial_owner != "external-router") {
        return {false, "real flight requires serial_owner=external-router"};
    }
    if (!config.commands_enabled || !config.allow_mavlink_writes ||
        !config.allow_vehicle_commands || !config.allow_arm) {
        return {false, "runtime policy does not enable all flight command permissions"};
    }
    if (!options.allow_arm) {
        return {false, "--allow-arm is required"};
    }
    if (!options.confirm_real_flight) {
        return {false, "--confirm-real-flight is required"};
    }
    if (!options.commands_enabled) {
        return {false, "commands_enabled must be explicitly enabled"};
    }
    if (!options.router_ownership_confirmed) {
        return {false, "external router serial ownership is not confirmed"};
    }
    if (options.router_serial.rfind("/dev/serial/by-id/", 0) != 0 ||
        options.router_serial.find("REPLACE_WITH") != std::string::npos) {
        return {false, "router serial must be an explicit /dev/serial/by-id path"};
    }
    if (!loopback_endpoint(options.command_endpoint) ||
        !loopback_endpoint(options.telemetry_endpoint)) {
        return {false, "command and telemetry endpoints must use 127.0.0.1"};
    }
    return {true, "explicit real-flight policy accepted"};
}

}  // namespace autopilot
