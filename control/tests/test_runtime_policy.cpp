#include "../autopilot/command_gate.hpp"
#include "../autopilot/real_flight_guard.hpp"
#include "../autopilot/runtime_transport.hpp"

#include <cstdlib>
#include <iostream>
#include <string>

namespace {

bool expect(bool condition, const char* expression, int line) {
    if (condition) return true;
    std::cerr << "line " << line << ": check failed: " << expression << std::endl;
    return false;
}

#define CHECK(expression) \
    do { \
        if (!expect(static_cast<bool>(expression), #expression, __LINE__)) return false; \
    } while (false)

void set_mode(const char* value) {
    if (value == nullptr) {
        unsetenv("ASTRODRONE_COMMAND_MODE");
    } else {
        setenv("ASTRODRONE_COMMAND_MODE", value, 1);
    }
}

bool runtime_yaml_and_shadow_policy() {
    setenv("ASTRODRONE_SETTINGS_DIR", std::string(ASTRODRONE_REPO_ROOT).append("/setting").c_str(), 1);
    set_mode("observe");
    auto real = autopilot::load_runtime_transport(
        autopilot::RuntimeTarget::Real, autopilot::TransportRole::TelemetrySubscriber);
    CHECK(real.command_mode == autopilot::CommandMode::Observe);
    CHECK(!real.commands_enabled);
    CHECK(!real.allow_mavlink_writes);
    CHECK(!real.allow_vehicle_commands);
    CHECK(!real.allow_telemetry_configuration);
    CHECK(real.endpoint == "udp:127.0.0.1:14551");

    set_mode("shadow");
    auto shadow = autopilot::load_runtime_transport(
        autopilot::RuntimeTarget::Real, autopilot::TransportRole::CommandOwner);
    CHECK(shadow.command_mode == autopilot::CommandMode::Shadow);
    CHECK(!shadow.allow_vehicle_commands);
    CHECK(!shadow.allow_mavlink_writes);

    autopilot::CommandGate gate;
    autopilot::CommandAuthority authority;
    authority.vehicle_commands_enabled = false;
    gate.set_authority(authority);
    bool callback_called = false;
    auto decision = gate.authorize_and_run(
        autopilot::CommandRequest::zero_velocity(), [&]() {
            callback_called = true;
            return true;
        });
    CHECK(!decision.allowed);
    CHECK(decision.block_reason == autopilot::GateBlockReason::CommandModeDisabled);
    CHECK(!callback_called);
    for (const auto& request : {
             autopilot::CommandRequest::set_mode("GUIDED"),
             autopilot::CommandRequest::arm_disarm(true),
             autopilot::CommandRequest::takeoff(5.0),
             autopilot::CommandRequest::velocity_body(0.2, 0.0, 0.2, 0.0),
             autopilot::CommandRequest::zero_velocity(),
             autopilot::CommandRequest::land(),
         }) {
        const auto blocked = gate.authorize_and_run(request, [&]() {
            callback_called = true;
            return true;
        });
        CHECK(!blocked.allowed);
        CHECK(!blocked.sent);
        CHECK(blocked.block_reason == autopilot::GateBlockReason::CommandModeDisabled);
    }
    CHECK(!callback_called);
    return true;
}

bool real_flight_requires_both_confirmations() {
    set_mode("flight");
    setenv("ASTRODRONE_ALLOW_MAVLINK_WRITES", "1", 1);
    setenv("ASTRODRONE_ALLOW_VEHICLE_COMMANDS", "1", 1);
    setenv("ASTRODRONE_ALLOW_ARM", "1", 1);
    auto config = autopilot::load_runtime_transport(
        autopilot::RuntimeTarget::Real, autopilot::TransportRole::CommandOwner);
    autopilot::RealFlightOptions options;
    options.router_serial = "/dev/serial/by-id/fake-pixhawk";
    options.command_endpoint = "udp:127.0.0.1:14550";
    options.telemetry_endpoint = "udp:127.0.0.1:14551";

    auto denied = autopilot::validate_real_flight(config, options);
    CHECK(!denied.allowed);
    options.allow_arm = true;
    denied = autopilot::validate_real_flight(config, options);
    CHECK(!denied.allowed);
    options.confirm_real_flight = true;
    options.commands_enabled = true;
    options.router_ownership_confirmed = true;
    auto accepted = autopilot::validate_real_flight(config, options);
    CHECK(accepted.allowed);

    options.commands_enabled = false;
    CHECK(!autopilot::validate_real_flight(config, options).allowed);
    options.commands_enabled = true;
    options.router_ownership_confirmed = false;
    CHECK(!autopilot::validate_real_flight(config, options).allowed);

    options.command_endpoint = "udp:192.168.0.10:14550";
    CHECK(!autopilot::validate_real_flight(config, options).allowed);
    return true;
}

}  // namespace

int main() {
    if (!runtime_yaml_and_shadow_policy()) return 1;
    if (!real_flight_requires_both_confirmations()) return 1;
    std::cout << "runtime policy tests passed" << std::endl;
    return 0;
}
