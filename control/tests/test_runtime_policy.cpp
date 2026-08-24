#include "../autopilot/autopilot_mavlink_adapter.hpp"
#include "../app/runtime_config.hpp"
#include "test_mavlink_sender.hpp"

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
    auto real = app::load_runtime_config(
        app::RuntimeTarget::Real, app::TransportRole::TelemetrySubscriber);
    CHECK(real.command_mode == app::CommandMode::Observe);
    CHECK(!real.commands_enabled);
    CHECK(!real.allow_mavlink_writes);
    CHECK(!real.allow_vehicle_commands);
    CHECK(!real.allow_telemetry_configuration);
    CHECK(real.endpoint == "udp:127.0.0.1:14551");

    set_mode("observe");
    auto real_owner = app::load_runtime_config(
        app::RuntimeTarget::Real, app::TransportRole::CommandOwner);
    CHECK(real_owner.serial_owner == "onboard");
    CHECK(real_owner.endpoint == "/dev/serial/by-id/REPLACE_WITH_PIXHAWK");
    CHECK(real_owner.telemetry_fanout_endpoints.size() == 3);

    set_mode("flight");
    auto sitl_owner = app::load_runtime_config(
        app::RuntimeTarget::Sitl, app::TransportRole::CommandOwner);
    CHECK(sitl_owner.endpoint == "tcp:127.0.0.1:5760");
    CHECK(sitl_owner.telemetry_fanout_endpoints.size() == 3);
    CHECK(sitl_owner.telemetry_fanout_endpoints[0] == "udp:127.0.0.1:14551");
    CHECK(sitl_owner.telemetry_fanout_endpoints[2] == "udp:127.0.0.1:14554");

    auto sitl_subscriber = app::load_runtime_config(
        app::RuntimeTarget::Sitl, app::TransportRole::TelemetrySubscriber);
    CHECK(!sitl_subscriber.allow_telemetry_configuration);

    set_mode("shadow");
    auto shadow = app::load_runtime_config(
        app::RuntimeTarget::Real, app::TransportRole::CommandOwner);
    CHECK(shadow.command_mode == app::CommandMode::Shadow);
    CHECK(!shadow.allow_vehicle_commands);
    CHECK(!shadow.allow_mavlink_writes);

    safety::SafetyMonitor monitor;
    safety::CommandAuthority authority;
    authority.vehicle_commands_enabled = false;
    monitor.set_authority(authority);
    bool callback_called = false;
    auto decision = monitor.authorize_and_run(
        safety::CommandRequest::zero_velocity(), [&]() {
            callback_called = true;
            return true;
        });
    CHECK(!decision.allowed);
    CHECK(decision.block_reason == safety::GateBlockReason::CommandModeDisabled);
    CHECK(!callback_called);
    for (const auto& request : {
             safety::CommandRequest::set_mode("GUIDED"),
             safety::CommandRequest::arm_disarm(true),
             safety::CommandRequest::takeoff(5.0),
             safety::CommandRequest::velocity_body(0.2, 0.0, 0.2, 0.0),
             safety::CommandRequest::zero_velocity(),
             safety::CommandRequest::land(),
         }) {
        const auto blocked = monitor.authorize_and_run(request, [&]() {
            callback_called = true;
            return true;
        });
        CHECK(!blocked.allowed);
        CHECK(!blocked.sent);
        CHECK(blocked.block_reason == safety::GateBlockReason::CommandModeDisabled);
    }
    CHECK(!callback_called);

    safety::CommandAuthority arm_disabled_authority;
    arm_disabled_authority.vehicle_commands_enabled = true;
    arm_disabled_authority.arm_commands_enabled = false;
    monitor.set_authority(arm_disabled_authority);
    const auto arm_blocked = monitor.authorize_and_run(
        safety::CommandRequest::arm_disarm(true), [&]() {
            callback_called = true;
            return true;
        });
    CHECK(!arm_blocked.allowed);
    CHECK(arm_blocked.block_reason == safety::GateBlockReason::AuthorityDisabled);
    CHECK(!callback_called);
    return true;
}

bool safety_monitor_command_api_tracks_preflight_and_lock() {
    safety::SafetyMonitor monitor;
    safety::CommandAuthority authority;
    authority.vehicle_commands_enabled = true;
    monitor.set_authority(authority);
    monitor.set_preflight_required(true);
    monitor.set_preflight_ready(false);

    CHECK(!monitor.can_arm());
    CHECK(!monitor.can_change_mode("GUIDED"));
    CHECK(!monitor.can_change_mode(""));
    CHECK(!monitor.can_send_velocity());
    CHECK(!monitor.can_takeoff());
    CHECK(!monitor.can_land());
    CHECK(!monitor.can_upload_mission());

    monitor.set_preflight_ready(true);
    CHECK(monitor.can_arm());
    CHECK(monitor.can_change_mode("GUIDED"));
    CHECK(monitor.can_send_velocity());
    CHECK(monitor.can_takeoff());
    CHECK(monitor.can_land());
    CHECK(monitor.can_upload_mission());

    monitor.set_real_flight_approved(false);
    CHECK(!monitor.can_arm());
    CHECK(!monitor.can_change_mode("GUIDED"));
    monitor.set_real_flight_approved(true);

    monitor.set_control_locked(true, "operator mode change");
    CHECK(!monitor.can_arm());
    CHECK(!monitor.can_change_mode("GUIDED"));
    CHECK(!monitor.can_send_velocity());
    CHECK(!monitor.can_takeoff());
    CHECK(!monitor.can_land());
    CHECK(!monitor.can_upload_mission());
    return true;
}

bool real_flight_requires_both_confirmations() {
    set_mode("flight");
    setenv("ASTRODRONE_SERIAL_ENDPOINT", "/dev/serial/by-id/fake-pixhawk", 1);
    setenv("ASTRODRONE_ALLOW_MAVLINK_WRITES", "1", 1);
    setenv("ASTRODRONE_ALLOW_VEHICLE_COMMANDS", "1", 1);
    setenv("ASTRODRONE_ALLOW_ARM", "1", 1);
    auto config = app::load_runtime_config(
        app::RuntimeTarget::Real, app::TransportRole::CommandOwner);
    app::RealFlightOptions options;
    options.serial_endpoint = "/dev/serial/by-id/fake-pixhawk";
    options.telemetry_endpoint = "udp:127.0.0.1:14551";

    auto denied = app::validate_real_flight(config, options);
    CHECK(!denied.allowed);
    options.allow_arm = true;
    denied = app::validate_real_flight(config, options);
    CHECK(!denied.allowed);
    options.confirm_real_flight = true;
    options.commands_enabled = true;
    options.serial_owner_confirmed = true;
    auto accepted = app::validate_real_flight(config, options);
    CHECK(accepted.allowed);

    options.commands_enabled = false;
    CHECK(!app::validate_real_flight(config, options).allowed);
    options.commands_enabled = true;
    options.serial_owner_confirmed = false;
    CHECK(!app::validate_real_flight(config, options).allowed);

    options.telemetry_endpoint = "udp:192.168.0.10:14551";
    CHECK(!app::validate_real_flight(config, options).allowed);
    unsetenv("ASTRODRONE_SERIAL_ENDPOINT");
    return true;
}

}  // namespace

int main() {
    if (!runtime_yaml_and_shadow_policy()) return 1;
    if (!safety_monitor_command_api_tracks_preflight_and_lock()) return 1;
    if (!real_flight_requires_both_confirmations()) return 1;
    std::cout << "runtime policy tests passed" << std::endl;
    return 0;
}
