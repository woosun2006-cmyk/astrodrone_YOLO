#include "../autopilot/autopilot_mavlink_adapter.hpp"
#include "../app/runtime_config.hpp"
#include "fake_transport.hpp"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <memory>

namespace {

bool check(bool value, const char* expression, int line) {
    if (value) return true;
    std::cerr << "line " << line << ": check failed: " << expression << '\n';
    return false;
}

#define CHECK(expression) \
    do { \
        if (!check(static_cast<bool>(expression), #expression, __LINE__)) return 1; \
    } while (false)

app::RuntimeConfig observe_config() {
    app::RuntimeConfig config{};
    config.target = app::RuntimeTarget::Sitl;
    config.role = app::TransportRole::TelemetrySubscriber;
    config.endpoint = "fake:health-check";
    config.command_mode = app::CommandMode::Observe;
    config.commands_enabled = false;
    config.allow_mavlink_writes = false;
    config.allow_vehicle_commands = false;
    config.allow_arm = false;
    config.allow_telemetry_configuration = false;
    return config;
}

mavlink_message_t heartbeat() {
    mavlink_message_t message{};
    mavlink_msg_heartbeat_pack(1, MAV_COMP_ID_AUTOPILOT1, &message,
                                MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_ARDUPILOTMEGA,
                                4, 0, MAV_STATE_ACTIVE);
    return message;
}

bool telemetry_snapshot_is_updated_without_transmit() {
    auto transport = std::make_unique<FakeTransport>();
    FakeTransport* raw = transport.get();
    transport->queue_incoming_message(heartbeat());

    mavlink_gps_raw_int_t gps{};
    gps.fix_type = 6;
    gps.satellites_visible = 12;
    gps.lat = 371234567;
    gps.lon = 1271234567;
    mavlink_message_t message{};
    mavlink_msg_gps_raw_int_encode(1, MAV_COMP_ID_AUTOPILOT1, &message, &gps);
    transport->queue_incoming_message(message);

    mavlink_global_position_int_t position{};
    position.relative_alt = 4980;
    mavlink_msg_global_position_int_encode(1, MAV_COMP_ID_AUTOPILOT1, &message, &position);
    transport->queue_incoming_message(message);

    mavlink_local_position_ned_t local{};
    local.x = 1.2F; local.y = -0.4F; local.z = -4.98F;
    local.vx = 0.12F; local.vy = 0.03F; local.vz = -0.20F;
    mavlink_msg_local_position_ned_encode(1, MAV_COMP_ID_AUTOPILOT1, &message, &local);
    transport->queue_incoming_message(message);

    mavlink_sys_status_t battery{};
    battery.voltage_battery = 16800;
    battery.current_battery = 420;
    battery.battery_remaining = 96;
    battery.onboard_control_sensors_health = MAV_SYS_STATUS_PREARM_CHECK;
    mavlink_msg_sys_status_encode(1, MAV_COMP_ID_AUTOPILOT1, &message, &battery);
    transport->queue_incoming_message(message);

    mavlink_ekf_status_report_t ekf{};
    ekf.flags = 0x1F;
    ekf.pos_horiz_variance = 0.1F;
    ekf.pos_vert_variance = 0.2F;
    ekf.velocity_variance = 0.3F;
    mavlink_msg_ekf_status_report_encode(1, MAV_COMP_ID_AUTOPILOT1, &message, &ekf);
    transport->queue_incoming_message(message);

    mavlink_rc_channels_t rc{};
    rc.chancount = 8;
    rc.chan1_raw = 1500;
    rc.rssi = 92;
    mavlink_msg_rc_channels_encode(1, MAV_COMP_ID_AUTOPILOT1, &message, &rc);
    transport->queue_incoming_message(message);

    mavlink_statustext_t status{};
    status.severity = MAV_SEVERITY_INFO;
    std::snprintf(status.text, sizeof(status.text), "%s", "health test status");
    mavlink_msg_statustext_encode(1, MAV_COMP_ID_AUTOPILOT1, &message, &status);
    transport->queue_incoming_message(message);

    autopilot::AutopilotMavlinkAdapter adapter(
        std::make_unique<MavConnection>(std::move(transport)), observe_config());
    CHECK(adapter.poll(0.1));
    const auto& state = adapter.state();
    CHECK(state.heartbeat_system_id == 1);
    CHECK(state.heartbeat_component_id == MAV_COMP_ID_AUTOPILOT1);
    CHECK(state.have_gps && state.fix_type == 6 && state.satellites == 12);
    CHECK(state.have_altitude && std::abs(state.altitude_m - 4.98) < 1e-6);
    CHECK(state.have_local_position && std::abs(state.local_vz_mps + 0.20) < 1e-6);
    CHECK(state.have_battery && state.battery_valid);
    CHECK(std::abs(state.battery_voltage_v - 16.8) < 1e-6);
    CHECK(std::abs(state.battery_current_a - 4.2) < 1e-6);
    CHECK(state.have_rc && state.rc_channel_count == 8 && state.rc_rssi == 92);
    CHECK(state.have_ekf && state.ekf_flags == 0x1F);
    CHECK(state.have_status_text && state.last_status_text == "health test status");
    CHECK(adapter.tx_packet_count() == 0);
    CHECK(raw->write_call_count() == 0);
    return true;
}

bool stale_and_serial_policy_are_observable() {
    CHECK(!app::serial_endpoint_is_allowed("/dev/ttyACM0"));
    CHECK(!app::serial_endpoint_is_allowed("/dev/ttyUSB0"));
    CHECK(app::serial_endpoint_is_allowed("/dev/serial/by-id/PIXHAWK"));

    autopilot::AutopilotState state{};
    const auto now = autopilot::AutopilotState::Clock::now();
    state.last_heartbeat = now - std::chrono::seconds(4);
    state.have_gps = true;
    state.gps_updated_at = now;
    CHECK(std::chrono::duration<double>(now - state.last_heartbeat).count() > 3.0);
    CHECK(state.have_gps);
    return true;
}

}  // namespace

int main() {
    if (!telemetry_snapshot_is_updated_without_transmit()) return 1;
    if (!stale_and_serial_policy_are_observable()) return 1;
    return 0;
}
