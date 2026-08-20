#include "../safety/health_monitor.hpp"

#include <chrono>
#include <iostream>

namespace {

bool check(bool condition, const char* label) {
    if (!condition) std::cerr << "check failed: " << label << '\n';
    return condition;
}

mavlink_message_t heartbeat_message() {
    mavlink_message_t message{};
    mavlink_msg_heartbeat_pack(1, MAV_COMP_ID_AUTOPILOT1, &message, MAV_TYPE_QUADROTOR,
                               MAV_AUTOPILOT_ARDUPILOTMEGA, MAV_MODE_FLAG_SAFETY_ARMED, 4,
                               MAV_STATE_ACTIVE);
    return message;
}

mavlink_message_t heartbeat_from(uint8_t system_id, uint8_t component_id,
                                 uint8_t autopilot, uint32_t custom_mode) {
    mavlink_message_t message{};
    mavlink_msg_heartbeat_pack(system_id, component_id, &message, MAV_TYPE_QUADROTOR,
                               autopilot, MAV_MODE_FLAG_SAFETY_ARMED, custom_mode,
                               MAV_STATE_ACTIVE);
    return message;
}

mavlink_message_t unavailable_battery_message() {
    mavlink_message_t message{};
    mavlink_sys_status_t status{};
    status.onboard_control_sensors_health = MAV_SYS_STATUS_PREARM_CHECK;
    status.voltage_battery = 0;
    status.battery_remaining = -1;
    mavlink_msg_sys_status_encode(1, MAV_COMP_ID_AUTOPILOT1, &message, &status);
    return message;
}

mavlink_message_t valid_battery_message() {
    mavlink_message_t message{};
    mavlink_sys_status_t status{};
    status.onboard_control_sensors_health = MAV_SYS_STATUS_PREARM_CHECK;
    status.voltage_battery = 16800;
    status.battery_remaining = 90;
    mavlink_msg_sys_status_encode(1, MAV_COMP_ID_AUTOPILOT1, &message, &status);
    return message;
}

}  // namespace

int main() {
    using Clock = std::chrono::steady_clock;
    const auto now = Clock::now();

    safety::HealthLimit limits;
    limits.require_prearm_healthy = false;
    safety::HealthMonitor monitor(limits);
    monitor.update(heartbeat_message(), now);
    monitor.update(unavailable_battery_message(), now);

    const safety::SafetyStatus status = monitor.status(now);
    if (!check(status.healthy, "unknown battery does not create a breach")) return 1;
    if (!check(monitor.state().armed, "armed state is recorded")) return 1;
    if (!check(monitor.state().mode_updated_at == now, "mode timestamp is recorded")) return 1;
    if (!check(monitor.state().battery_updated_at == now, "battery timestamp is recorded")) return 1;
    if (!check(!monitor.state().battery_valid,
               "sentinel battery values are not marked valid")) return 1;

    safety::HealthMonitor valid_monitor(limits);
    valid_monitor.update(valid_battery_message(), now);
    if (!check(valid_monitor.state().battery_valid,
               "usable battery values are marked valid")) return 1;

    const auto later = now + std::chrono::milliseconds(10);
    monitor.update(heartbeat_from(255, MAV_COMP_ID_AUTOPILOT1, MAV_AUTOPILOT_INVALID, 9), later);
    monitor.update(heartbeat_from(1, 42, MAV_AUTOPILOT_ARDUPILOTMEGA, 10), later);
    if (!check(monitor.state().custom_mode == 4,
               "GCS or other-component heartbeat does not overwrite vehicle mode")) return 1;
    if (!check(monitor.state().mode_updated_at == now,
               "invalid heartbeat does not refresh vehicle mode timestamp")) return 1;
    return 0;
}
