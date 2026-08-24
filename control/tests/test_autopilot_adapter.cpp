#include "../autopilot/autopilot_mavlink_adapter.hpp"
#include "fake_transport.hpp"

#include <iostream>
#include <memory>
#include <cstring>
#include <cmath>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

namespace {

bool check(bool value, const char* expression, int line) {
    if (value) return true;
    std::cerr << "line " << line << ": check failed: " << expression << std::endl;
    return false;
}

#define CHECK(expression) \
    do { \
        if (!check(static_cast<bool>(expression), #expression, __LINE__)) return 1; \
    } while (false)

mavlink_message_t heartbeat() {
    mavlink_message_t message{};
    mavlink_msg_heartbeat_pack(
        1, MAV_COMP_ID_AUTOPILOT1, &message, MAV_TYPE_QUADROTOR,
        MAV_AUTOPILOT_ARDUPILOTMEGA, 0, 0, MAV_STATE_ACTIVE);
    return message;
}

bool telemetry_fanout_publishes_incoming_message() {
    const int receiver = ::socket(AF_INET, SOCK_DGRAM | SOCK_CLOEXEC, 0);
    const int health_receiver = ::socket(AF_INET, SOCK_DGRAM | SOCK_CLOEXEC, 0);
    if (receiver < 0 || health_receiver < 0) {
        if (receiver >= 0) ::close(receiver);
        if (health_receiver >= 0) ::close(health_receiver);
        return false;
    }

    auto bind_ephemeral = [](int fd, sockaddr_in& address) {
        address = {};
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        address.sin_port = htons(0);
        if (::bind(fd, reinterpret_cast<const sockaddr*>(&address), sizeof(address)) != 0) {
            return false;
        }
        socklen_t length = sizeof(address);
        return ::getsockname(fd, reinterpret_cast<sockaddr*>(&address), &length) == 0;
    };
    sockaddr_in bind_address{};
    sockaddr_in health_bind_address{};
    if (!bind_ephemeral(receiver, bind_address) ||
        !bind_ephemeral(health_receiver, health_bind_address)) {
        ::close(receiver);
        ::close(health_receiver);
        return false;
    }

    auto transport = std::make_unique<FakeTransport>();
    transport->queue_incoming_message(heartbeat());
    app::RuntimeConfig config{
        app::RuntimeTarget::Sitl,
        app::TransportRole::CommandOwner,
        "fake:fanout"};
    config.command_mode = app::CommandMode::Observe;
    config.allow_vehicle_commands = false;
    config.telemetry_fanout_endpoints = {
        "udp:127.0.0.1:" + std::to_string(ntohs(bind_address.sin_port)),
        "udp:127.0.0.1:" + std::to_string(ntohs(health_bind_address.sin_port))};

    autopilot::AutopilotMavlinkAdapter adapter(
        std::make_unique<MavConnection>(std::move(transport)), config);
    if (!adapter.wait_heartbeat(0.1)) {
        ::close(receiver);
        ::close(health_receiver);
        return false;
    }

    pollfd ready[2]{{receiver, POLLIN, 0}, {health_receiver, POLLIN, 0}};
    if (::poll(ready, 2, 100) <= 0 ||
        !(ready[0].revents & POLLIN) || !(ready[1].revents & POLLIN)) {
        ::close(receiver);
        ::close(health_receiver);
        return false;
    }
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN]{};
    const ssize_t length = ::recv(receiver, buffer, sizeof(buffer), 0);
    uint8_t health_buffer[MAVLINK_MAX_PACKET_LEN]{};
    const ssize_t health_length = ::recv(health_receiver, health_buffer,
                                         sizeof(health_buffer), 0);
    ::close(receiver);
    ::close(health_receiver);
    if (length <= 0 || health_length <= 0) return false;

    mavlink_reset_channel_status(MAVLINK_COMM_1);
    mavlink_message_t decoded{};
    mavlink_status_t status{};
    bool receiver_ok = false;
    for (ssize_t i = 0; i < length; ++i) {
        if (mavlink_parse_char(MAVLINK_COMM_1, buffer[i], &decoded, &status)) {
            receiver_ok = decoded.msgid == MAVLINK_MSG_ID_HEARTBEAT && decoded.sysid == 1;
            break;
        }
    }
    if (!receiver_ok) return false;
    mavlink_reset_channel_status(MAVLINK_COMM_2);
    mavlink_message_t health_decoded{};
    mavlink_status_t health_status{};
    for (ssize_t i = 0; i < health_length; ++i) {
        if (mavlink_parse_char(MAVLINK_COMM_2, health_buffer[i], &health_decoded,
                               &health_status)) {
            return health_decoded.msgid == MAVLINK_MSG_ID_HEARTBEAT && health_decoded.sysid == 1;
        }
    }
    return false;
}

}  // namespace

int main() {
    if (!check(telemetry_fanout_publishes_incoming_message(),
               "telemetry fan-out publishes incoming MAVLink", __LINE__)) return 1;
    auto transport = std::make_unique<FakeTransport>();
    FakeTransport* raw_transport = transport.get();
    transport->queue_incoming_message(heartbeat());

    app::RuntimeConfig config{
        app::RuntimeTarget::Sitl,
        app::TransportRole::CommandOwner,
        "fake:autopilot-adapter"};
    config.command_mode = app::CommandMode::Observe;
    config.commands_enabled = false;
    config.allow_vehicle_commands = false;
    config.allow_telemetry_configuration = false;

    safety::SafetyMonitor monitor;
    safety::CommandAuthority authority;
    authority.vehicle_commands_enabled = false;
    monitor.set_authority(authority);
    autopilot::AutopilotMavlinkAdapter adapter(
        std::make_unique<MavConnection>(std::move(transport)), config,
        [&](const safety::CommandRequest& request) {
            return monitor.authorize(request);
        });
    CHECK(adapter.wait_heartbeat(0.1));
    CHECK(adapter.state().last_heartbeat !=
          autopilot::AutopilotState::Clock::time_point{});

    const auto decision = adapter.arm_disarm(true);
    CHECK(!decision.allowed);
    CHECK(raw_transport->write_call_count() == 0);

    mavlink_sys_status_t sys_status{};
    sys_status.voltage_battery = 16800;
    sys_status.current_battery = 250;
    sys_status.battery_remaining = 82;
    sys_status.onboard_control_sensors_health = MAV_SYS_STATUS_PREARM_CHECK;
    mavlink_message_t message{};
    mavlink_msg_sys_status_encode(1, MAV_COMP_ID_AUTOPILOT1, &message, &sys_status);
    raw_transport->queue_incoming_message(message);
    CHECK(adapter.poll(0.1));
    CHECK(adapter.state().have_sys_status);
    CHECK(adapter.state().battery_valid);
    CHECK(adapter.state().battery_voltage_v == 16.8);
    CHECK(adapter.state().battery_percent == 82);
    CHECK(adapter.state().prearm_healthy);

    mavlink_local_position_ned_t local_position{};
    local_position.x = 1.0F;
    local_position.y = -2.0F;
    local_position.z = -3.0F;
    local_position.vx = 0.1F;
    local_position.vy = -0.2F;
    local_position.vz = 0.3F;
    mavlink_msg_local_position_ned_encode(
        1, MAV_COMP_ID_AUTOPILOT1, &message, &local_position);
    raw_transport->queue_incoming_message(message);
    CHECK(adapter.poll(0.1));
    CHECK(adapter.state().have_local_position);
    CHECK(adapter.state().local_x_m == 1.0);
    CHECK(std::fabs(adapter.state().local_vy_mps + 0.2) < 1e-5);

    mavlink_gps_raw_int_t gps{};
    gps.fix_type = 3;
    gps.satellites_visible = 12;
    gps.lat = 375000000;
    gps.lon = 1270000000;
    mavlink_msg_gps_raw_int_encode(42, MAV_COMP_ID_AUTOPILOT1, &message, &gps);
    raw_transport->queue_incoming_message(message);
    CHECK(adapter.poll(0.1));
    CHECK(!adapter.state().have_gps);

    // Only the autopilot system may update the unified vehicle state.
    mavlink_msg_gps_raw_int_encode(1, MAV_COMP_ID_AUTOPILOT1, &message, &gps);
    raw_transport->queue_incoming_message(message);
    CHECK(adapter.poll(0.1));
    CHECK(adapter.state().have_gps);
    CHECK(adapter.state().fix_type == 3);
    CHECK(adapter.state().satellites == 12);
    CHECK(adapter.state().have_position);

    mavlink_attitude_t attitude{};
    attitude.roll = 0.1F;
    attitude.pitch = -0.2F;
    attitude.yaw = 0.3F;
    mavlink_msg_attitude_encode(1, MAV_COMP_ID_AUTOPILOT1, &message, &attitude);
    raw_transport->queue_incoming_message(message);
    CHECK(adapter.poll(0.1));
    CHECK(adapter.state().have_attitude);
    CHECK(std::fabs(adapter.state().yaw_rad - 0.3) < 1e-5);

    mavlink_statustext_t status_text{};
    status_text.severity = MAV_SEVERITY_ERROR;
    std::strncpy(status_text.text, "PreArm: test diagnostic", sizeof(status_text.text));
    mavlink_msg_statustext_encode(
        1, MAV_COMP_ID_AUTOPILOT1, &message, &status_text);
    raw_transport->queue_incoming_message(message);
    CHECK(adapter.poll(0.1));
    CHECK(adapter.state().have_status_text);
    CHECK(adapter.state().last_status_text == "PreArm: test diagnostic");

    mavlink_command_ack_t ack{};
    ack.command = MAV_CMD_COMPONENT_ARM_DISARM;
    ack.result = MAV_RESULT_DENIED;
    ack.result_param2 = 9;
    mavlink_msg_command_ack_encode(1, MAV_COMP_ID_AUTOPILOT1, &message, &ack);
    raw_transport->queue_incoming_message(message);
    CHECK(adapter.poll(0.1));
    CHECK(adapter.state().have_command_ack);
    CHECK(adapter.state().last_command_ack_command == MAV_CMD_COMPONENT_ARM_DISARM);
    CHECK(adapter.state().last_command_ack_result == MAV_RESULT_DENIED);
    CHECK(adapter.state().last_command_ack_param2 == 9);

    return 0;
}
