#define main astrodrone_control_main_for_characterization
#include "../app/flight_mission_app.cpp"
#undef main

#include "../autopilot/autopilot_mavlink_adapter.hpp"
#include "test_mavlink_sender.hpp"
#include "fake_transport.hpp"

#include <cmath>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace {

constexpr uint8_t kTargetSystem = 1;
constexpr uint8_t kTargetComponent = MAV_COMP_ID_AUTOPILOT1;
constexpr uint8_t kSourceSystem = 255;
constexpr uint8_t kSourceComponent = 0;
// Velocity and yaw_rate are active; position, acceleration and yaw are ignored.
constexpr uint16_t kCurrentVelocityTypeMask = 0b0000011111000111;

FakeTransport* g_opened_transport = nullptr;

bool expect(bool condition, const char* expression, int line) {
    if (condition) return true;
    std::cerr << "line " << line << ": check failed: " << expression << std::endl;
    return false;
}

#define CHECK(expression) \
    do { \
        if (!expect(static_cast<bool>(expression), #expression, __LINE__)) return false; \
    } while (false)

bool near(float actual, float expected) {
    return std::fabs(actual - expected) < 0.0001F;
}

mavlink_message_t autopilot_heartbeat(uint8_t system_id = kTargetSystem) {
    mavlink_message_t message{};
    mavlink_msg_heartbeat_pack(system_id, kTargetComponent, &message, MAV_TYPE_QUADROTOR,
                               MAV_AUTOPILOT_ARDUPILOTMEGA, 0, 0, MAV_STATE_ACTIVE);
    return message;
}

mavlink_message_t sys_status(uint8_t system_id, uint8_t component_id) {
    mavlink_message_t message{};
    mavlink_msg_sys_status_pack(system_id, component_id, &message, 0, 0, 0, 0, 15000, -1, 50,
                                0, 0, 0, 0, 0, 0, 0, 0, 0);
    return message;
}

bool has_expected_header(const mavlink_message_t& message, uint32_t msgid) {
    return message.msgid == msgid && message.sysid == kSourceSystem &&
           message.compid == kSourceComponent;
}

bool has_expected_command_target(const mavlink_command_long_t& command, uint16_t command_id) {
    return command.target_system == kTargetSystem &&
           command.target_component == kTargetComponent && command.command == command_id &&
           command.confirmation == 0;
}

bool production_flight_commands_keep_current_fields_and_order() {
    mavlink_reset_channel_status(MAVLINK_COMM_0);
    MavConnection& connection = drone::connect("fake://characterization", 0.1);
    CHECK(g_opened_transport != nullptr);
    CHECK(connection.target_system() == kTargetSystem);
    CHECK(connection.target_component() == kTargetComponent);

    g_opened_transport->queue_incoming_message(sys_status(kTargetSystem, 42));
    g_opened_transport->queue_incoming_message(sys_status(77, kTargetComponent));
    mavlink_message_t incoming{};
    CHECK(connection.recv_match({MAVLINK_MSG_ID_SYS_STATUS}, incoming, 0.1));
    CHECK(connection.recv_match({MAVLINK_MSG_ID_SYS_STATUS}, incoming, 0.1));
    CHECK(connection.target_system() == kTargetSystem);
    CHECK(connection.target_component() == kTargetComponent);

    g_opened_transport->clear_written();
    CHECK(drone::set_mode("GUIDED"));
    drone::arm_disarm(true);
    drone::takeoff(6.5);
    drone::send_velocity(1.25, -2.5, 0.75, -0.2);
    drone::send_velocity_body(-1.5, 2.25, -0.5, 0.3);
    drone::send_velocity_body(0.0, 0.0, 0.0, 0.0);
    drone::land();
    drone::arm_disarm(false);

    CHECK(g_opened_transport->write_call_count() == 8);
    const std::vector<mavlink_message_t> messages =
        g_opened_transport->decode_written_messages();
    CHECK(messages.size() == 8);

    CHECK(has_expected_header(messages[0], MAVLINK_MSG_ID_SET_MODE));
    mavlink_set_mode_t set_mode{};
    mavlink_msg_set_mode_decode(&messages[0], &set_mode);
    CHECK(set_mode.target_system == kTargetSystem);
    CHECK(set_mode.base_mode == MAV_MODE_FLAG_CUSTOM_MODE_ENABLED);
    CHECK(set_mode.custom_mode == 4);

    CHECK(has_expected_header(messages[1], MAVLINK_MSG_ID_COMMAND_LONG));
    mavlink_command_long_t arm{};
    mavlink_msg_command_long_decode(&messages[1], &arm);
    CHECK(has_expected_command_target(arm, MAV_CMD_COMPONENT_ARM_DISARM));
    CHECK(near(arm.param1, 1.0F));
    CHECK(near(arm.param2, 0.0F));
    CHECK(near(arm.param7, 0.0F));

    CHECK(has_expected_header(messages[2], MAVLINK_MSG_ID_COMMAND_LONG));
    mavlink_command_long_t takeoff{};
    mavlink_msg_command_long_decode(&messages[2], &takeoff);
    CHECK(has_expected_command_target(takeoff, MAV_CMD_NAV_TAKEOFF));
    CHECK(near(takeoff.param1, 0.0F));
    CHECK(near(takeoff.param6, 0.0F));
    CHECK(near(takeoff.param7, 6.5F));

    CHECK(has_expected_header(messages[3], MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED));
    mavlink_set_position_target_local_ned_t local_velocity{};
    mavlink_msg_set_position_target_local_ned_decode(&messages[3], &local_velocity);
    CHECK(local_velocity.target_system == kTargetSystem);
    CHECK(local_velocity.target_component == kTargetComponent);
    CHECK(local_velocity.coordinate_frame == MAV_FRAME_LOCAL_NED);
    CHECK(local_velocity.type_mask == kCurrentVelocityTypeMask);
    CHECK(local_velocity.time_boot_ms == 0);
    CHECK(near(local_velocity.vx, 1.25F));
    CHECK(near(local_velocity.vy, -2.5F));
    CHECK(near(local_velocity.vz, 0.75F));
    CHECK(near(local_velocity.yaw_rate, -0.2F));
    CHECK(near(local_velocity.x, 0.0F));
    CHECK(near(local_velocity.afx, 0.0F));

    CHECK(has_expected_header(messages[4], MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED));
    mavlink_set_position_target_local_ned_t body_velocity{};
    mavlink_msg_set_position_target_local_ned_decode(&messages[4], &body_velocity);
    CHECK(body_velocity.target_system == kTargetSystem);
    CHECK(body_velocity.target_component == kTargetComponent);
    CHECK(body_velocity.coordinate_frame == MAV_FRAME_BODY_OFFSET_NED);
    CHECK(body_velocity.type_mask == kCurrentVelocityTypeMask);
    CHECK(near(body_velocity.vx, -1.5F));
    CHECK(near(body_velocity.vy, 2.25F));
    CHECK(near(body_velocity.vz, -0.5F));
    CHECK(near(body_velocity.yaw_rate, 0.3F));

    CHECK(has_expected_header(messages[5], MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED));
    mavlink_set_position_target_local_ned_t zero_velocity{};
    mavlink_msg_set_position_target_local_ned_decode(&messages[5], &zero_velocity);
    CHECK(zero_velocity.target_system == kTargetSystem);
    CHECK(zero_velocity.target_component == kTargetComponent);
    CHECK(zero_velocity.coordinate_frame == MAV_FRAME_BODY_OFFSET_NED);
    CHECK(zero_velocity.type_mask == kCurrentVelocityTypeMask);
    CHECK(near(zero_velocity.vx, 0.0F));
    CHECK(near(zero_velocity.vy, 0.0F));
    CHECK(near(zero_velocity.vz, 0.0F));
    CHECK(near(zero_velocity.yaw_rate, 0.0F));

    CHECK(has_expected_header(messages[6], MAVLINK_MSG_ID_COMMAND_LONG));
    mavlink_command_long_t land{};
    mavlink_msg_command_long_decode(&messages[6], &land);
    CHECK(has_expected_command_target(land, MAV_CMD_NAV_LAND));
    CHECK(near(land.param1, 0.0F));
    CHECK(near(land.param7, 0.0F));

    CHECK(has_expected_header(messages[7], MAVLINK_MSG_ID_COMMAND_LONG));
    mavlink_command_long_t disarm{};
    mavlink_msg_command_long_decode(&messages[7], &disarm);
    CHECK(has_expected_command_target(disarm, MAV_CMD_COMPONENT_ARM_DISARM));
    CHECK(near(disarm.param1, 0.0F));
    CHECK(near(disarm.param2, 0.0F));
    CHECK(near(disarm.param7, 0.0F));
    return true;
}

bool control_message_interval_keeps_current_fields() {
    mavlink_reset_channel_status(MAVLINK_COMM_0);
    auto transport = std::make_unique<FakeTransport>();
    FakeTransport* fake = transport.get();
    transport->queue_incoming_message(autopilot_heartbeat());
    app::RuntimeConfig config{
        app::RuntimeTarget::Sitl,
        app::TransportRole::CommandOwner,
        "fake:command-characterization"};
    config.allow_telemetry_configuration = true;
    autopilot::AutopilotMavlinkAdapter adapter(
        std::make_unique<MavConnection>(std::move(transport)), config);
    CHECK(adapter.wait_heartbeat(0.1));

    CHECK(adapter.request_message_interval(MAVLINK_MSG_ID_SYS_STATUS, 5.0));
    CHECK(fake->write_call_count() == 1);
    const std::vector<mavlink_message_t> messages = fake->decode_written_messages();
    CHECK(messages.size() == 1);
    CHECK(has_expected_header(messages[0], MAVLINK_MSG_ID_COMMAND_LONG));

    mavlink_command_long_t request{};
    mavlink_msg_command_long_decode(&messages[0], &request);
    CHECK(has_expected_command_target(request, MAV_CMD_SET_MESSAGE_INTERVAL));
    CHECK(near(request.param1, static_cast<float>(MAVLINK_MSG_ID_SYS_STATUS)));
    CHECK(near(request.param2, 200000.0F));
    CHECK(near(request.param3, 0.0F));
    CHECK(near(request.param4, 0.0F));
    CHECK(near(request.param5, 0.0F));
    CHECK(near(request.param6, 0.0F));
    CHECK(near(request.param7, 0.0F));
    return true;
}

bool command_sender_keeps_existing_packet_order() {
    mavlink_reset_channel_status(MAVLINK_COMM_0);
    MavConnection& connection = drone::connect("fake://command-sender", 0.1);
    CHECK(g_opened_transport != nullptr);
    g_opened_transport->clear_written();

    safety::SafetyMonitor monitor;
    test::TestPacketSender sender(connection, monitor);
    CHECK(sender.set_mode("GUIDED"));
    CHECK(sender.arm_disarm(true));
    CHECK(sender.takeoff(6.5));
    CHECK(sender.send_velocity(1.25, -2.5, 0.75, -0.2));
    CHECK(sender.send_velocity_body(-1.5, 2.25, -0.5, 0.3));
    CHECK(sender.send_zero_velocity());
    CHECK(sender.land());
    CHECK(sender.arm_disarm(false));

    const std::vector<mavlink_message_t> messages =
        g_opened_transport->decode_written_messages();
    CHECK(messages.size() == 8);
    CHECK(messages[0].msgid == MAVLINK_MSG_ID_SET_MODE);
    CHECK(messages[1].msgid == MAVLINK_MSG_ID_COMMAND_LONG);
    CHECK(messages[2].msgid == MAVLINK_MSG_ID_COMMAND_LONG);
    CHECK(messages[3].msgid == MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED);
    CHECK(messages[4].msgid == MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED);
    CHECK(messages[5].msgid == MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED);
    CHECK(messages[6].msgid == MAVLINK_MSG_ID_COMMAND_LONG);
    CHECK(messages[7].msgid == MAVLINK_MSG_ID_COMMAND_LONG);

    mavlink_set_position_target_local_ned_t local_velocity{};
    mavlink_msg_set_position_target_local_ned_decode(&messages[3], &local_velocity);
    CHECK(local_velocity.coordinate_frame == MAV_FRAME_LOCAL_NED);
    CHECK(local_velocity.type_mask == kCurrentVelocityTypeMask);
    CHECK(near(local_velocity.vx, 1.25F));
    CHECK(near(local_velocity.vy, -2.5F));
    CHECK(near(local_velocity.vz, 0.75F));
    CHECK(near(local_velocity.yaw_rate, -0.2F));

    mavlink_set_position_target_local_ned_t body_velocity{};
    mavlink_msg_set_position_target_local_ned_decode(&messages[4], &body_velocity);
    CHECK(body_velocity.coordinate_frame == MAV_FRAME_BODY_OFFSET_NED);
    CHECK(body_velocity.type_mask == kCurrentVelocityTypeMask);
    CHECK(near(body_velocity.vx, -1.5F));
    CHECK(near(body_velocity.vy, 2.25F));
    CHECK(near(body_velocity.vz, -0.5F));
    CHECK(near(body_velocity.yaw_rate, 0.3F));

    mavlink_set_position_target_local_ned_t zero_velocity{};
    mavlink_msg_set_position_target_local_ned_decode(&messages[5], &zero_velocity);
    CHECK(zero_velocity.coordinate_frame == MAV_FRAME_BODY_OFFSET_NED);
    CHECK(zero_velocity.type_mask == kCurrentVelocityTypeMask);
    CHECK(near(zero_velocity.vx, 0.0F));
    CHECK(near(zero_velocity.vy, 0.0F));
    CHECK(near(zero_velocity.vz, 0.0F));
    CHECK(near(zero_velocity.yaw_rate, 0.0F));
    return true;
}

bool locked_authority_blocks_every_vehicle_command() {
    mavlink_reset_channel_status(MAVLINK_COMM_0);
    MavConnection& connection = drone::connect("fake://locked-command-sender", 0.1);
    CHECK(g_opened_transport != nullptr);
    g_opened_transport->clear_written();

    std::vector<safety::CommandDecision> decisions;
    safety::SafetyMonitor monitor;
    monitor.set_control_locked(true);
    test::TestPacketSender sender(
        connection, monitor,
        [&](const safety::CommandDecision& decision) { decisions.push_back(decision); });

    const auto mode = sender.set_mode("GUIDED");
    const auto arm = sender.arm_disarm(true);
    const auto takeoff = sender.takeoff(5.0);
    const auto velocity = sender.send_velocity_body(1.0, 0.0, 0.0, 0.0);
    const auto zero = sender.send_zero_velocity();
    const auto land = sender.land();

    for (const auto& decision : {mode, arm, takeoff, velocity, zero, land}) {
        CHECK(!decision.allowed);
        CHECK(!decision.sent);
        CHECK(decision.block_reason == safety::GateBlockReason::ControlLocked);
    }
    CHECK(g_opened_transport->write_call_count() == 0);
    CHECK(decisions.size() == 6);
    CHECK(decisions[4].type == safety::CommandType::VelocitySetpoint);
    CHECK(decisions[5].type == safety::CommandType::Land);
    return true;
}

bool authority_change_and_expiry_are_checked_before_send() {
    mavlink_reset_channel_status(MAVLINK_COMM_0);
    MavConnection& connection = drone::connect("fake://authority-change", 0.1);
    CHECK(g_opened_transport != nullptr);
    g_opened_transport->clear_written();

    safety::SafetyMonitor monitor;
    test::TestPacketSender sender(connection, monitor);
    CHECK(sender.set_mode("GUIDED"));
    CHECK(g_opened_transport->write_call_count() == 1);

    monitor.set_control_locked(true);
    const auto changed = sender.land();
    CHECK(!changed.allowed);
    CHECK(changed.block_reason == safety::GateBlockReason::ControlLocked);
    CHECK(g_opened_transport->write_call_count() == 1);

    monitor.set_control_locked(false);
    const auto now = safety::CommandRequest::Clock::now();
    const auto expired = safety::CommandRequest::zero_velocity(
        now - std::chrono::seconds(2), now - std::chrono::milliseconds(1));
    const auto stale = sender.send(expired);
    CHECK(!stale.allowed);
    CHECK(stale.block_reason == safety::GateBlockReason::RequestExpired);
    CHECK(g_opened_transport->write_call_count() == 1);
    return true;
}

}  // namespace

std::unique_ptr<Transport> open_transport(const std::string& address, int baud) {
    (void)address;
    (void)baud;
    auto transport = std::make_unique<FakeTransport>();
    g_opened_transport = transport.get();
    transport->queue_incoming_message(autopilot_heartbeat());
    return transport;
}

int main() {
    const std::vector<std::pair<const char*, std::function<bool()>>> tests = {
        {"production_flight_commands_keep_current_fields_and_order",
         production_flight_commands_keep_current_fields_and_order},
        {"control_message_interval_keeps_current_fields",
         control_message_interval_keeps_current_fields},
        {"command_sender_keeps_existing_packet_order",
         command_sender_keeps_existing_packet_order},
        {"locked_authority_blocks_every_vehicle_command",
         locked_authority_blocks_every_vehicle_command},
        {"authority_change_and_expiry_are_checked_before_send",
         authority_change_and_expiry_are_checked_before_send},
    };

    int failed = 0;
    for (const auto& test : tests) {
        if (test.second()) {
            std::cout << "PASS " << test.first << std::endl;
        } else {
            std::cerr << "FAIL " << test.first << std::endl;
            ++failed;
        }
    }
    return failed == 0 ? 0 : 1;
}
