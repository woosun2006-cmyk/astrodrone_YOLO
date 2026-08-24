#include "../mission/mission_setup.hpp"
#include "../autopilot/autopilot_mavlink_adapter.hpp"
#include "fake_transport.hpp"
#include "test_mavlink_sender.hpp"

#include <cmath>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

constexpr uint8_t kAutopilotSystem = 1;
constexpr uint8_t kAutopilotComponent = MAV_COMP_ID_AUTOPILOT1;
constexpr uint8_t kSourceSystem = 255;
constexpr uint8_t kSourceComponent = 0;
constexpr uint8_t kMissionType = MAV_MISSION_TYPE_MISSION;
constexpr int32_t kHomeLat = -353632621;
constexpr int32_t kHomeLon = 1491652374;
constexpr int32_t kTargetLat = -353631720;
constexpr int32_t kTargetLon = 1491652370;

bool check(bool condition, const char* expression, int line) {
    if (condition) return true;
    std::cerr << "line " << line << ": check failed: " << expression << '\n';
    return false;
}

#define CHECK(expression) \
    do { \
        if (!check(static_cast<bool>(expression), #expression, __LINE__)) return false; \
    } while (false)

mavlink_message_t heartbeat(uint32_t mode, bool armed = false) {
    mavlink_message_t message{};
    mavlink_msg_heartbeat_pack(
        kAutopilotSystem, kAutopilotComponent, &message, MAV_TYPE_QUADROTOR,
        MAV_AUTOPILOT_ARDUPILOTMEGA,
        armed ? MAV_MODE_FLAG_SAFETY_ARMED : 0, mode, MAV_STATE_ACTIVE);
    return message;
}

mavlink_message_t gps() {
    mavlink_message_t message{};
    mavlink_msg_gps_raw_int_pack(
        kAutopilotSystem, kAutopilotComponent, &message, 0, 3, kHomeLat,
        kHomeLon, 0, 0, 0, 0, 0, 10, 0, 0, 0, 0, 0, 0);
    return message;
}

mavlink_message_t mission_ack(uint8_t result) {
    mavlink_message_t message{};
    mavlink_msg_mission_ack_pack(
        kAutopilotSystem, kAutopilotComponent, &message, kSourceSystem,
        kSourceComponent, result, kMissionType, 0);
    return message;
}

mavlink_message_t mission_request(uint16_t seq) {
    mavlink_message_t message{};
    mavlink_msg_mission_request_int_pack(
        kAutopilotSystem, kAutopilotComponent, &message, kSourceSystem,
        kSourceComponent, seq, kMissionType);
    return message;
}

mavlink_message_t mission_count(uint16_t count) {
    mavlink_message_t message{};
    mavlink_msg_mission_count_pack(
        kAutopilotSystem, kAutopilotComponent, &message, kSourceSystem,
        kSourceComponent, count, kMissionType, 0);
    return message;
}

mavlink_message_t mission_item(uint16_t seq, int32_t x, int32_t y,
                               uint16_t command, float altitude) {
    mavlink_message_t message{};
    mavlink_msg_mission_item_int_pack(
        kAutopilotSystem, kAutopilotComponent, &message, kSourceSystem,
        kSourceComponent, seq, MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, command, 0,
        1, 0, 0, 0, 0, x, y, altitude, kMissionType);
    return message;
}

mavlink_message_t mission_current(uint16_t seq) {
    mavlink_message_t message{};
    mavlink_msg_mission_current_pack(
        kAutopilotSystem, kAutopilotComponent, &message, seq, 3, 0, 1, 0, 0,
        0);
    return message;
}

mavlink_message_t arm_ack(uint8_t result, int32_t result_param2 = 0) {
    mavlink_message_t message{};
    mavlink_msg_command_ack_pack(
        kAutopilotSystem, kAutopilotComponent, &message,
        MAV_CMD_COMPONENT_ARM_DISARM, result, 0, result_param2,
        kSourceSystem, kSourceComponent);
    return message;
}

mavlink_message_t status_text(const char* text) {
    mavlink_message_t message{};
    mavlink_msg_statustext_pack(
        kAutopilotSystem, kAutopilotComponent, &message, MAV_SEVERITY_ERROR,
        text, 0, 0);
    return message;
}

mavlink_message_t raw_imu(uint8_t id, int16_t xacc, int16_t yacc, int16_t zacc) {
    mavlink_message_t message{};
    mavlink_msg_raw_imu_pack(
        kAutopilotSystem, kAutopilotComponent, &message, 1000, xacc, yacc, zacc,
        0, 0, 0, 0, 0, 0, id, 2500);
    return message;
}

mavlink_message_t vibration(float x, float y, float z) {
    mavlink_message_t message{};
    mavlink_msg_vibration_pack(
        kAutopilotSystem, kAutopilotComponent, &message, 1000, x, y, z, 0, 0, 0);
    return message;
}

void queue_initial_setup(FakeTransport& transport) {
    // One heartbeat is consumed by the connection bootstrap; MissionSetup
    // then confirms the setup-stage vehicle state with a fresh heartbeat.
    transport.queue_incoming_message(heartbeat(copter_mode_mapping().at("LOITER")));
    transport.queue_incoming_message(heartbeat(copter_mode_mapping().at("LOITER")));
    transport.queue_incoming_message(gps());
}

void queue_setup_until_arm_result(FakeTransport& transport, uint8_t arm_result,
                                  bool armed_heartbeat) {
    queue_initial_setup(transport);
    transport.queue_incoming_message(mission_ack(MAV_MISSION_ACCEPTED));
    transport.queue_incoming_message(mission_request(0));
    transport.queue_incoming_message(mission_request(1));
    transport.queue_incoming_message(mission_request(2));
    transport.queue_incoming_message(mission_ack(MAV_MISSION_ACCEPTED));
    transport.queue_incoming_message(mission_count(3));
    transport.queue_incoming_message(mission_item(
        0, kHomeLat, kHomeLon, MAV_CMD_NAV_WAYPOINT, 0.0F));
    transport.queue_incoming_message(mission_item(
        1, kHomeLat, kHomeLon, MAV_CMD_NAV_TAKEOFF, 5.0F));
    transport.queue_incoming_message(mission_item(
        2, kTargetLat, kTargetLon, MAV_CMD_NAV_WAYPOINT, 5.0F));
    transport.queue_incoming_message(mission_current(0));
    transport.queue_incoming_message(heartbeat(copter_mode_mapping().at("AUTO")));
    transport.queue_incoming_message(raw_imu(0, 0, 0, 981));
    transport.queue_incoming_message(vibration(0.01F, 0.02F, 0.03F));
    transport.queue_incoming_message(raw_imu(0, 0, 0, 981));
    transport.queue_incoming_message(vibration(0.01F, 0.02F, 0.03F));
    transport.queue_incoming_message(raw_imu(0, 0, 0, 981));
    transport.queue_incoming_message(vibration(0.01F, 0.02F, 0.03F));
    transport.queue_incoming_message(raw_imu(0, 0, 0, 981));
    transport.queue_incoming_message(vibration(0.01F, 0.02F, 0.03F));
    transport.queue_incoming_message(raw_imu(0, 0, 0, 981));
    transport.queue_incoming_message(vibration(0.01F, 0.02F, 0.03F));
    transport.queue_incoming_message(arm_ack(arm_result, 17));
    transport.queue_incoming_message(status_text("PreArm: Accels inconsistent"));
    transport.queue_incoming_message(
        heartbeat(copter_mode_mapping().at("AUTO"), armed_heartbeat));
}

std::unique_ptr<autopilot::AutopilotMavlinkAdapter> mission_adapter(
    std::unique_ptr<FakeTransport> transport) {
    app::RuntimeConfig config{
        app::RuntimeTarget::Sitl,
        app::TransportRole::CommandOwner,
        "fake://mission-setup"};
    config.allow_vehicle_commands = true;
    return std::make_unique<autopilot::AutopilotMavlinkAdapter>(
        std::make_unique<MavConnection>(std::move(transport)), config);
}

bool mission_packets_use_central_sender_boundary() {
    auto transport = std::make_unique<FakeTransport>();
    FakeTransport* fake = transport.get();
    MavConnection connection(std::move(transport));
    fake->queue_incoming_message(heartbeat(copter_mode_mapping().at("LOITER")));
    CHECK(connection.wait_heartbeat(0.1));

    safety::SafetyMonitor monitor;
    test::TestPacketSender sender(connection, monitor);
    CHECK(sender.send_mission_clear());
    CHECK(sender.send_mission_count(3));
    CHECK(sender.send_mission_item(
        0, MAV_CMD_NAV_WAYPOINT, MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        kHomeLat, kHomeLon, 0.0F));
    CHECK(sender.send_mission_request_list());
    CHECK(sender.send_mission_request_item(1));
    CHECK(sender.send_mission_set_current(0));

    const auto messages = fake->decode_written_messages();
    CHECK(messages.size() == 6);
    CHECK(messages[0].msgid == MAVLINK_MSG_ID_MISSION_CLEAR_ALL);
    CHECK(messages[1].msgid == MAVLINK_MSG_ID_MISSION_COUNT);
    CHECK(messages[2].msgid == MAVLINK_MSG_ID_MISSION_ITEM_INT);
    CHECK(messages[3].msgid == MAVLINK_MSG_ID_MISSION_REQUEST_LIST);
    CHECK(messages[4].msgid == MAVLINK_MSG_ID_MISSION_REQUEST_INT);
    CHECK(messages[5].msgid == MAVLINK_MSG_ID_MISSION_SET_CURRENT);
    for (const auto& message : messages) {
        CHECK(message.sysid == kSourceSystem);
        CHECK(message.compid == kSourceComponent);
    }

    mavlink_mission_count_t count{};
    mavlink_msg_mission_count_decode(&messages[1], &count);
    CHECK(count.target_system == kAutopilotSystem);
    CHECK(count.target_component == kAutopilotComponent);
    CHECK(count.count == 3);
    CHECK(count.mission_type == kMissionType);

    mavlink_mission_item_int_t item{};
    mavlink_msg_mission_item_int_decode(&messages[2], &item);
    CHECK(item.seq == 0);
    CHECK(item.x == kHomeLat);
    CHECK(item.y == kHomeLon);
    return true;
}

bool preflight_blocks_mission_protocol() {
    auto transport = std::make_unique<FakeTransport>();
    FakeTransport* fake = transport.get();
    MavConnection connection(std::move(transport));
    fake->queue_incoming_message(heartbeat(copter_mode_mapping().at("LOITER")));
    CHECK(connection.wait_heartbeat(0.1));

    safety::SafetyMonitor monitor;
    monitor.set_preflight_required(true);
    monitor.set_preflight_ready(false);
    test::TestPacketSender sender(connection, monitor);

    const auto clear = sender.send_mission_clear();
    const auto count = sender.send_mission_count(3);
    const auto item = sender.send_mission_item(
        0, MAV_CMD_NAV_WAYPOINT, MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        kHomeLat, kHomeLon, 0.0F);
    const auto request_list = sender.send_mission_request_list();
    const auto request_item = sender.send_mission_request_item(0);
    const auto current = sender.send_mission_set_current(0);
    for (const auto& decision :
         {clear, count, item, request_list, request_item, current}) {
        CHECK(!decision.allowed);
        CHECK(!decision.sent);
        CHECK(decision.block_reason == safety::GateBlockReason::PreflightNotReady);
    }
    CHECK(fake->write_call_count() == 0);
    return true;
}

bool mission_setup_rejects_upload_ack() {
    auto transport = std::make_unique<FakeTransport>();
    FakeTransport* fake = transport.get();
    queue_initial_setup(*fake);
    fake->queue_incoming_message(mission_ack(MAV_MISSION_ERROR));
    auto adapter = mission_adapter(std::move(transport));
    CHECK(adapter->wait_heartbeat(0.1));
    bool rejected = false;
    try {
        mission::MissionSetup setup(*adapter);
        setup.run();
    } catch (const std::runtime_error& error) {
        rejected = std::string(error.what()).find("rejected") != std::string::npos;
    }
    CHECK(rejected);
    return true;
}

bool mission_setup_rejects_readback_mismatch() {
    auto transport = std::make_unique<FakeTransport>();
    FakeTransport* fake = transport.get();
    queue_initial_setup(*fake);
    fake->queue_incoming_message(mission_ack(MAV_MISSION_ACCEPTED));
    fake->queue_incoming_message(mission_request(0));
    fake->queue_incoming_message(mission_request(1));
    fake->queue_incoming_message(mission_request(2));
    fake->queue_incoming_message(mission_ack(MAV_MISSION_ACCEPTED));
    fake->queue_incoming_message(mission_count(3));
    fake->queue_incoming_message(mission_item(
        0, kHomeLat, kHomeLon, MAV_CMD_NAV_WAYPOINT, 0.0F));
    fake->queue_incoming_message(mission_item(
        1, kHomeLat, kHomeLon, MAV_CMD_NAV_TAKEOFF, 5.0F));
    fake->queue_incoming_message(mission_item(
        2, kHomeLat + 1, kTargetLon, MAV_CMD_NAV_WAYPOINT, 5.0F));
    auto adapter = mission_adapter(std::move(transport));
    CHECK(adapter->wait_heartbeat(0.1));
    bool rejected = false;
    try {
        mission::MissionSetup setup(*adapter);
        setup.run();
    } catch (const std::runtime_error& error) {
        rejected = std::string(error.what()).find("read-back") != std::string::npos;
    }
    CHECK(rejected);
    return true;
}

bool arm_failure_preserves_prearm_diagnostics() {
    auto transport = std::make_unique<FakeTransport>();
    FakeTransport* fake = transport.get();
    queue_setup_until_arm_result(*fake, MAV_RESULT_FAILED, false);
    auto adapter = mission_adapter(std::move(transport));
    CHECK(adapter->wait_heartbeat(0.1));
    bool rejected = false;
    std::string message;
    try {
        mission::MissionSetupConfig config;
        config.timeout_sec = 2.0;
        mission::MissionSetup setup(*adapter, config);
        setup.run();
    } catch (const std::runtime_error& error) {
        message = error.what();
        rejected = message.find("result=FAILED") != std::string::npos &&
                   message.find("Accels inconsistent") != std::string::npos;
    }
    CHECK(rejected);
    return true;
}

bool arm_success_requires_armed_heartbeat() {
    auto transport = std::make_unique<FakeTransport>();
    FakeTransport* fake = transport.get();
    queue_setup_until_arm_result(*fake, MAV_RESULT_ACCEPTED, true);
    auto adapter = mission_adapter(std::move(transport));
    CHECK(adapter->wait_heartbeat(0.1));
    bool succeeded = true;
    try {
        mission::MissionSetupConfig config;
        config.timeout_sec = 2.0;
        mission::MissionSetup setup(*adapter, config);
        succeeded = setup.run();
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        succeeded = false;
    }
    CHECK(succeeded);
    return true;
}

bool raw_imu_stability_gate_blocks_arm() {
    auto transport = std::make_unique<FakeTransport>();
    FakeTransport* fake = transport.get();
    queue_initial_setup(*fake);
    fake->queue_incoming_message(mission_ack(MAV_MISSION_ACCEPTED));
    fake->queue_incoming_message(mission_request(0));
    fake->queue_incoming_message(mission_request(1));
    fake->queue_incoming_message(mission_request(2));
    fake->queue_incoming_message(mission_ack(MAV_MISSION_ACCEPTED));
    fake->queue_incoming_message(mission_count(3));
    fake->queue_incoming_message(mission_item(0, kHomeLat, kHomeLon,
                                              MAV_CMD_NAV_WAYPOINT, 0.0F));
    fake->queue_incoming_message(mission_item(1, kHomeLat, kHomeLon,
                                              MAV_CMD_NAV_TAKEOFF, 5.0F));
    fake->queue_incoming_message(mission_item(2, kTargetLat, kTargetLon,
                                              MAV_CMD_NAV_WAYPOINT, 5.0F));
    fake->queue_incoming_message(mission_current(0));
    fake->queue_incoming_message(heartbeat(copter_mode_mapping().at("AUTO")));
    auto adapter = mission_adapter(std::move(transport));
    CHECK(adapter->wait_heartbeat(0.1));
    bool rejected = false;
    try {
        mission::MissionSetupConfig config;
        config.timeout_sec = 0.1;
        config.require_raw_imu_stable = true;
        mission::MissionSetup setup(*adapter, config);
        setup.run();
    } catch (const std::runtime_error& error) {
        rejected = std::string(error.what()).find("raw IMU") != std::string::npos;
    }
    CHECK(rejected);
    for (const auto& message : fake->decode_written_messages()) {
        if (message.msgid != MAVLINK_MSG_ID_COMMAND_LONG) continue;
        mavlink_command_long_t command{};
        mavlink_msg_command_long_decode(&message, &command);
        CHECK(command.command != MAV_CMD_COMPONENT_ARM_DISARM);
    }
    return true;
}

bool sitl_raw_imu_sample_floor_is_applied() {
    auto transport = std::make_unique<FakeTransport>();
    FakeTransport* fake = transport.get();
    queue_setup_until_arm_result(*fake, MAV_RESULT_ACCEPTED, true);
    auto adapter = mission_adapter(std::move(transport));
    CHECK(adapter->wait_heartbeat(0.1));

    setenv("SITL_REQUIRE_RAW_IMU_STABLE", "1", 1);
    setenv("SITL_RAW_IMU_MIN_SAMPLES", "6", 1);
    bool rejected = false;
    try {
        mission::MissionSetupConfig config;
        config.timeout_sec = 0.1;
        mission::MissionSetup setup(*adapter, config);
        setup.run();
    } catch (const std::runtime_error& error) {
        rejected = std::string(error.what()).find("raw IMU") != std::string::npos;
    }
    unsetenv("SITL_RAW_IMU_MIN_SAMPLES");
    unsetenv("SITL_REQUIRE_RAW_IMU_STABLE");
    CHECK(rejected);
    for (const auto& message : fake->decode_written_messages()) {
        if (message.msgid != MAVLINK_MSG_ID_COMMAND_LONG) continue;
        mavlink_command_long_t command{};
        mavlink_msg_command_long_decode(&message, &command);
        CHECK(command.command != MAV_CMD_COMPONENT_ARM_DISARM);
    }
    return true;
}

}  // namespace

int main() {
    const std::vector<std::pair<const char*, std::function<bool()>>> tests = {
        {"mission_packets_use_central_sender_boundary",
         mission_packets_use_central_sender_boundary},
        {"preflight_blocks_mission_protocol", preflight_blocks_mission_protocol},
        {"mission_setup_rejects_upload_ack", mission_setup_rejects_upload_ack},
        {"mission_setup_rejects_readback_mismatch",
         mission_setup_rejects_readback_mismatch},
        {"arm_failure_preserves_prearm_diagnostics",
         arm_failure_preserves_prearm_diagnostics},
        {"arm_success_requires_armed_heartbeat",
         arm_success_requires_armed_heartbeat},
        {"raw_imu_stability_gate_blocks_arm",
         raw_imu_stability_gate_blocks_arm},
        {"sitl_raw_imu_sample_floor_is_applied",
         sitl_raw_imu_sample_floor_is_applied},
    };

    int failed = 0;
    for (const auto& test : tests) {
        if (test.second()) {
            std::cout << "PASS " << test.first << '\n';
        } else {
            std::cerr << "FAIL " << test.first << '\n';
            ++failed;
        }
    }
    return failed == 0 ? 0 : 1;
}
