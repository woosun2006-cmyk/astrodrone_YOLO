#include "../autopilot/command_sender.hpp"
#include "../safety/control_authority.hpp"
#include "../safety/health_monitor.hpp"
#include "fake_transport.hpp"

#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace {

constexpr uint8_t kTargetSystem = 1;
constexpr uint8_t kTargetComponent = MAV_COMP_ID_AUTOPILOT1;

FakeTransport* g_opened_transport = nullptr;

bool check(bool condition, const char* expression, int line) {
    if (condition) return true;
    std::cerr << "line " << line << ": check failed: " << expression << std::endl;
    return false;
}

#define CHECK(expression) \
    do { \
        if (!check(static_cast<bool>(expression), #expression, __LINE__)) return false; \
    } while (false)

mavlink_heartbeat_t heartbeat(uint32_t mode, uint8_t system_status = MAV_STATE_ACTIVE,
                             bool armed = true) {
    mavlink_heartbeat_t message{};
    message.type = MAV_TYPE_QUADROTOR;
    message.autopilot = MAV_AUTOPILOT_ARDUPILOTMEGA;
    message.base_mode = armed ? MAV_MODE_FLAG_SAFETY_ARMED : 0;
    message.custom_mode = mode;
    message.system_status = system_status;
    return message;
}

mavlink_message_t heartbeat_message(uint32_t mode) {
    mavlink_message_t message{};
    mavlink_msg_heartbeat_pack(kTargetSystem, kTargetComponent, &message, MAV_TYPE_QUADROTOR,
                               MAV_AUTOPILOT_ARDUPILOTMEGA, MAV_MODE_FLAG_SAFETY_ARMED, mode,
                               MAV_STATE_ACTIVE);
    return message;
}

bool startup_mode_is_baseline_only() {
    safety::ControlAuthority authority;
    authority.observe_heartbeat(heartbeat(0));  // STABILIZE
    authority.begin_automation_session();
    authority.observe_heartbeat(heartbeat(0));

    CHECK(authority.baseline_known());
    CHECK(authority.baseline_mode() == 0);
    CHECK(!authority.control_locked());
    CHECK(authority.heartbeat_count() == 2);
    return true;
}

bool mission_setup_before_session_does_not_lock() {
    safety::ControlAuthority authority;
    authority.observe_heartbeat(heartbeat(0, MAV_STATE_ACTIVE, false));  // STABILIZE baseline
    authority.observe_heartbeat(heartbeat(5, MAV_STATE_ACTIVE, false));  // LOITER setup
    authority.observe_heartbeat(heartbeat(3, MAV_STATE_ACTIVE, false));  // AUTO setup
    authority.observe_heartbeat(heartbeat(3, MAV_STATE_ACTIVE, true));   // ARM setup

    CHECK(!authority.automation_session_started());
    CHECK(!authority.control_locked());
    CHECK(authority.begin_automation_session(3));
    CHECK(authority.automation_session_started());
    CHECK(!authority.control_locked());
    CHECK(authority.actual_mode() == 3);
    CHECK(authority.baseline_mode() == 3);
    CHECK(authority.snapshot().previous_mode.has_value());
    CHECK(*authority.snapshot().previous_mode == 3);
    return true;
}

bool jetson_mode_request_is_accepted() {
    safety::ControlAuthority authority;
    authority.observe_heartbeat(heartbeat(0));
    authority.begin_automation_session();
    const auto now = safety::ControlAuthority::Clock::now();
    authority.record_mode_request(
        autopilot::CommandRequest::set_mode("GUIDED", now, now + std::chrono::seconds(5)), true);
    authority.observe_heartbeat(heartbeat(4));

    CHECK(!authority.control_locked());
    CHECK(authority.actual_mode() == 4);
    return true;
}

bool command_sender_records_successful_mode_request() {
    safety::ControlAuthority authority;
    authority.observe_heartbeat(heartbeat(0));
    authority.begin_automation_session();

    autopilot::CommandGate gate;
    MavConnection& connection = drone::connect("fake://mode-request", 0.1);
    CHECK(g_opened_transport != nullptr);
    g_opened_transport->clear_written();
    autopilot::CommandSender sender(
        connection, gate, {},
        [&](const autopilot::CommandRequest& request, bool write_succeeded) {
            authority.record_mode_request(request, write_succeeded);
        });

    CHECK(sender.set_mode("GUIDED"));
    authority.observe_heartbeat(heartbeat(4));
    CHECK(!authority.control_locked());
    CHECK(g_opened_transport->write_call_count() == 1);
    return true;
}

bool setup_baseline_allows_guided_and_velocity() {
    safety::ControlAuthority authority;
    authority.observe_heartbeat(heartbeat(0, MAV_STATE_ACTIVE, false));
    authority.observe_heartbeat(heartbeat(5, MAV_STATE_ACTIVE, false));
    authority.observe_heartbeat(heartbeat(3, MAV_STATE_ACTIVE, true));
    CHECK(authority.begin_automation_session(3));

    autopilot::CommandGate gate;
    MavConnection& connection = drone::connect("fake://guided-after-setup", 0.1);
    CHECK(g_opened_transport != nullptr);
    g_opened_transport->clear_written();
    autopilot::CommandSender sender(
        connection, gate, {},
        [&](const autopilot::CommandRequest& request, bool write_succeeded) {
            authority.record_mode_request(request, write_succeeded);
        });

    CHECK(authority.prepare_mode_request("GUIDED"));
    const auto guided = sender.set_mode("GUIDED");
    CHECK(guided.allowed);
    CHECK(guided.sent);
    CHECK(authority.snapshot().expected_mode.has_value());
    CHECK(*authority.snapshot().expected_mode == "GUIDED");

    authority.observe_heartbeat(heartbeat(4, MAV_STATE_ACTIVE, true));
    CHECK(!authority.control_locked());
    const auto velocity = sender.send_velocity_body(0.2, 0.0, 0.1, 0.0);
    CHECK(velocity.allowed);
    CHECK(velocity.sent);
    const auto messages = g_opened_transport->decode_written_messages();
    CHECK(messages.size() == 2);
    CHECK(messages[0].msgid == MAVLINK_MSG_ID_SET_MODE);
    CHECK(messages[1].msgid == MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED);
    return true;
}

bool write_success_without_expected_heartbeat_is_not_success() {
    safety::ControlAuthority authority;
    authority.observe_heartbeat(heartbeat(0));
    authority.begin_automation_session();

    autopilot::CommandGate gate;
    MavConnection& connection = drone::connect("fake://mode-timeout", 0.1);
    CHECK(g_opened_transport != nullptr);
    g_opened_transport->clear_written();
    autopilot::CommandSender sender(
        connection, gate, {},
        [&](const autopilot::CommandRequest& request, bool write_succeeded) {
            authority.record_mode_request(request, write_succeeded);
        });

    const auto sent_at = safety::ControlAuthority::Clock::now();
    const auto request = autopilot::CommandRequest::set_mode(
        "GUIDED", sent_at, sent_at + std::chrono::seconds(1));
    const auto decision = sender.send(request);
    CHECK(decision.allowed);
    CHECK(decision.sent);
    CHECK(authority.latest_mode_request().has_value());
    CHECK(authority.latest_mode_request()->write_succeeded);
    CHECK(!authority.control_locked());

    authority.poll(sent_at + std::chrono::seconds(2));
    CHECK(authority.control_locked());
    CHECK(authority.lock_reason() == safety::ControlLockReason::UnknownExternalModeChange);
    return true;
}

bool unexpected_mode_change_latches() {
    safety::ControlAuthority authority;
    authority.observe_heartbeat(heartbeat(4));
    authority.begin_automation_session();
    authority.observe_heartbeat(heartbeat(0));

    CHECK(authority.control_locked());
    CHECK(authority.lock_reason() ==
          safety::ControlLockReason::UnexpectedOperatorOrGcsModeChange);
    return true;
}

bool lock_blocks_all_commands_and_records_reason() {
    safety::ControlAuthority authority;
    authority.observe_heartbeat(heartbeat(4));
    authority.begin_automation_session();
    authority.observe_heartbeat(heartbeat(0));

    autopilot::CommandGate gate;
    authority.apply_to(gate);
    MavConnection& connection = drone::connect("fake://control-authority", 0.1);
    CHECK(g_opened_transport != nullptr);
    g_opened_transport->clear_written();
    autopilot::CommandSender sender(connection, gate);

    const auto mode = sender.set_mode("GUIDED");
    const auto arm = sender.arm_disarm(true);
    const auto takeoff = sender.takeoff(5.0);
    const auto velocity = sender.send_velocity_body(0.2, 0.0, 0.0, 0.0);
    const auto zero = sender.send_zero_velocity();
    const auto land = sender.land();

    for (const auto& decision : {mode, arm, takeoff, velocity, zero, land}) {
        CHECK(!decision.allowed);
        CHECK(!decision.sent);
        CHECK(decision.block_reason == autopilot::GateBlockReason::ControlLocked);
        CHECK(decision.control_lock_reason == "UnexpectedOperatorOrGcsModeChange");
    }
    CHECK(g_opened_transport->write_call_count() == 0);
    return true;
}

bool lock_is_sticky_and_telemetry_continues() {
    using Clock = safety::ControlAuthority::Clock;
    const auto t0 = Clock::now();
    const auto t1 = t0 + std::chrono::milliseconds(10);
    const auto t2 = t1 + std::chrono::milliseconds(10);

    safety::ControlAuthority authority;
    authority.observe_heartbeat(heartbeat(4), t0);
    authority.begin_automation_session();
    authority.observe_heartbeat(heartbeat(0), t1);
    CHECK(authority.control_locked());
    authority.observe_heartbeat(heartbeat(4), t2);
    CHECK(authority.control_locked());
    CHECK(authority.lock_reason() ==
          safety::ControlLockReason::UnexpectedOperatorOrGcsModeChange);
    CHECK(authority.actual_mode() == 4);
    CHECK(authority.heartbeat_count() == 3);
    CHECK(authority.last_heartbeat() == t2);

    safety::HealthMonitor monitor;
    mavlink_message_t first{};
    mavlink_msg_heartbeat_pack(1, MAV_COMP_ID_AUTOPILOT1, &first, MAV_TYPE_QUADROTOR,
                               MAV_AUTOPILOT_ARDUPILOTMEGA, 0, 4, MAV_STATE_ACTIVE);
    monitor.update(first, t0);
    mavlink_message_t second{};
    mavlink_msg_heartbeat_pack(1, MAV_COMP_ID_AUTOPILOT1, &second, MAV_TYPE_QUADROTOR,
                               MAV_AUTOPILOT_ARDUPILOTMEGA, 0, 0, MAV_STATE_ACTIVE);
    monitor.update(second, t2);
    CHECK(monitor.state().mode_updated_at == t2);
    CHECK(monitor.state().custom_mode == 0);
    return true;
}

bool failsafe_and_unknown_reasons_are_classified() {
    safety::ControlAuthority no_evidence;
    no_evidence.observe_heartbeat(heartbeat(4));
    no_evidence.begin_automation_session();
    no_evidence.observe_heartbeat(heartbeat(6));  // RTL alone is not evidence.
    CHECK(no_evidence.lock_reason() == safety::ControlLockReason::UnknownExternalModeChange);

    safety::ControlAuthority failsafe;
    failsafe.observe_heartbeat(heartbeat(4));
    failsafe.begin_automation_session();
    failsafe.observe_failsafe_evidence("Radio failsafe: RTL", safety::ControlAuthority::Clock::now());
    failsafe.observe_heartbeat(heartbeat(6));  // RTL with explicit evidence.
    CHECK(failsafe.lock_reason() == safety::ControlLockReason::ArduPilotFailsafeModeChange);

    safety::ControlAuthority unknown;
    unknown.observe_heartbeat(heartbeat(4));
    unknown.begin_automation_session();
    unknown.observe_heartbeat(heartbeat(999));
    CHECK(unknown.lock_reason() == safety::ControlLockReason::UnknownExternalModeChange);
    return true;
}

}  // namespace

std::unique_ptr<Transport> open_transport(const std::string& address, int baud) {
    (void)address;
    (void)baud;
    auto transport = std::make_unique<FakeTransport>();
    g_opened_transport = transport.get();
    transport->queue_incoming_message(heartbeat_message(4));
    return transport;
}

int main() {
    const std::vector<std::pair<const char*, std::function<bool()>>> tests = {
        {"startup_mode_is_baseline_only", startup_mode_is_baseline_only},
        {"mission_setup_before_session_does_not_lock",
         mission_setup_before_session_does_not_lock},
        {"jetson_mode_request_is_accepted", jetson_mode_request_is_accepted},
        {"command_sender_records_successful_mode_request",
         command_sender_records_successful_mode_request},
        {"setup_baseline_allows_guided_and_velocity",
         setup_baseline_allows_guided_and_velocity},
        {"write_success_without_expected_heartbeat_is_not_success",
         write_success_without_expected_heartbeat_is_not_success},
        {"unexpected_mode_change_latches", unexpected_mode_change_latches},
        {"lock_blocks_all_commands_and_records_reason",
         lock_blocks_all_commands_and_records_reason},
        {"lock_is_sticky_and_telemetry_continues", lock_is_sticky_and_telemetry_continues},
        {"failsafe_and_unknown_reasons_are_classified",
         failsafe_and_unknown_reasons_are_classified},
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
