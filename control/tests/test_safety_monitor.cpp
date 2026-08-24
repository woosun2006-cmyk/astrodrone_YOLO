#include "../safety/safety_monitor.hpp"
#include "test_mavlink_sender.hpp"
#include "fake_transport.hpp"

#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <vector>

namespace {

bool check(bool value, const char* expression, int line) {
    if (value) return true;
    std::cerr << "line " << line << ": check failed: " << expression << '\n';
    return false;
}
#define CHECK(expression) do { if (!check(static_cast<bool>(expression), #expression, __LINE__)) return false; } while (false)

mavlink_heartbeat_t heartbeat(uint32_t mode, bool armed = true) {
    mavlink_heartbeat_t value{};
    value.type = MAV_TYPE_QUADROTOR;
    value.autopilot = MAV_AUTOPILOT_ARDUPILOTMEGA;
    value.base_mode = armed ? MAV_MODE_FLAG_SAFETY_ARMED : 0;
    value.custom_mode = mode;
    value.system_status = MAV_STATE_ACTIVE;
    return value;
}

bool setup_baseline_does_not_lock() {
    safety::SafetyMonitor monitor;
    monitor.observe_heartbeat(heartbeat(0, false));
    monitor.observe_heartbeat(heartbeat(5, false));
    monitor.observe_heartbeat(heartbeat(3, true));
    CHECK(!monitor.automation_session_started());
    CHECK(!monitor.control_locked());
    CHECK(monitor.begin_automation_session(3));
    CHECK(monitor.baseline_mode() == 3);
    return true;
}

bool expected_guided_mode_is_accepted() {
    safety::SafetyMonitor monitor;
    monitor.observe_heartbeat(heartbeat(3));
    CHECK(monitor.begin_automation_session());
    CHECK(monitor.prepare_mode_request("GUIDED"));
    monitor.record_mode_request(safety::CommandRequest::set_mode("GUIDED"), true);
    monitor.observe_heartbeat(heartbeat(4));
    CHECK(!monitor.control_locked());
    return true;
}

bool unexpected_mode_change_latches() {
    safety::SafetyMonitor monitor;
    monitor.observe_heartbeat(heartbeat(4));
    CHECK(monitor.begin_automation_session());
    monitor.observe_heartbeat(heartbeat(0));
    CHECK(monitor.control_locked());
    CHECK(monitor.lock_reason() == safety::ControlLockReason::UnexpectedOperatorOrGcsModeChange);
    return true;
}

bool matching_mode_lease_expiry_does_not_lock() {
    safety::SafetyMonitor monitor;
    monitor.observe_heartbeat(heartbeat(4));
    CHECK(monitor.begin_automation_session());
    const auto now = safety::SafetyMonitor::Clock::now();
    const auto request = safety::CommandRequest::set_mode(
        "GUIDED", now - std::chrono::seconds(6), now - std::chrono::seconds(1));
    monitor.record_mode_request(request, true);
    monitor.poll(now);
    CHECK(!monitor.control_locked());
    return true;
}

bool delayed_requested_mode_after_lease_does_not_lock() {
    safety::SafetyMonitor monitor;
    monitor.observe_heartbeat(heartbeat(4));
    CHECK(monitor.begin_automation_session());
    const auto now = safety::SafetyMonitor::Clock::now();
    const auto request = safety::CommandRequest::set_mode(
        "LOITER", now - std::chrono::seconds(6), now - std::chrono::seconds(1));
    monitor.record_mode_request(request, true);
    monitor.poll(now);
    CHECK(!monitor.control_locked());
    monitor.observe_heartbeat(heartbeat(5), now + std::chrono::milliseconds(1));
    CHECK(!monitor.control_locked());
    return true;
}

bool expired_mode_lease_with_changed_mode_latches() {
    safety::SafetyMonitor monitor;
    monitor.observe_heartbeat(heartbeat(4));
    CHECK(monitor.begin_automation_session());
    const auto now = safety::SafetyMonitor::Clock::now();
    const auto request = safety::CommandRequest::set_mode(
        "GUIDED", now - std::chrono::seconds(6), now - std::chrono::seconds(1));
    monitor.record_mode_request(request, true);
    monitor.observe_heartbeat(heartbeat(0), now - std::chrono::seconds(2));
    monitor.poll(now);
    CHECK(monitor.control_locked());
    return true;
}

bool lock_blocks_all_vehicle_commands() {
    safety::SafetyMonitor monitor;
    monitor.observe_heartbeat(heartbeat(4));
    CHECK(monitor.begin_automation_session());
    monitor.observe_heartbeat(heartbeat(0));
    auto connection = std::make_unique<MavConnection>(std::make_unique<FakeTransport>());
    test::TestPacketSender sender(*connection, monitor);
    for (const auto& decision : {sender.set_mode("GUIDED"), sender.arm_disarm(true),
                                 sender.takeoff(5.0), sender.send_velocity_body(0.2, 0, 0, 0),
                                 sender.send_zero_velocity(), sender.land()}) {
        CHECK(!decision.allowed);
        CHECK(decision.block_reason == safety::GateBlockReason::ControlLocked);
    }
    return true;
}

bool preflight_blocks_until_ready() {
    safety::SafetyMonitor monitor;
    monitor.set_preflight_required(true);
    monitor.set_preflight_ready(false);
    CHECK(!monitor.can_arm());
    CHECK(!monitor.can_send_velocity());
    monitor.set_preflight_ready(true);
    CHECK(monitor.can_arm());
    CHECK(monitor.can_send_velocity());
    return true;
}

}  // namespace

int main() {
    const std::vector<std::pair<const char*, std::function<bool()>>> tests = {
        {"setup_baseline_does_not_lock", setup_baseline_does_not_lock},
        {"expected_guided_mode_is_accepted", expected_guided_mode_is_accepted},
        {"unexpected_mode_change_latches", unexpected_mode_change_latches},
        {"matching_mode_lease_expiry_does_not_lock", matching_mode_lease_expiry_does_not_lock},
        {"delayed_requested_mode_after_lease_does_not_lock", delayed_requested_mode_after_lease_does_not_lock},
        {"expired_mode_lease_with_changed_mode_latches", expired_mode_lease_with_changed_mode_latches},
        {"lock_blocks_all_vehicle_commands", lock_blocks_all_vehicle_commands},
        {"preflight_blocks_until_ready", preflight_blocks_until_ready},
    };
    int failed = 0;
    for (const auto& test : tests) {
        if (test.second()) std::cout << "PASS " << test.first << '\n';
        else { std::cerr << "FAIL " << test.first << '\n'; ++failed; }
    }
    return failed == 0 ? 0 : 1;
}
