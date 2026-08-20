#include "../autopilot/command_sender.hpp"
#include "../safety/flight_state.hpp"
#include "../safety/preflight_gate.hpp"
#include "fake_transport.hpp"

#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace {

using Clock = std::chrono::steady_clock;

bool check(bool condition, const char* expression, int line) {
    if (condition) return true;
    std::cerr << "line " << line << ": check failed: " << expression << '\n';
    return false;
}

#define CHECK(expression) \
    do { \
        if (!check(static_cast<bool>(expression), #expression, __LINE__)) return false; \
    } while (false)

FakeTransport* g_transport = nullptr;

safety::PreflightSample stable_sample(bool heartbeat_event, Clock::time_point now) {
    safety::PreflightSample sample;
    sample.heartbeat_event = heartbeat_event;
    sample.valid_autopilot_heartbeat = heartbeat_event;
    sample.gps_ok = true;
    sample.ekf_ok = true;
    sample.battery_valid = true;
    sample.rc_policy_ok = true;
    sample.telemetry_fresh = true;
    sample.yolo_ready = true;
    sample.camera_frame = false;
    sample.now = now;
    return sample;
}

bool preflight_requires_stable_inputs() {
    const auto t0 = Clock::now();
    safety::PreflightPolicy policy;
    policy.require_vision = true;
    policy.required_camera_frames = 2;
    safety::PreflightGate gate(policy);

    auto first = stable_sample(true, t0);
    first.camera_frame = true;
    gate.observe(first);
    CHECK(!gate.ready());
    CHECK(gate.consecutive_heartbeats() == 1);

    auto second = stable_sample(true, t0 + std::chrono::seconds(1));
    second.camera_frame = true;
    gate.observe(second);
    CHECK(!gate.ready());

    auto third = stable_sample(true, t0 + std::chrono::seconds(2));
    gate.observe(third);
    CHECK(gate.ready());
    CHECK(gate.reasons().empty());

    gate.poll(t0 + std::chrono::milliseconds(5100));
    CHECK(!gate.ready());
    CHECK(gate.consecutive_heartbeats() == 0);
    return true;
}

bool heartbeat_freshness_is_separate_from_sensor_freshness() {
    const auto t0 = Clock::now();
    safety::PreflightGate gate;

    auto sample = stable_sample(true, t0);
    gate.observe(sample);
    sample = stable_sample(true, t0 + std::chrono::milliseconds(1100));
    gate.observe(sample);
    sample = stable_sample(true, t0 + std::chrono::milliseconds(2200));
    gate.observe(sample);
    CHECK(gate.ready());

    // A 2.5 s heartbeat gap is within the dedicated 3 s heartbeat policy.
    gate.poll(t0 + std::chrono::milliseconds(4700));
    CHECK(gate.ready());

    // A 3.1 s gap is rejected even though the non-heartbeat telemetry remains
    // marked fresh in this synthetic sample.
    gate.poll(t0 + std::chrono::milliseconds(5301));
    CHECK(!gate.ready());
    CHECK(gate.consecutive_heartbeats() == 0);

    safety::PreflightGate sensor_gate;
    auto first = stable_sample(true, t0);
    sensor_gate.observe(first);
    auto second = stable_sample(true, t0 + std::chrono::milliseconds(1100));
    sensor_gate.observe(second);
    auto third = stable_sample(true, t0 + std::chrono::milliseconds(2200));
    sensor_gate.observe(third);
    CHECK(sensor_gate.ready());

    // Heartbeat is still fresh, but GPS/EKF/SYS_STATUS freshness is not.
    auto stale_sensors = stable_sample(false, t0 + std::chrono::milliseconds(2300));
    stale_sensors.heartbeat_fresh = true;
    stale_sensors.telemetry_fresh = false;
    sensor_gate.observe(stale_sensors);
    CHECK(!sensor_gate.ready());
    return true;
}

bool command_gate_blocks_until_preflight() {
    autopilot::CommandGate gate;
    gate.set_preflight_required(true);
    gate.set_preflight_ready(false);

    MavConnection& connection = drone::connect("fake://preflight-gate", 0.1);
    CHECK(g_transport != nullptr);
    g_transport->clear_written();
    autopilot::CommandSender sender(connection, gate);

    const auto mode = sender.set_mode("GUIDED");
    const auto arm = sender.arm_disarm(true);
    const auto takeoff = sender.takeoff(5.0);
    const auto velocity = sender.send_velocity_body(0.2, 0.0, 0.1, 0.0);
    const auto zero = sender.send_zero_velocity();
    const auto land = sender.land();
    for (const auto& decision : {mode, arm, takeoff, velocity, zero, land}) {
        CHECK(!decision.allowed);
        CHECK(!decision.sent);
        CHECK(decision.block_reason == autopilot::GateBlockReason::PreflightNotReady);
    }
    CHECK(g_transport->write_call_count() == 0);

    gate.set_preflight_ready(true);
    CHECK(sender.set_mode("GUIDED"));
    CHECK(g_transport->write_call_count() == 1);
    return true;
}

bool state_machine_has_safe_latch_and_order() {
    const auto t0 = Clock::now();
    safety::FlightStateMachine machine;

    safety::FlightObservation observation;
    observation.preflight_ready = true;
    observation.mission_ready = true;
    observation.auto_confirmed = true;
    observation.armed = true;
    machine.step(observation, t0, 1);
    CHECK(machine.current() == safety::FlightState::ARMED_TAKEOFF);

    observation.altitude_reached = true;
    machine.step(observation, t0 + std::chrono::milliseconds(100), 2);
    CHECK(machine.current() == safety::FlightState::TARGET_SEARCH);

    observation.target_confirmed = true;
    machine.step(observation, t0 + std::chrono::milliseconds(200), 3);
    CHECK(machine.current() == safety::FlightState::GUIDED);

    observation.target_confirmed = false;
    observation.target_loss = true;
    machine.step(observation, t0 + std::chrono::milliseconds(300), 4);
    CHECK(machine.current() == safety::FlightState::TARGET_LOSS_HOVER);

    observation.target_loss = false;
    observation.target_confirmed = true;
    machine.step(observation, t0 + std::chrono::milliseconds(400), 5);
    CHECK(machine.current() == safety::FlightState::GUIDED);

    observation.target_confirmed = false;
    observation.control_locked = true;
    machine.step(observation, t0 + std::chrono::milliseconds(500), 6);
    CHECK(machine.current() == safety::FlightState::SAFE_HOVER);
    CHECK(machine.safe_hover_latched());

    observation.control_locked = false;
    observation.target_confirmed = true;
    machine.step(observation, t0 + std::chrono::milliseconds(600), 7);
    CHECK(machine.current() == safety::FlightState::SAFE_HOVER);

    observation.landing = true;
    machine.step(observation, t0 + std::chrono::milliseconds(700), 8);
    CHECK(machine.current() == safety::FlightState::LANDING);
    observation.disarmed = true;
    machine.step(observation, t0 + std::chrono::milliseconds(800), 9);
    CHECK(machine.current() == safety::FlightState::DISARMED);
    CHECK(machine.events().size() == 9);

    // A late event cannot append a transition after the current state.
    observation.fatal_error = true;
    machine.step(observation, t0 + std::chrono::milliseconds(750), 8);
    CHECK(machine.current() == safety::FlightState::DISARMED);
    return true;
}

}  // namespace

std::unique_ptr<Transport> open_transport(const std::string& address, int baud) {
    (void)address;
    (void)baud;
    auto transport = std::make_unique<FakeTransport>();
    g_transport = transport.get();
    mavlink_message_t heartbeat{};
    mavlink_msg_heartbeat_pack(1, MAV_COMP_ID_AUTOPILOT1, &heartbeat,
                               MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_ARDUPILOTMEGA,
                               0, 0, MAV_STATE_ACTIVE);
    transport->queue_incoming_message(heartbeat);
    return transport;
}

int main() {
    const std::vector<std::pair<const char*, std::function<bool()>>> tests = {
        {"preflight_requires_stable_inputs", preflight_requires_stable_inputs},
        {"heartbeat_freshness_is_separate_from_sensor_freshness",
         heartbeat_freshness_is_separate_from_sensor_freshness},
        {"command_gate_blocks_until_preflight", command_gate_blocks_until_preflight},
        {"state_machine_has_safe_latch_and_order", state_machine_has_safe_latch_and_order},
    };
    int failed = 0;
    for (const auto& test : tests) {
        if (test.second()) std::cout << "PASS " << test.first << '\n';
        else {
            std::cerr << "FAIL " << test.first << '\n';
            ++failed;
        }
    }
    return failed == 0 ? 0 : 1;
}
