#include "../app/flight_mission_app.hpp"
#include "../safety/safety_monitor.hpp"

#include <chrono>
#include <iostream>

namespace {
using Clock = std::chrono::steady_clock;

bool check(bool value, const char* expression, int line) {
    if (value) return true;
    std::cerr << "line " << line << ": check failed: " << expression << '\n';
    return false;
}
#define CHECK(expression) \
    do { \
        if (!check(static_cast<bool>(expression), #expression, __LINE__)) return false; \
    } while (false)

bool preflight_requires_stability() {
    const auto t0 = Clock::now();
    safety::SafetyMonitor monitor;
    safety::PreflightSample sample;
    sample.heartbeat_event = true;
    sample.valid_autopilot_heartbeat = true;
    sample.heartbeat_fresh = true;
    sample.gps_ok = true;
    sample.ekf_ok = true;
    sample.battery_valid = true;
    sample.prearm_healthy = true;
    sample.telemetry_fresh = true;
    sample.now = t0;
    monitor.observe_preflight(sample);
    CHECK(!monitor.preflight_ready());
    sample.now = t0 + std::chrono::seconds(1);
    monitor.observe_preflight(sample);
    CHECK(!monitor.preflight_ready());
    sample.now = t0 + std::chrono::seconds(2);
    monitor.observe_preflight(sample);
    CHECK(monitor.preflight_ready());
    return true;
}

bool phase_order_latch_and_stale_events() {
    const auto t0 = Clock::now();
    app::FlightPhaseController phases;
    CHECK(phases.current() == app::FlightPhase::PRECHECK);
    CHECK(phases.transition(app::FlightPhase::TAKEOFF, "AUTO+ARM confirmed", t0, 1));
    CHECK(phases.transition(app::FlightPhase::TARGET_SEARCH, "5m reached", t0 +
                            std::chrono::milliseconds(100), 2));
    CHECK(phases.transition(app::FlightPhase::GUIDED, "GUIDED heartbeat confirmed", t0 +
                            std::chrono::milliseconds(200), 3));

    // A late event cannot overwrite the current phase.
    CHECK(!phases.transition(app::FlightPhase::TARGET_SEARCH, "late target event", t0,
                             2));
    CHECK(phases.current() == app::FlightPhase::GUIDED);

    CHECK(phases.transition(app::FlightPhase::REACQUIRE, "target loss", t0 +
                            std::chrono::milliseconds(250)));
    CHECK(phases.transition(app::FlightPhase::GUIDED, "target reacquired", t0 +
                            std::chrono::milliseconds(275)));

    // A latched SAFE_HOVER is distinct from the recoverable REACQUIRE phase.
    // Rediscovery must not reopen GUIDED after SAFE_HOVER is latched.
    CHECK(phases.transition(app::FlightPhase::SAFE_HOVER, "target loss", t0 +
                            std::chrono::milliseconds(300), 4));
    CHECK(phases.safe_hover_latched());
    CHECK(!phases.transition(app::FlightPhase::GUIDED, "target rediscovered", t0 +
                             std::chrono::milliseconds(400), 5));
    CHECK(phases.current() == app::FlightPhase::SAFE_HOVER);

    CHECK(phases.transition(app::FlightPhase::LANDING, "explicit landing", t0 +
                            std::chrono::milliseconds(500), 6));
    CHECK(phases.transition(app::FlightPhase::FINISHED, "disarmed telemetry", t0 +
                            std::chrono::milliseconds(600), 7));
    CHECK(phases.events().size() == 8);
    CHECK(phases.events().front().sequence == 1);
    CHECK(phases.events().back().current == app::FlightPhase::FINISHED);
    return true;
}

}  // namespace

int main() {
    if (!preflight_requires_stability()) return 1;
    if (!phase_order_latch_and_stale_events()) return 1;
    std::cout << "preflight/flight phase tests passed\n";
    return 0;
}
