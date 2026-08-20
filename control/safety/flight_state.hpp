#pragma once

#include <chrono>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace safety {

enum class FlightState {
    AUTO_WAIT,
    ARMED_TAKEOFF,
    TARGET_SEARCH,
    GUIDED,
    TARGET_LOSS_HOVER,
    SAFE_HOVER,
    LANDING,
    DISARMED,
    FAILED,
};

const char* flight_state_name(FlightState state);

struct FlightObservation {
    bool preflight_ready = false;
    bool mission_ready = false;
    bool auto_confirmed = false;
    bool armed = false;
    bool altitude_reached = false;
    bool target_confirmed = false;
    bool target_loss = false;
    bool target_loss_timeout = false;
    bool health_ok = true;
    bool heartbeat_timeout = false;
    bool control_locked = false;
    bool operator_mode_change = false;
    bool approach_timeout = false;
    bool landing = false;
    bool disarmed = false;
    bool fatal_error = false;
};

struct FlightStateEvent {
    uint64_t sequence = 0;
    FlightState previous = FlightState::AUTO_WAIT;
    FlightState current = FlightState::AUTO_WAIT;
    std::string reason;
    std::chrono::steady_clock::time_point timestamp{};
};

class FlightStateMachine {
public:
    using Clock = std::chrono::steady_clock;
    using EventSink = std::function<void(const FlightStateEvent&)>;

    explicit FlightStateMachine(EventSink sink = {});

    // The only method that changes current_. It rejects stale external event
    // sequences and timestamps, and keeps SAFE_HOVER latched.
    bool transition(FlightState next, std::string reason,
                    Clock::time_point timestamp = Clock::now(),
                    uint64_t input_sequence = 0);

    void step(const FlightObservation& observation,
              Clock::time_point timestamp = Clock::now(),
              uint64_t input_sequence = 0);

    FlightState current() const { return current_; }
    uint64_t next_sequence() const { return next_sequence_; }
    uint64_t last_input_sequence() const { return last_input_sequence_; }
    const std::vector<FlightStateEvent>& events() const { return events_; }
    bool safe_hover_latched() const { return safe_hover_latched_; }

private:
    bool accept_input(Clock::time_point timestamp, uint64_t input_sequence);
    bool safety_fault(const FlightObservation& observation) const;

    FlightState current_ = FlightState::AUTO_WAIT;
    uint64_t next_sequence_ = 1;
    uint64_t last_input_sequence_ = 0;
    Clock::time_point last_input_timestamp_{};
    bool safe_hover_latched_ = false;
    EventSink sink_;
    std::vector<FlightStateEvent> events_;
};

}  // namespace safety
