#include "flight_state.hpp"

#include <utility>

namespace safety {

const char* flight_state_name(FlightState state) {
    switch (state) {
        case FlightState::AUTO_WAIT: return "AUTO_WAIT";
        case FlightState::ARMED_TAKEOFF: return "ARMED_TAKEOFF";
        case FlightState::TARGET_SEARCH: return "TARGET_SEARCH";
        case FlightState::GUIDED: return "GUIDED";
        case FlightState::TARGET_LOSS_HOVER: return "TARGET_LOSS_HOVER";
        case FlightState::SAFE_HOVER: return "SAFE_HOVER";
        case FlightState::LANDING: return "LANDING";
        case FlightState::DISARMED: return "DISARMED";
        case FlightState::FAILED: return "FAILED";
    }
    return "UNKNOWN";
}

FlightStateMachine::FlightStateMachine(EventSink sink) : sink_(std::move(sink)) {
    const FlightStateEvent initial{0, FlightState::AUTO_WAIT, FlightState::AUTO_WAIT,
                                   "initial", Clock::now()};
    events_.push_back(initial);
    if (sink_) sink_(initial);
}

bool FlightStateMachine::accept_input(Clock::time_point timestamp,
                                      uint64_t input_sequence) {
    if (input_sequence != 0 && input_sequence <= last_input_sequence_) return false;
    if (last_input_timestamp_ != Clock::time_point{} && timestamp < last_input_timestamp_) {
        return false;
    }
    if (input_sequence != 0) last_input_sequence_ = input_sequence;
    last_input_timestamp_ = timestamp;
    return true;
}

bool FlightStateMachine::transition(FlightState next, std::string reason,
                                    Clock::time_point timestamp,
                                    uint64_t input_sequence) {
    if (!accept_input(timestamp, input_sequence)) return false;
    if (current_ == FlightState::SAFE_HOVER && next != FlightState::LANDING &&
        next != FlightState::DISARMED && next != FlightState::SAFE_HOVER) {
        return false;
    }
    if (current_ == next) return false;

    const FlightStateEvent event{next_sequence_++, current_, next, std::move(reason), timestamp};
    current_ = next;
    if (next == FlightState::SAFE_HOVER) safe_hover_latched_ = true;
    events_.push_back(event);
    if (sink_) sink_(event);
    return true;
}

bool FlightStateMachine::safety_fault(const FlightObservation& observation) const {
    return observation.heartbeat_timeout || !observation.health_ok ||
           observation.control_locked || observation.operator_mode_change ||
           observation.approach_timeout;
}

void FlightStateMachine::step(const FlightObservation& observation,
                              Clock::time_point timestamp,
                              uint64_t input_sequence) {
    if (!accept_input(timestamp, input_sequence)) return;

    // step() accepted the input already; transition() must not reject it a
    // second time. Use a zero sequence and the same monotonic timestamp.
    const auto apply = [&](FlightState next, const char* reason) {
        transition(next, reason, timestamp, 0);
    };

    if (observation.fatal_error && current_ != FlightState::DISARMED) {
        apply(FlightState::FAILED, "fatal startup or runtime error");
        return;
    }

    switch (current_) {
        case FlightState::AUTO_WAIT:
            if (safety_fault(observation)) {
                apply(FlightState::SAFE_HOVER, "preflight or health safety fault");
            } else if (observation.preflight_ready && observation.mission_ready &&
                       observation.auto_confirmed && observation.armed) {
                apply(FlightState::ARMED_TAKEOFF, "preflight, AUTO and ARM confirmed");
            }
            break;
        case FlightState::ARMED_TAKEOFF:
            if (safety_fault(observation)) {
                apply(FlightState::SAFE_HOVER, "takeoff safety fault");
            } else if (observation.altitude_reached) {
                apply(FlightState::TARGET_SEARCH, "target altitude reached");
            }
            break;
        case FlightState::TARGET_SEARCH:
            if (safety_fault(observation)) {
                apply(FlightState::SAFE_HOVER, "search safety fault");
            } else if (observation.target_confirmed) {
                apply(FlightState::GUIDED, "target confirmed");
            }
            break;
        case FlightState::GUIDED:
            if (safety_fault(observation)) {
                apply(FlightState::SAFE_HOVER, "GUIDED safety fault");
            } else if (observation.landing) {
                apply(FlightState::LANDING, "landing observed");
            } else if (observation.target_loss) {
                apply(FlightState::TARGET_LOSS_HOVER, "target temporarily lost");
            }
            break;
        case FlightState::TARGET_LOSS_HOVER:
            if (safety_fault(observation) || observation.target_loss_timeout) {
                apply(FlightState::SAFE_HOVER, "target loss safety timeout");
            } else if (observation.landing) {
                apply(FlightState::LANDING, "landing observed");
            } else if (observation.target_confirmed) {
                apply(FlightState::GUIDED, "target re-confirmed");
            }
            break;
        case FlightState::SAFE_HOVER:
            if (observation.landing) apply(FlightState::LANDING, "landing observed");
            else if (observation.disarmed) apply(FlightState::DISARMED, "disarm observed");
            break;
        case FlightState::LANDING:
            if (observation.disarmed) apply(FlightState::DISARMED, "disarm observed");
            break;
        case FlightState::DISARMED:
        case FlightState::FAILED:
            break;
    }
}

}  // namespace safety
