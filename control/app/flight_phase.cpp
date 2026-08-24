#include "flight_mission_app.hpp"

#include <utility>

namespace app {

const char* flight_phase_name(FlightPhase phase) {
    switch (phase) {
        case FlightPhase::PRECHECK: return "PRECHECK";
        case FlightPhase::TAKEOFF: return "TAKEOFF";
        case FlightPhase::TARGET_SEARCH: return "TARGET_SEARCH";
        case FlightPhase::GUIDED: return "GUIDED";
        case FlightPhase::REACQUIRE: return "REACQUIRE";
        case FlightPhase::SAFE_HOVER: return "SAFE_HOVER";
        case FlightPhase::LANDING: return "LANDING";
        case FlightPhase::FINISHED: return "FINISHED";
    }
    return "FINISHED";
}

bool FlightPhaseController::accept_input(Clock::time_point timestamp,
                                          uint64_t input_sequence) {
    if (input_sequence != 0 && input_sequence <= last_input_sequence_) return false;
    if (last_input_timestamp_ != Clock::time_point{} && timestamp < last_input_timestamp_) {
        return false;
    }
    if (input_sequence != 0) last_input_sequence_ = input_sequence;
    last_input_timestamp_ = timestamp;
    return true;
}

bool FlightPhaseController::transition(FlightPhase next, std::string reason,
                                        Clock::time_point timestamp,
                                        uint64_t input_sequence) {
    if (!accept_input(timestamp, input_sequence) || current_ == next) return false;
    if (safe_hover_latched_ && next != FlightPhase::SAFE_HOVER &&
        next != FlightPhase::LANDING && next != FlightPhase::FINISHED) {
        return false;
    }
    const FlightPhaseEvent event{next_sequence_++, current_, next,
                                 std::move(reason), timestamp};
    current_ = next;
    if (next == FlightPhase::SAFE_HOVER) safe_hover_latched_ = true;
    events_.push_back(event);
    if (sink_) sink_(events_.back());
    return true;
}

}  // namespace app
