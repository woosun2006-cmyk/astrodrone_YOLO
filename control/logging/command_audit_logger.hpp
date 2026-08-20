#pragma once

#include <fstream>
#include <mutex>
#include <string>

#include "../autopilot/command_gate.hpp"
#include "../safety/control_authority.hpp"
#include "../safety/flight_state.hpp"

namespace logging {

// Writes control-side decisions as JSONL for the simulation packet audit.
// This is observability only; it does not alter command authorization or
// MAVLink serialization.
class CommandAuditLogger {
public:
    CommandAuditLogger();

    void record_decision(const autopilot::CommandDecision& decision,
                         const safety::ControlAuthority::Snapshot& authority);
    void record_control_lock(const std::string& reason,
                             const safety::ControlAuthority::Snapshot& authority);
    void record_session_started(const safety::ControlAuthority::Snapshot& authority);
    void record_state_event(const safety::FlightStateEvent& event);

private:
    void write_event(const std::string& json);

    std::ofstream output_;
    std::mutex mutex_;
};

}  // namespace logging
