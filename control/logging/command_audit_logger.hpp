#pragma once

#include <fstream>
#include <cstdint>
#include <mutex>
#include <string>

#include "../app/flight_mission_app.hpp"
#include "../safety/safety_monitor.hpp"

namespace logging {

// Writes control-side decisions as JSONL for the simulation packet audit.
// This is observability only; it does not alter command authorization or
// MAVLink serialization.
class CommandAuditLogger {
public:
    CommandAuditLogger();

    void record_decision(const safety::CommandDecision& decision,
                         const safety::SafetyMonitor::Snapshot& authority);
    void record_control_lock(const std::string& reason,
                             const safety::SafetyMonitor::Snapshot& authority);
    void record_session_started(const safety::SafetyMonitor::Snapshot& authority);
    void record_phase_event(const app::FlightPhaseEvent& event);
    void record_algorithm_state(const std::string& algorithm,
                                const std::string& previous_state,
                                const std::string& current_state,
                                double elapsed_sec, bool tracking,
                                double observation_age_ms);

private:
    void write_event(const std::string& json);

    std::ofstream output_;
    std::mutex mutex_;
    uint64_t next_algorithm_sequence_ = 1;
};

}  // namespace logging
