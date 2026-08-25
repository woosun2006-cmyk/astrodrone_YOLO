#include "command_audit_logger.hpp"

#include <chrono>
#include <ctime>
#include <cstdlib>
#include <iomanip>
#include <sstream>

#if __has_include(<filesystem>)
#include <filesystem>
#else
#include <experimental/filesystem>
#endif

namespace logging {
namespace {

#if __has_include(<filesystem>)
namespace fs = std::filesystem;
#else
namespace fs = std::experimental::filesystem;
#endif

struct EventTime {
    long long unix_ms;
    std::string iso;
};

EventTime now() {
    const auto current = std::chrono::system_clock::now();
    const auto unix_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                             current.time_since_epoch())
                             .count();
    const auto seconds = std::chrono::time_point_cast<std::chrono::seconds>(current);
    const auto milliseconds = static_cast<int>(unix_ms % 1000);
    const std::time_t time = std::chrono::system_clock::to_time_t(seconds);
    std::tm utc{};
#if defined(_WIN32)
    gmtime_s(&utc, &time);
#else
    gmtime_r(&time, &utc);
#endif

    std::ostringstream iso;
    iso << std::put_time(&utc, "%Y-%m-%dT%H:%M:%S") << '.' << std::setfill('0')
        << std::setw(3) << milliseconds << 'Z';
    return {unix_ms, iso.str()};
}

std::string json_escape(const std::string& value) {
    std::ostringstream escaped;
    for (const unsigned char character : value) {
        switch (character) {
            case '\\': escaped << "\\\\"; break;
            case '"': escaped << "\\\""; break;
            case '\n': escaped << "\\n"; break;
            case '\r': escaped << "\\r"; break;
            case '\t': escaped << "\\t"; break;
            default:
                if (character < 0x20) {
                    escaped << "\\u" << std::hex << std::setw(4) << std::setfill('0')
                            << static_cast<int>(character) << std::dec;
                } else {
                    escaped << character;
                }
        }
    }
    return escaped.str();
}

const char* nullable_reason(const std::string& reason) {
    return reason.empty() ? nullptr : reason.c_str();
}

void append_authority_snapshot(
    std::ostringstream& event,
    const safety::SafetyMonitor::Snapshot& authority) {
    event << ",\"previous_mode\":";
    if (authority.previous_mode) event << *authority.previous_mode;
    else event << "null";
    event << ",\"current_mode\":";
    if (authority.current_mode) event << *authority.current_mode;
    else event << "null";
    event << ",\"session_started\":" << (authority.session_started ? "true" : "false")
          << ",\"expected_mode\":";
    if (authority.expected_mode) {
        event << "\"" << json_escape(*authority.expected_mode) << "\"";
    } else {
        event << "null";
    }
}

}  // namespace

CommandAuditLogger::CommandAuditLogger() {
    const char* path = std::getenv("MAVLINK_AUDIT_EVENT_FILE");
    if (path == nullptr || *path == '\0') return;

    std::error_code error;
    const fs::path event_path(path);
    if (!event_path.parent_path().empty()) {
        fs::create_directories(event_path.parent_path(), error);
    }
    output_.open(event_path, std::ios::out | std::ios::app);
}

void CommandAuditLogger::write_event(const std::string& json) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!output_.is_open()) return;
    output_ << json << '\n';
    output_.flush();
}

void CommandAuditLogger::record_decision(
    const safety::CommandDecision& decision,
    const safety::SafetyMonitor::Snapshot& authority) {
    const EventTime event_time = now();
    const std::string block_reason = safety::gate_block_reason_name(decision.block_reason);
    const char* lock_reason = nullable_reason(decision.control_lock_reason);

    std::ostringstream event;
    event << "{\"event\":\"command_decision\",\"source\":\"command_gate\","
          << "\"timestamp\":\"" << event_time.iso << "\",\"timestamp_unix_ms\":"
          << event_time.unix_ms << ",\"command_type\":\""
          << safety::command_type_name(decision.type) << "\",\"allowed\":"
          << (decision.allowed ? "true" : "false") << ",\"sent\":"
          << (decision.sent ? "true" : "false") << ",\"blocked\":"
          << (decision.allowed ? "false" : "true") << ",\"block_reason\":";
    if (decision.allowed) {
        event << "null";
    } else {
        event << "\"" << block_reason << "\"";
    }
    event << ",\"mission_operation\":";
    if (decision.mission_operation_name.empty()) {
        event << "null";
    } else {
        event << "\"" << json_escape(decision.mission_operation_name) << "\"";
    }
    event << ",\"control_lock_reason\":";
    if (lock_reason == nullptr) {
        event << "null";
    } else {
        event << "\"" << json_escape(lock_reason) << "\"";
    }
    event << ",\"vehicle_affecting\":true";
    append_authority_snapshot(event, authority);
    event << "}";
    write_event(event.str());
}

void CommandAuditLogger::record_control_lock(
    const std::string& reason,
    const safety::SafetyMonitor::Snapshot& authority) {
    const EventTime event_time = now();
    std::ostringstream event;
    event << "{\"event\":\"control_lock\",\"source\":\"safety_monitor\","
          << "\"timestamp\":\"" << event_time.iso << "\",\"timestamp_unix_ms\":"
          << event_time.unix_ms << ",\"command_type\":null,\"allowed\":false,"
          << "\"sent\":false,\"blocked\":true,\"block_reason\":\"CONTROL_LOCKED\","
          << "\"control_lock_reason\":\"" << json_escape(reason)
          << "\",\"vehicle_affecting\":false";
    append_authority_snapshot(event, authority);
    event << "}";
    write_event(event.str());
}

void CommandAuditLogger::record_session_started(
    const safety::SafetyMonitor::Snapshot& authority) {
    const EventTime event_time = now();
    std::ostringstream event;
    event << "{\"event\":\"authority_session_started\",\"source\":\"safety_monitor\","
          << "\"timestamp\":\"" << event_time.iso << "\",\"timestamp_unix_ms\":"
          << event_time.unix_ms;
    append_authority_snapshot(event, authority);
    event << "}";
    write_event(event.str());
}

void CommandAuditLogger::record_phase_event(const app::FlightPhaseEvent& phase_event) {
    const EventTime event_time = now();
    const auto timestamp_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                  phase_event.timestamp.time_since_epoch())
                                  .count();
    std::ostringstream event;
    event << "{\"event\":\"state_transition\",\"source\":\"flight_mission_app\","
          << "\"timestamp\":\"" << event_time.iso << "\",\"timestamp_unix_ms\":"
          << event_time.unix_ms << ",\"monotonic_timestamp_ms\":" << timestamp_ms
          << ",\"sequence\":" << phase_event.sequence << ",\"previous\":\""
          << app::flight_phase_name(phase_event.previous) << "\",\"current\":\""
          << app::flight_phase_name(phase_event.current) << "\",\"reason\":\""
          << json_escape(phase_event.reason) << "\"}";
    write_event(event.str());
}

void CommandAuditLogger::record_algorithm_state(const std::string& algorithm,
                                                const std::string& previous_state,
                                                const std::string& current_state,
                                                double elapsed_sec, bool tracking,
                                                double observation_age_ms) {
    const EventTime event_time = now();
    std::ostringstream event;
    event << "{\"event\":\"algorithm_state_transition\",\"source\":\"guidance\",\"timestamp\":\""
          << event_time.iso
          << "\",\"timestamp_unix_ms\":" << event_time.unix_ms
          << ",\"sequence\":" << next_algorithm_sequence_++
          << ",\"algorithm\":\"" << json_escape(algorithm)
          << "\",\"previous\":\"" << json_escape(previous_state)
          << "\",\"current\":\"" << json_escape(current_state)
          << "\",\"elapsed_sec\":" << elapsed_sec
          << ",\"tracking\":" << (tracking ? "true" : "false")
          << ",\"observation_age_ms\":" << observation_age_ms << "}";
    write_event(event.str());
}

}  // namespace logging
