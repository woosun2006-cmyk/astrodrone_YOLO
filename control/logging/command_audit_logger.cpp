#include "command_audit_logger.hpp"

#include <chrono>
#include <ctime>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <sstream>

namespace logging {
namespace {

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
    const safety::ControlAuthority::Snapshot& authority) {
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
    const std::filesystem::path event_path(path);
    if (!event_path.parent_path().empty()) {
        std::filesystem::create_directories(event_path.parent_path(), error);
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
    const autopilot::CommandDecision& decision,
    const safety::ControlAuthority::Snapshot& authority) {
    const EventTime event_time = now();
    const std::string block_reason = autopilot::gate_block_reason_name(decision.block_reason);
    const char* lock_reason = nullable_reason(decision.control_lock_reason);

    std::ostringstream event;
    event << "{\"event\":\"command_decision\",\"source\":\"command_gate\","
          << "\"timestamp\":\"" << event_time.iso << "\",\"timestamp_unix_ms\":"
          << event_time.unix_ms << ",\"command_type\":\""
          << autopilot::command_type_name(decision.type) << "\",\"allowed\":"
          << (decision.allowed ? "true" : "false") << ",\"sent\":"
          << (decision.sent ? "true" : "false") << ",\"blocked\":"
          << (decision.allowed ? "false" : "true") << ",\"block_reason\":";
    if (decision.allowed) {
        event << "null";
    } else {
        event << "\"" << block_reason << "\"";
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
    const safety::ControlAuthority::Snapshot& authority) {
    const EventTime event_time = now();
    std::ostringstream event;
    event << "{\"event\":\"control_lock\",\"source\":\"control_authority\","
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
    const safety::ControlAuthority::Snapshot& authority) {
    const EventTime event_time = now();
    std::ostringstream event;
    event << "{\"event\":\"authority_session_started\",\"source\":\"control_authority\","
          << "\"timestamp\":\"" << event_time.iso << "\",\"timestamp_unix_ms\":"
          << event_time.unix_ms;
    append_authority_snapshot(event, authority);
    event << "}";
    write_event(event.str());
}

void CommandAuditLogger::record_state_event(const safety::FlightStateEvent& state_event) {
    const EventTime event_time = now();
    const auto timestamp_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                  state_event.timestamp.time_since_epoch())
                                  .count();
    std::ostringstream event;
    event << "{\"event\":\"state_transition\",\"source\":\"control_state_machine\"," 
          << "\"timestamp\":\"" << event_time.iso << "\",\"timestamp_unix_ms\":"
          << event_time.unix_ms << ",\"monotonic_timestamp_ms\":" << timestamp_ms
          << ",\"sequence\":" << state_event.sequence << ",\"previous\":\""
          << safety::flight_state_name(state_event.previous) << "\",\"current\":\""
          << safety::flight_state_name(state_event.current) << "\",\"reason\":\""
          << json_escape(state_event.reason) << "\"}";
    write_event(event.str());
}

}  // namespace logging
