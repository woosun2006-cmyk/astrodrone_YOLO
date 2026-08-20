#pragma once

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>

#include "../autopilot/command_gate.hpp"
#include "../third_party/mavlink/common/mavlink.h"

namespace safety {

enum class ControlLockReason {
    None,
    UnexpectedOperatorOrGcsModeChange,
    ArduPilotFailsafeModeChange,
    UnknownExternalModeChange,
};

const char* control_lock_reason_name(ControlLockReason reason);

class ControlAuthority {
public:
    using Clock = std::chrono::steady_clock;

    struct ModeRequestRecord {
        std::string expected_mode;
        uint32_t expected_custom_mode = 0;
        Clock::time_point sent_at{};
        Clock::time_point expires_at{};
        bool write_succeeded = false;
    };

    struct Snapshot {
        std::optional<uint32_t> previous_mode;
        std::optional<uint32_t> current_mode;
        bool session_started = false;
        std::optional<std::string> expected_mode;
    };

    explicit ControlAuthority(std::chrono::milliseconds mode_request_lease =
                                  std::chrono::seconds(5));

    // The first fresh HEARTBEAT establishes the baseline. It is never treated
    // as an external mode change.
    void observe_heartbeat(const mavlink_heartbeat_t& heartbeat,
                           Clock::time_point now = Clock::now());

    // Call after the baseline has been recorded. Subsequent mode changes are
    // then checked against the latest successful Jetson mode request.
    bool begin_automation_session();
    bool begin_automation_session(uint32_t baseline_mode);

    // Register the mode expected from the next fresh HEARTBEAT before the
    // corresponding vehicle command is sent.
    bool prepare_mode_request(const std::string& mode,
                              Clock::time_point sent_at = Clock::now(),
                              Clock::time_point expires_at = Clock::time_point::max());

    // Records the write result, but does not treat it as a mode transition.
    // A fresh HEARTBEAT must still report expected_mode before expires_at.
    void record_mode_request(const autopilot::CommandRequest& request,
                             bool write_succeeded);

    // Explicit evidence must come from an ArduPilot status/event report, not
    // from the mode name alone. HEARTBEAT mode changes use this evidence only
    // while it is recent.
    void observe_failsafe_evidence(const std::string& evidence,
                                   Clock::time_point now = Clock::now());

    // Advances timeout handling without requiring a new HEARTBEAT.
    void poll(Clock::time_point now = Clock::now());

    // Sticky: this only propagates a lock to the gate. It never unlocks it.
    void apply_to(autopilot::CommandGate& gate) const;

    bool baseline_known() const { return baseline_known_; }
    uint32_t baseline_mode() const { return baseline_mode_; }
    uint32_t actual_mode() const { return actual_mode_; }
    bool actual_mode_known() const { return actual_mode_known_; }
    bool automation_session_started() const { return session_started_; }
    bool control_locked() const { return control_locked_; }
    ControlLockReason lock_reason() const { return lock_reason_; }
    uint64_t heartbeat_count() const { return heartbeat_count_; }
    Clock::time_point last_heartbeat() const { return last_heartbeat_; }
    const std::optional<ModeRequestRecord>& latest_mode_request() const {
        return latest_mode_request_;
    }
    Snapshot snapshot() const;

private:
    bool is_known_mode(uint32_t custom_mode) const;
    bool is_ambiguous_safety_mode(uint32_t custom_mode) const;
    void latch(ControlLockReason reason);

    std::chrono::milliseconds mode_request_lease_;
    bool baseline_known_ = false;
    uint32_t baseline_mode_ = 0;
    bool have_actual_mode_ = false;
    std::optional<uint32_t> previous_mode_;
    bool actual_mode_known_ = false;
    uint32_t actual_mode_ = 0;
    uint8_t actual_system_status_ = MAV_STATE_UNINIT;
    bool session_started_ = false;
    bool control_locked_ = false;
    ControlLockReason lock_reason_ = ControlLockReason::None;
    std::optional<ModeRequestRecord> latest_mode_request_;
    Clock::time_point failsafe_evidence_until_{};
    Clock::time_point last_heartbeat_{};
    uint64_t heartbeat_count_ = 0;
};

}  // namespace safety
