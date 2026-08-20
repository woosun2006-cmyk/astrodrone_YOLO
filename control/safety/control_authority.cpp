#include "control_authority.hpp"

#include <algorithm>
#include <cctype>
#include <map>

#include "../drone_lib.hpp"

namespace safety {

const char* control_lock_reason_name(ControlLockReason reason) {
    switch (reason) {
        case ControlLockReason::None: return "None";
        case ControlLockReason::UnexpectedOperatorOrGcsModeChange:
            return "UnexpectedOperatorOrGcsModeChange";
        case ControlLockReason::ArduPilotFailsafeModeChange:
            return "ArduPilotFailsafeModeChange";
        case ControlLockReason::UnknownExternalModeChange:
            return "UnknownExternalModeChange";
    }
    return "Unknown";
}

ControlAuthority::ControlAuthority(std::chrono::milliseconds mode_request_lease)
    : mode_request_lease_(mode_request_lease) {}

bool ControlAuthority::is_known_mode(uint32_t custom_mode) const {
    const auto& modes = copter_mode_mapping();
    return std::any_of(modes.begin(), modes.end(), [custom_mode](const auto& entry) {
        return entry.second == custom_mode;
    });
}

bool ControlAuthority::is_ambiguous_safety_mode(uint32_t custom_mode) const {
    const auto& modes = copter_mode_mapping();
    for (const char* name : {"RTL", "LAND", "AUTO_RTL", "SMART_RTL"}) {
        auto it = modes.find(name);
        if (it != modes.end() && it->second == custom_mode) return true;
    }
    return false;
}

void ControlAuthority::latch(ControlLockReason reason) {
    if (control_locked_) return;
    control_locked_ = true;
    lock_reason_ = reason;
}

void ControlAuthority::observe_heartbeat(const mavlink_heartbeat_t& heartbeat,
                                         Clock::time_point now) {
    poll(now);

    const bool had_actual_mode = have_actual_mode_;
    const uint32_t previous_mode = actual_mode_;
    const bool mode_changed = had_actual_mode && heartbeat.custom_mode != previous_mode;

    previous_mode_ = had_actual_mode ? std::optional<uint32_t>(previous_mode) : std::nullopt;

    actual_mode_ = heartbeat.custom_mode;
    have_actual_mode_ = true;
    actual_mode_known_ = is_known_mode(heartbeat.custom_mode);
    actual_system_status_ = heartbeat.system_status;
    last_heartbeat_ = now;
    ++heartbeat_count_;

    if (!baseline_known_) {
        baseline_known_ = true;
        baseline_mode_ = heartbeat.custom_mode;
        return;
    }

    // Before the session starts, mode changes are part of setup. The session
    // baseline is explicitly captured by begin_automation_session().
    if (!session_started_) return;

    if (latest_mode_request_ && latest_mode_request_->write_succeeded &&
        now >= latest_mode_request_->sent_at && now < latest_mode_request_->expires_at &&
        heartbeat.custom_mode == latest_mode_request_->expected_custom_mode) {
        latest_mode_request_.reset();
        return;
    }

    if (!mode_changed) return;

    if (!actual_mode_known_) {
        latch(ControlLockReason::UnknownExternalModeChange);
    } else if (failsafe_evidence_until_ >= now) {
        latch(ControlLockReason::ArduPilotFailsafeModeChange);
    } else if (is_ambiguous_safety_mode(heartbeat.custom_mode)) {
        // RTL/LAND-like names can be operator-selected as well as entered by
        // ArduPilot. Without explicit evidence, do not call them failsafe.
        latch(ControlLockReason::UnknownExternalModeChange);
    } else {
        latch(ControlLockReason::UnexpectedOperatorOrGcsModeChange);
    }
}

bool ControlAuthority::begin_automation_session() {
    if (!have_actual_mode_) return false;
    return begin_automation_session(actual_mode_);
}

bool ControlAuthority::begin_automation_session(uint32_t baseline_mode) {
    if (!baseline_known_ || !have_actual_mode_ || actual_mode_ != baseline_mode) return false;

    baseline_mode_ = actual_mode_;
    previous_mode_ = actual_mode_;
    latest_mode_request_.reset();
    session_started_ = true;
    return true;
}

bool ControlAuthority::prepare_mode_request(const std::string& mode,
                                            Clock::time_point sent_at,
                                            Clock::time_point expires_at) {
    if (control_locked_) return false;
    const auto& modes = copter_mode_mapping();
    const auto it = modes.find(mode);
    if (it == modes.end()) return false;

    ModeRequestRecord record;
    record.expected_mode = mode;
    record.expected_custom_mode = it->second;
    record.sent_at = sent_at;
    record.expires_at = expires_at == Clock::time_point::max()
                            ? sent_at + mode_request_lease_
                            : expires_at;
    latest_mode_request_ = std::move(record);
    return true;
}

void ControlAuthority::record_mode_request(const autopilot::CommandRequest& request,
                                           bool write_succeeded) {
    if (request.type != autopilot::CommandType::SetMode || control_locked_) return;

    const auto& modes = copter_mode_mapping();
    auto it = modes.find(request.mode);
    if (it == modes.end()) return;

    ModeRequestRecord record;
    record.expected_mode = request.mode;
    record.expected_custom_mode = it->second;
    record.sent_at = request.created_at;
    record.expires_at = request.valid_until;
    if (record.sent_at == Clock::time_point{}) record.sent_at = Clock::now();
    if (record.expires_at == Clock::time_point::max()) {
        record.expires_at = record.sent_at + mode_request_lease_;
    }
    record.write_succeeded = write_succeeded;
    latest_mode_request_ = record;
}

void ControlAuthority::observe_failsafe_evidence(const std::string& evidence,
                                                 Clock::time_point now) {
    std::string normalized;
    normalized.reserve(evidence.size());
    for (unsigned char character : evidence) {
        normalized.push_back(static_cast<char>(std::tolower(character)));
    }
    const bool mentions_failsafe = normalized.find("failsafe") != std::string::npos ||
                                   normalized.find("fail-safe") != std::string::npos;
    const bool reports_clear = normalized.find("clear") != std::string::npos ||
                               normalized.find("recover") != std::string::npos ||
                               normalized.find("resolve") != std::string::npos;
    if (mentions_failsafe && !reports_clear) {
        failsafe_evidence_until_ = now + mode_request_lease_;
    }
}

void ControlAuthority::poll(Clock::time_point now) {
    if (!latest_mode_request_ || control_locked_) return;
    if (now < latest_mode_request_->expires_at) return;

    // A write that never receives the expected fresh HEARTBEAT is a failed
    // mode transition. With no explicit failsafe evidence, keep the reason
    // conservative and classify it as an unknown external change.
    latest_mode_request_.reset();
    if (failsafe_evidence_until_ >= now) {
        latch(ControlLockReason::ArduPilotFailsafeModeChange);
    } else {
        latch(ControlLockReason::UnknownExternalModeChange);
    }
}

void ControlAuthority::apply_to(autopilot::CommandGate& gate) const {
    if (!control_locked_) return;
    gate.set_control_locked(true, control_lock_reason_name(lock_reason_));
}

ControlAuthority::Snapshot ControlAuthority::snapshot() const {
    Snapshot result;
    result.previous_mode = previous_mode_;
    if (have_actual_mode_) result.current_mode = actual_mode_;
    result.session_started = session_started_;
    if (latest_mode_request_) result.expected_mode = latest_mode_request_->expected_mode;
    return result;
}

}  // namespace safety
