#include "safety_monitor.hpp"

#include <algorithm>
#include <cctype>
#include <iomanip>
#include <limits>
#include <map>
#include <sstream>

#include "../drone_lib.hpp"

namespace safety {
namespace {

CommandRequest mission_request(MissionOperation operation, std::string name) {
    CommandRequest request;
    request.type = CommandType::MissionProtocol;
    request.mission_operation = operation;
    request.mission_operation_name = std::move(name);
    request.created_at = CommandRequest::Clock::now();
    return request;
}

CommandRequest velocity_request(double x, double y, double z, double yaw, bool body,
                                CommandRequest::Clock::time_point created_at,
                                CommandRequest::Clock::time_point valid_until) {
    CommandRequest request;
    request.type = CommandType::VelocitySetpoint;
    request.vx = x;
    request.vy = y;
    request.vz = z;
    request.yaw_rate = yaw;
    request.body_frame = body;
    request.created_at = created_at;
    request.valid_until = valid_until;
    return request;
}

}  // namespace

CommandRequest CommandRequest::set_mode(std::string value, Clock::time_point created_at,
                                         Clock::time_point valid_until) {
    CommandRequest request;
    request.type = CommandType::SetMode;
    request.mode = std::move(value);
    request.created_at = created_at;
    request.valid_until = valid_until;
    return request;
}

CommandRequest CommandRequest::arm_disarm(bool value, Clock::time_point created_at,
                                           Clock::time_point valid_until) {
    CommandRequest request;
    request.type = CommandType::ArmDisarm;
    request.arm = value;
    request.created_at = created_at;
    request.valid_until = valid_until;
    return request;
}

CommandRequest CommandRequest::takeoff(double altitude, Clock::time_point created_at,
                                       Clock::time_point valid_until) {
    CommandRequest request;
    request.type = CommandType::Takeoff;
    request.altitude_m = altitude;
    request.created_at = created_at;
    request.valid_until = valid_until;
    return request;
}

CommandRequest CommandRequest::velocity_local(double x, double y, double z, double yaw,
                                               Clock::time_point created_at,
                                               Clock::time_point valid_until) {
    return velocity_request(x, y, z, yaw, false, created_at, valid_until);
}

CommandRequest CommandRequest::velocity_body(double x, double y, double z, double yaw,
                                              Clock::time_point created_at,
                                              Clock::time_point valid_until) {
    return velocity_request(x, y, z, yaw, true, created_at, valid_until);
}

CommandRequest CommandRequest::zero_velocity(Clock::time_point created_at,
                                              Clock::time_point valid_until) {
    return velocity_body(0.0, 0.0, 0.0, 0.0, created_at, valid_until);
}

CommandRequest CommandRequest::land(Clock::time_point created_at,
                                    Clock::time_point valid_until) {
    CommandRequest request;
    request.type = CommandType::Land;
    request.created_at = created_at;
    request.valid_until = valid_until;
    return request;
}

CommandRequest CommandRequest::mission_clear() {
    return mission_request(MissionOperation::Clear, "MISSION_CLEAR_ALL");
}

CommandRequest CommandRequest::mission_count(uint16_t count) {
    CommandRequest request = mission_request(MissionOperation::Count, "MISSION_COUNT");
    request.mission_count_value = count;
    return request;
}

CommandRequest CommandRequest::mission_item(uint16_t seq, uint16_t command, uint8_t frame,
                                             int32_t x, int32_t y, float altitude,
                                             uint8_t current) {
    CommandRequest request = mission_request(MissionOperation::Item, "MISSION_ITEM_INT");
    request.mission_seq = seq;
    request.mission_command = command;
    request.mission_frame = frame;
    request.mission_x = x;
    request.mission_y = y;
    request.mission_altitude_m = altitude;
    request.mission_current = current;
    return request;
}

CommandRequest CommandRequest::mission_request_list() {
    return mission_request(MissionOperation::RequestList, "MISSION_REQUEST_LIST");
}

CommandRequest CommandRequest::mission_request_item(uint16_t seq) {
    CommandRequest request = mission_request(MissionOperation::RequestItem, "MISSION_REQUEST_INT");
    request.mission_seq = seq;
    return request;
}

CommandRequest CommandRequest::mission_set_current(uint16_t seq) {
    CommandRequest request = mission_request(MissionOperation::SetCurrent, "MISSION_SET_CURRENT");
    request.mission_seq = seq;
    return request;
}

const char* command_type_name(CommandType type) {
    switch (type) {
        case CommandType::SetMode: return "SET_MODE";
        case CommandType::ArmDisarm: return "ARM_DISARM";
        case CommandType::Takeoff: return "TAKEOFF";
        case CommandType::VelocitySetpoint: return "VELOCITY_SETPOINT";
        case CommandType::Land: return "LAND";
        case CommandType::MissionProtocol: return "MISSION_PROTOCOL";
    }
    return "UNKNOWN";
}

const char* gate_block_reason_name(GateBlockReason reason) {
    switch (reason) {
        case GateBlockReason::None: return "NONE";
        case GateBlockReason::AuthorityDisabled: return "AUTHORITY_DISABLED";
        case GateBlockReason::CommandModeDisabled: return "COMMAND_MODE_DISABLED";
        case GateBlockReason::ControlLocked: return "CONTROL_LOCKED";
        case GateBlockReason::PreflightNotReady: return "PREFLIGHT_NOT_READY";
        case GateBlockReason::RequestExpired: return "REQUEST_EXPIRED";
        case GateBlockReason::InvalidRequest: return "INVALID_REQUEST";
    }
    return "UNKNOWN";
}

CommandAuthority SafetyMonitor::authority() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return authority_;
}

void SafetyMonitor::set_authority(CommandAuthority authority) {
    std::lock_guard<std::mutex> lock(mutex_);
    authority_ = std::move(authority);
}

void SafetyMonitor::set_control_locked(bool locked, std::string reason) {
    std::lock_guard<std::mutex> lock(mutex_);
    authority_.control_locked = locked;
    if (locked && !reason.empty()) authority_.control_lock_reason = std::move(reason);
    if (!locked) authority_.control_lock_reason.clear();
}

void SafetyMonitor::set_preflight_required(bool required) {
    std::lock_guard<std::mutex> lock(mutex_);
    authority_.preflight_required = required;
}

void SafetyMonitor::set_preflight_ready(bool ready) {
    std::lock_guard<std::mutex> lock(mutex_);
    authority_.preflight_ready = ready;
}

void SafetyMonitor::set_real_flight_approved(bool approved) {
    std::lock_guard<std::mutex> lock(mutex_);
    authority_.real_flight_approved = approved;
}

GateBlockReason SafetyMonitor::authority_block_reason() const {
    if (!authority_.automation_enabled) return GateBlockReason::AuthorityDisabled;
    if (!authority_.vehicle_commands_enabled) return GateBlockReason::CommandModeDisabled;
    if (!authority_.real_flight_approved) return GateBlockReason::AuthorityDisabled;
    if (authority_.control_locked) return GateBlockReason::ControlLocked;
    if (authority_.preflight_required && !authority_.preflight_ready) {
        return GateBlockReason::PreflightNotReady;
    }
    return GateBlockReason::None;
}

GateBlockReason SafetyMonitor::block_reason(const CommandRequest& request,
                                            Clock::time_point now) const {
    const auto authority_reason = authority_block_reason();
    if (authority_reason != GateBlockReason::None) return authority_reason;
    if (request.type == CommandType::ArmDisarm && request.arm &&
        !authority_.arm_commands_enabled) {
        return GateBlockReason::AuthorityDisabled;
    }
    if (request.valid_until != Clock::time_point::max() && now >= request.valid_until) {
        return GateBlockReason::RequestExpired;
    }
    if (request.type == CommandType::SetMode && request.mode.empty()) {
        return GateBlockReason::InvalidRequest;
    }
    if (request.type == CommandType::Takeoff &&
        (!std::isfinite(request.altitude_m) || request.altitude_m < 0.0)) {
        return GateBlockReason::InvalidRequest;
    }
    if (request.type == CommandType::VelocitySetpoint &&
        (!std::isfinite(request.vx) || !std::isfinite(request.vy) ||
         !std::isfinite(request.vz) || !std::isfinite(request.yaw_rate))) {
        return GateBlockReason::InvalidRequest;
    }
    if (request.type == CommandType::MissionProtocol) {
        if (request.mission_operation_name.empty()) return GateBlockReason::InvalidRequest;
        if (request.mission_operation == MissionOperation::Count &&
            request.mission_count_value == 0) return GateBlockReason::InvalidRequest;
        if (request.mission_operation == MissionOperation::Item &&
            (!std::isfinite(request.mission_altitude_m) || request.mission_command == 0)) {
            return GateBlockReason::InvalidRequest;
        }
    }
    return GateBlockReason::None;
}

bool SafetyMonitor::can_arm() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return authority_block_reason() == GateBlockReason::None;
}

bool SafetyMonitor::can_change_mode(const std::string& mode) const {
    if (mode.empty()) return false;
    std::lock_guard<std::mutex> lock(mutex_);
    return authority_block_reason() == GateBlockReason::None;
}

bool SafetyMonitor::can_send_velocity() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return authority_block_reason() == GateBlockReason::None;
}

bool SafetyMonitor::can_takeoff() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return authority_block_reason() == GateBlockReason::None;
}

bool SafetyMonitor::can_land() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return authority_block_reason() == GateBlockReason::None;
}

bool SafetyMonitor::can_upload_mission() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return authority_block_reason() == GateBlockReason::None;
}

CommandDecision SafetyMonitor::authorize(const CommandRequest& request) const {
    std::lock_guard<std::mutex> lock(mutex_);
    CommandDecision decision;
    decision.type = request.type;
    decision.evaluated_at = Clock::now();
    decision.block_reason = block_reason(request, decision.evaluated_at);
    if (decision.block_reason == GateBlockReason::ControlLocked) {
        decision.control_lock_reason = authority_.control_lock_reason;
    }
    decision.mission_operation_name = request.mission_operation_name;
    decision.allowed = decision.block_reason == GateBlockReason::None;
    return decision;
}

SafetyStatus SafetyMonitor::health_status(const HealthState& state,
                                          const HealthLimit& limits,
                                          Clock::time_point now) const {
    SafetyStatus status;
    status.reasons = evaluate_health_breach(state, limits, now);
    status.healthy = status.reasons.empty();
    return status;
}

std::vector<std::string> evaluate_health_breach(
    const HealthState& state, const HealthLimit& limits,
    std::chrono::steady_clock::time_point now) {
    std::vector<std::string> reasons;
    const double heartbeat_gap = std::chrono::duration<double>(now - state.last_heartbeat).count();
    if (heartbeat_gap > limits.max_heartbeat_gap_sec) {
        std::ostringstream text;
        text << "heartbeat gap " << std::fixed << std::setprecision(1) << heartbeat_gap
             << "s > " << limits.max_heartbeat_gap_sec << "s";
        reasons.push_back(text.str());
    }
    if (state.have_battery && state.battery_percent >= 0 &&
        state.battery_percent < limits.min_battery_percent) {
        reasons.push_back("battery " + std::to_string(state.battery_percent) + "% < " +
                          std::to_string(limits.min_battery_percent) + "%");
    }
    if (state.have_battery && state.battery_voltage_v >= 0 &&
        state.battery_voltage_v < limits.min_battery_voltage_v) {
        std::ostringstream text;
        text << "battery " << std::fixed << std::setprecision(2) << state.battery_voltage_v
             << "V < " << limits.min_battery_voltage_v << "V";
        reasons.push_back(text.str());
    }
    if (state.have_gps && state.satellites != 255 &&
        (state.fix_type < limits.min_fix_type || state.satellites < limits.min_satellites)) {
        reasons.push_back("gps fix_type=" + std::to_string(static_cast<int>(state.fix_type)) +
                          " satellites=" + std::to_string(static_cast<int>(state.satellites)));
    }
    if (state.have_sys_status && limits.require_prearm_healthy && !state.prearm_healthy) {
        reasons.push_back("prearm check unhealthy");
    }
    if (state.have_sys_status && limits.require_normal_state &&
        (state.system_status == MAV_STATE_CRITICAL || state.system_status == MAV_STATE_EMERGENCY)) {
        reasons.push_back("system_status=" + std::to_string(static_cast<int>(state.system_status)) +
                          " (CRITICAL/EMERGENCY)");
    }
    if (state.have_ekf &&
        (state.ekf_pos_horiz_variance > limits.ekf_pos_horiz_variance_max ||
         state.ekf_velocity_variance > limits.ekf_velocity_variance_max)) {
        std::ostringstream text;
        text << "ekf pos_horiz_variance=" << std::fixed << std::setprecision(2)
             << state.ekf_pos_horiz_variance << " velocity_variance="
             << state.ekf_velocity_variance;
        reasons.push_back(text.str());
    }
    return reasons;
}

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

SafetyMonitor::SafetyMonitor(PreflightPolicy policy) : preflight_policy_(policy) {}

void SafetyMonitor::configure_preflight(PreflightPolicy policy) {
    preflight_policy_ = std::move(policy);
    preflight_ready_ = false;
    preflight_reasons_.clear();
    prearm_ok_ = false;
}

void SafetyMonitor::observe_preflight(const PreflightSample& sample) {
    const auto now = sample.now;
    if (sample.heartbeat_event) {
        if (sample.valid_autopilot_heartbeat) {
            if (!last_heartbeat_valid_ || heartbeat_stable_since_ == Clock::time_point{}) {
                heartbeat_stable_since_ = now;
                heartbeat_count_ = 1;
            } else {
                ++heartbeat_count_;
            }
            last_heartbeat_at_ = now;
            last_heartbeat_valid_ = true;
            heartbeat_fresh_ = true;
        } else {
            heartbeat_count_ = 0;
            heartbeat_stable_since_ = Clock::time_point{};
            last_heartbeat_valid_ = false;
            heartbeat_fresh_ = false;
        }
    } else {
        heartbeat_fresh_ = sample.heartbeat_fresh;
    }
    health_ok_ = sample.gps_ok && sample.ekf_ok && sample.battery_valid;
    prearm_ok_ = !preflight_policy_.require_prearm_healthy || sample.prearm_healthy;
    rc_policy_ok_ = sample.rc_policy_ok;
    telemetry_fresh_ = sample.telemetry_fresh;
    yolo_ready_ = sample.yolo_ready;
    if (sample.camera_frame) {
        ++camera_frame_count_;
        last_camera_frame_at_ = now;
        camera_frame_ = true;
    }
    if (health_ok_) {
        if (health_stable_since_ == Clock::time_point{}) health_stable_since_ = now;
    } else {
        health_stable_since_ = Clock::time_point{};
    }
    recompute_preflight(now);
}

void SafetyMonitor::poll_preflight(Clock::time_point now) {
    if (last_heartbeat_at_ != Clock::time_point{} &&
        std::chrono::duration<double>(now - last_heartbeat_at_).count() >
            preflight_policy_.heartbeat_freshness_sec) {
        last_heartbeat_valid_ = false;
        heartbeat_count_ = 0;
        heartbeat_stable_since_ = Clock::time_point{};
        heartbeat_fresh_ = false;
    }
    if (last_camera_frame_at_ != Clock::time_point{} &&
        std::chrono::duration<double>(now - last_camera_frame_at_).count() >
            preflight_policy_.telemetry_freshness_sec) {
        camera_frame_ = false;
    }
    recompute_preflight(now);
}

void SafetyMonitor::recompute_preflight(Clock::time_point now) {
    preflight_reasons_.clear();
    const bool count_ok = heartbeat_count_ >= preflight_policy_.required_heartbeat_count;
    const bool time_ok = heartbeat_stable_since_ != Clock::time_point{} &&
        std::chrono::duration<double>(now - heartbeat_stable_since_).count() >=
            preflight_policy_.required_heartbeat_stable_sec;
    if (!heartbeat_fresh_ || !count_ok || !time_ok) {
        preflight_reasons_.emplace_back("autopilot HEARTBEAT stability incomplete");
    }
    const bool health_time_ok = health_stable_since_ != Clock::time_point{} &&
        std::chrono::duration<double>(now - health_stable_since_).count() >=
            preflight_policy_.required_health_stable_sec;
    if (!health_ok_ || !health_time_ok) {
        preflight_reasons_.emplace_back("GPS/EKF/battery stability incomplete");
    }
    if (!prearm_ok_) preflight_reasons_.emplace_back("ArduPilot pre-arm health incomplete");
    if (preflight_policy_.require_rc_policy && !rc_policy_ok_) {
        preflight_reasons_.emplace_back("RC policy failed");
    }
    if (!telemetry_fresh_) preflight_reasons_.emplace_back("telemetry is not fresh");
    if (preflight_policy_.require_vision && !yolo_ready_) {
        preflight_reasons_.emplace_back("YOLO readiness incomplete");
    }
    if (preflight_policy_.require_vision &&
        camera_frame_count_ < preflight_policy_.required_camera_frames) {
        preflight_reasons_.emplace_back("camera frame stability incomplete");
    }
    preflight_ready_ = preflight_reasons_.empty();
    authority_.preflight_ready = preflight_ready_;
}

bool SafetyMonitor::preflight_ready() const { return preflight_ready_; }
std::size_t SafetyMonitor::consecutive_heartbeats() const { return heartbeat_count_; }
std::size_t SafetyMonitor::camera_frames() const { return camera_frame_count_; }
const std::vector<std::string>& SafetyMonitor::preflight_reasons() const {
    return preflight_reasons_;
}

bool SafetyMonitor::is_known_mode(uint32_t custom_mode) const {
    const auto& modes = copter_mode_mapping();
    return std::any_of(modes.begin(), modes.end(), [custom_mode](const auto& entry) {
        return entry.second == custom_mode;
    });
}

bool SafetyMonitor::is_ambiguous_safety_mode(uint32_t custom_mode) const {
    const auto& modes = copter_mode_mapping();
    for (const char* name : {"RTL", "LAND", "AUTO_RTL", "SMART_RTL"}) {
        const auto it = modes.find(name);
        if (it != modes.end() && it->second == custom_mode) return true;
    }
    return false;
}

void SafetyMonitor::latch(ControlLockReason reason) {
    if (authority_.control_locked) return;
    authority_.control_locked = true;
    lock_reason_ = reason;
    authority_.control_lock_reason = control_lock_reason_name(reason);
}

void SafetyMonitor::observe_heartbeat(const mavlink_heartbeat_t& heartbeat, Clock::time_point now) {
    poll(now);
    const bool had_mode = have_actual_mode_;
    const uint32_t previous = actual_mode_;
    const bool changed = had_mode && heartbeat.custom_mode != previous;
    previous_mode_ = had_mode ? std::optional<uint32_t>(previous) : std::nullopt;
    actual_mode_ = heartbeat.custom_mode;
    have_actual_mode_ = true;
    actual_mode_known_ = is_known_mode(heartbeat.custom_mode);
    actual_system_status_ = heartbeat.system_status;
    last_heartbeat_ = now;
    ++heartbeat_total_;
    if (!baseline_known_) {
        baseline_known_ = true;
        baseline_mode_ = heartbeat.custom_mode;
        return;
    }
    if (!session_started_) return;
    if (latest_mode_request_ && latest_mode_request_->write_succeeded &&
        now >= latest_mode_request_->sent_at) {
        latest_mode_request_->heartbeat_observed_after_request = true;
    }
    // The mode lease bounds how long we wait for confirmation; it does not
    // make a delayed confirmation an external takeover.  SITL/router gaps can
    // deliver the requested mode after the lease has expired.
    if (latest_mode_request_ && latest_mode_request_->write_succeeded &&
        now >= latest_mode_request_->sent_at &&
        heartbeat.custom_mode == latest_mode_request_->expected_custom_mode) {
        latest_mode_request_.reset();
        return;
    }
    if (!changed) return;
    if (!actual_mode_known_) latch(ControlLockReason::UnknownExternalModeChange);
    else if (failsafe_evidence_until_ >= now) latch(ControlLockReason::ArduPilotFailsafeModeChange);
    else if (is_ambiguous_safety_mode(heartbeat.custom_mode)) {
        latch(ControlLockReason::UnknownExternalModeChange);
    } else latch(ControlLockReason::UnexpectedOperatorOrGcsModeChange);
}

void SafetyMonitor::observe_autopilot_state(const autopilot::AutopilotState& state,
                                            Clock::time_point now) {
    if (state.last_heartbeat == Clock::time_point{} ||
        state.last_heartbeat == last_state_heartbeat_) {
        poll(now);
        return;
    }
    last_state_heartbeat_ = state.last_heartbeat;
    mavlink_heartbeat_t heartbeat{};
    heartbeat.custom_mode = state.custom_mode;
    heartbeat.base_mode = state.base_mode;
    heartbeat.system_status = state.system_status;
    observe_heartbeat(heartbeat, state.last_heartbeat);
}

bool SafetyMonitor::begin_automation_session() {
    if (!have_actual_mode_) return false;
    return begin_automation_session(actual_mode_);
}

bool SafetyMonitor::begin_automation_session(uint32_t baseline_mode) {
    if (!baseline_known_ || !have_actual_mode_ || actual_mode_ != baseline_mode) return false;
    baseline_mode_ = actual_mode_;
    previous_mode_ = actual_mode_;
    latest_mode_request_.reset();
    session_started_ = true;
    return true;
}

bool SafetyMonitor::prepare_mode_request(const std::string& mode, Clock::time_point sent_at,
                                         Clock::time_point expires_at) {
    if (authority_.control_locked) return false;
    const auto it = copter_mode_mapping().find(mode);
    if (it == copter_mode_mapping().end()) return false;
    ModeRequestRecord record;
    record.expected_mode = mode;
    record.expected_custom_mode = it->second;
    if (have_actual_mode_) {
        record.mode_at_request = actual_mode_;
        record.mode_at_request_known = true;
    }
    record.sent_at = sent_at;
    record.expires_at = expires_at == Clock::time_point::max()
                            ? sent_at + mode_request_lease_ : expires_at;
    latest_mode_request_ = std::move(record);
    return true;
}

void SafetyMonitor::record_mode_request(const CommandRequest& request, bool write_succeeded) {
    if (request.type != CommandType::SetMode || authority_.control_locked) return;
    const auto it = copter_mode_mapping().find(request.mode);
    if (it == copter_mode_mapping().end()) return;
    ModeRequestRecord record;
    record.expected_mode = request.mode;
    record.expected_custom_mode = it->second;
    if (have_actual_mode_) {
        record.mode_at_request = actual_mode_;
        record.mode_at_request_known = true;
    }
    record.sent_at = request.created_at == Clock::time_point{} ? Clock::now() : request.created_at;
    record.expires_at = request.valid_until == Clock::time_point::max()
                            ? record.sent_at + mode_request_lease_ : request.valid_until;
    record.write_succeeded = write_succeeded;
    latest_mode_request_ = std::move(record);
}

void SafetyMonitor::observe_failsafe_evidence(const std::string& evidence, Clock::time_point now) {
    std::string normalized;
    for (unsigned char character : evidence) normalized.push_back(static_cast<char>(std::tolower(character)));
    const bool failsafe = normalized.find("failsafe") != std::string::npos ||
                          normalized.find("fail-safe") != std::string::npos;
    const bool clear = normalized.find("clear") != std::string::npos ||
                       normalized.find("recover") != std::string::npos ||
                       normalized.find("resolve") != std::string::npos;
    if (failsafe && !clear) failsafe_evidence_until_ = now + mode_request_lease_;
}

void SafetyMonitor::poll(Clock::time_point now) {
    if (!latest_mode_request_ || authority_.control_locked || now < latest_mode_request_->expires_at) return;
    const ModeRequestRecord expired_request = *latest_mode_request_;
    // Expiration only ends the observation lease. It is not itself evidence
    // of an external mode change. A matching current mode means the vehicle
    // is still in the mode we requested, so leave the safety latch untouched.
    if (!expired_request.write_succeeded) {
        latest_mode_request_.reset();
        return;
    }
    // If no heartbeat arrived after the request, the cached mode may predate
    // the request. A lease timeout alone is not evidence of an external mode
    // change; the normal heartbeat freshness gate handles that failure.
    if (!expired_request.heartbeat_observed_after_request) return;
    if (have_actual_mode_ && actual_mode_ == expired_request.expected_custom_mode) {
        latest_mode_request_.reset();
        return;
    }
    // A delayed or rejected request can leave the vehicle in the exact mode
    // it had before the request. That is not an external takeover; an actual
    // mode change is handled by observe_heartbeat() when the changed
    // heartbeat arrives.
    if (expired_request.mode_at_request_known && have_actual_mode_ &&
        actual_mode_ == expired_request.mode_at_request) {
        // Keep the request pending while the vehicle is still in the mode it
        // had before the request.  A delayed heartbeat can then confirm the
        // requested mode without being classified as an external change.
        return;
    }
    latest_mode_request_.reset();
    if (failsafe_evidence_until_ >= now) latch(ControlLockReason::ArduPilotFailsafeModeChange);
    else latch(ControlLockReason::UnknownExternalModeChange);
}

bool SafetyMonitor::baseline_known() const { return baseline_known_; }
uint32_t SafetyMonitor::baseline_mode() const { return baseline_mode_; }
uint32_t SafetyMonitor::actual_mode() const { return actual_mode_; }
bool SafetyMonitor::actual_mode_known() const { return actual_mode_known_; }
bool SafetyMonitor::automation_session_started() const { return session_started_; }
bool SafetyMonitor::control_locked() const { return authority_.control_locked; }
ControlLockReason SafetyMonitor::lock_reason() const { return lock_reason_; }
uint64_t SafetyMonitor::heartbeat_count() const { return heartbeat_total_; }
SafetyMonitor::Clock::time_point SafetyMonitor::last_heartbeat() const { return last_heartbeat_; }
const std::optional<SafetyMonitor::ModeRequestRecord>& SafetyMonitor::latest_mode_request() const {
    return latest_mode_request_;
}
SafetyMonitor::Snapshot SafetyMonitor::snapshot() const {
    Snapshot result;
    result.previous_mode = previous_mode_;
    if (have_actual_mode_) result.current_mode = actual_mode_;
    result.session_started = session_started_;
    if (latest_mode_request_) result.expected_mode = latest_mode_request_->expected_mode;
    return result;
}

}  // namespace safety
