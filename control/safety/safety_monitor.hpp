#pragma once

#include <chrono>
#include <cstdint>
#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "../autopilot/autopilot_state.hpp"
#include "../third_party/mavlink/common/mavlink.h"

namespace safety {

struct HealthLimit {
    long min_battery_percent = 20;
    double min_battery_voltage_v = 14.8;
    double max_heartbeat_gap_sec = 3;
    long min_fix_type = 3;
    long min_satellites = 6;
    bool require_prearm_healthy = true;
    bool require_normal_state = true;
    double poll_rate_hz = 5;
    double land_gap_sec = 5;
    double ekf_pos_horiz_variance_max = 1.0;
    double ekf_velocity_variance_max = 1.0;
};

struct SafetyStatus {
    bool healthy = true;
    std::vector<std::string> reasons;
    explicit operator bool() const { return healthy; }
};

using HealthState = autopilot::AutopilotState;
std::vector<std::string> evaluate_health_breach(
    const HealthState& state, const HealthLimit& limits,
    std::chrono::steady_clock::time_point now);

struct PreflightPolicy {
    std::size_t required_heartbeat_count = 3;
    double required_heartbeat_stable_sec = 2.0;
    double required_health_stable_sec = 1.0;
    std::size_t required_camera_frames = 1;
    bool require_rc_policy = false;
    bool require_vision = false;
    bool require_prearm_healthy = true;
    double heartbeat_freshness_sec = 3.0;
    double telemetry_freshness_sec = 1.0;
};

struct PreflightSample {
    using Clock = std::chrono::steady_clock;
    bool heartbeat_event = false;
    bool valid_autopilot_heartbeat = false;
    bool heartbeat_fresh = false;
    bool gps_ok = false;
    bool ekf_ok = false;
    bool battery_valid = false;
    bool prearm_healthy = false;
    bool rc_policy_ok = true;
    bool telemetry_fresh = false;
    bool yolo_ready = false;
    bool camera_frame = false;
    Clock::time_point now = Clock::now();
};

enum class ControlLockReason {
    None,
    UnexpectedOperatorOrGcsModeChange,
    ArduPilotFailsafeModeChange,
    UnknownExternalModeChange,
};
const char* control_lock_reason_name(ControlLockReason reason);

enum class CommandType { SetMode, ArmDisarm, Takeoff, VelocitySetpoint, Land, MissionProtocol };
enum class MissionOperation { Clear, Count, Item, RequestList, RequestItem, SetCurrent };
enum class GateBlockReason {
    None, AuthorityDisabled, CommandModeDisabled, ControlLocked,
    PreflightNotReady, RequestExpired, InvalidRequest,
};

struct CommandAuthority {
    bool automation_enabled = true;
    bool vehicle_commands_enabled = true;
    bool arm_commands_enabled = true;
    bool control_locked = false;
    std::string control_lock_reason;
    bool preflight_required = false;
    bool preflight_ready = true;
    bool real_flight_approved = true;
};

struct CommandRequest {
    using Clock = std::chrono::steady_clock;
    CommandType type = CommandType::VelocitySetpoint;
    std::string mode;
    bool arm = false;
    double altitude_m = 0.0;
    double vx = 0.0, vy = 0.0, vz = 0.0, yaw_rate = 0.0;
    bool body_frame = true;
    MissionOperation mission_operation = MissionOperation::Clear;
    std::string mission_operation_name;
    uint16_t mission_count_value = 0, mission_seq = 0, mission_command = 0;
    uint8_t mission_frame = MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
    int32_t mission_x = 0, mission_y = 0;
    float mission_altitude_m = 0.0F;
    uint8_t mission_current = 0;
    Clock::time_point created_at{};
    Clock::time_point valid_until = Clock::time_point::max();

    static CommandRequest set_mode(std::string, Clock::time_point = Clock::now(),
                                   Clock::time_point = Clock::time_point::max());
    static CommandRequest arm_disarm(bool, Clock::time_point = Clock::now(),
                                     Clock::time_point = Clock::time_point::max());
    static CommandRequest takeoff(double, Clock::time_point = Clock::now(),
                                  Clock::time_point = Clock::time_point::max());
    static CommandRequest velocity_local(double, double, double, double,
                                         Clock::time_point = Clock::now(),
                                         Clock::time_point = Clock::time_point::max());
    static CommandRequest velocity_body(double, double, double, double,
                                        Clock::time_point = Clock::now(),
                                        Clock::time_point = Clock::time_point::max());
    static CommandRequest zero_velocity(Clock::time_point = Clock::now(),
                                        Clock::time_point = Clock::time_point::max());
    static CommandRequest land(Clock::time_point = Clock::now(),
                               Clock::time_point = Clock::time_point::max());
    static CommandRequest mission_clear();
    static CommandRequest mission_count(uint16_t);
    static CommandRequest mission_item(uint16_t, uint16_t, uint8_t, int32_t, int32_t, float,
                                       uint8_t = 0);
    static CommandRequest mission_request_list();
    static CommandRequest mission_request_item(uint16_t);
    static CommandRequest mission_set_current(uint16_t);
};

struct CommandDecision {
    using Clock = CommandRequest::Clock;
    CommandType type = CommandType::VelocitySetpoint;
    bool allowed = false, sent = false;
    GateBlockReason block_reason = GateBlockReason::None;
    std::string control_lock_reason, mission_operation_name;
    Clock::time_point evaluated_at{};
    explicit operator bool() const { return allowed && sent; }
};
const char* command_type_name(CommandType type);
const char* gate_block_reason_name(GateBlockReason reason);

class SafetyMonitor {
public:
    using Clock = std::chrono::steady_clock;
    struct ModeRequestRecord {
        std::string expected_mode;
        uint32_t expected_custom_mode = 0;
        uint32_t mode_at_request = 0;
        bool mode_at_request_known = false;
        Clock::time_point sent_at{}, expires_at{};
        bool write_succeeded = false;
        bool heartbeat_observed_after_request = false;
    };
    struct Snapshot {
        std::optional<uint32_t> previous_mode, current_mode;
        bool session_started = false;
        std::optional<std::string> expected_mode;
    };

    explicit SafetyMonitor(PreflightPolicy policy = {});
    CommandAuthority authority() const;
    void set_authority(CommandAuthority);
    void set_control_locked(bool, std::string reason = {});
    void set_preflight_required(bool);
    void set_preflight_ready(bool);
    void set_real_flight_approved(bool);
    bool can_arm() const;
    bool can_change_mode(const std::string&) const;
    bool can_send_velocity() const;
    bool can_takeoff() const;
    bool can_land() const;
    bool can_upload_mission() const;
    CommandDecision authorize(const CommandRequest&) const;

    // Test/tooling hook that preserves the old CommandGate atomic check-and-send
    // semantics while keeping the policy in SafetyMonitor.
    template <typename SendFn>
    CommandDecision authorize_and_run(const CommandRequest& request, SendFn&& send) const {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto now = CommandRequest::Clock::now();
        CommandDecision decision;
        decision.type = request.type;
        decision.evaluated_at = now;
        decision.block_reason = block_reason(request, now);
        if (decision.block_reason == GateBlockReason::ControlLocked) {
            decision.control_lock_reason = authority_.control_lock_reason;
        }
        decision.mission_operation_name = request.mission_operation_name;
        if (decision.block_reason != GateBlockReason::None) return decision;
        decision.allowed = true;
        decision.sent = static_cast<bool>(send());
        return decision;
    }

    SafetyStatus health_status(const HealthState&, const HealthLimit&, Clock::time_point) const;
    void configure_preflight(PreflightPolicy);
    void observe_preflight(const PreflightSample&);
    void poll_preflight(Clock::time_point now = Clock::now());
    bool preflight_ready() const;
    std::size_t consecutive_heartbeats() const;
    std::size_t camera_frames() const;
    const std::vector<std::string>& preflight_reasons() const;

    void observe_heartbeat(const mavlink_heartbeat_t&, Clock::time_point = Clock::now());
    void observe_autopilot_state(const autopilot::AutopilotState&,
                                 Clock::time_point = Clock::now());
    bool begin_automation_session();
    bool begin_automation_session(uint32_t baseline_mode);
    bool prepare_mode_request(const std::string&, Clock::time_point = Clock::now(),
                              Clock::time_point = Clock::time_point::max());
    void record_mode_request(const CommandRequest&, bool write_succeeded);
    void observe_failsafe_evidence(const std::string&, Clock::time_point = Clock::now());
    void poll(Clock::time_point now = Clock::now());
    bool baseline_known() const;
    uint32_t baseline_mode() const;
    uint32_t actual_mode() const;
    bool actual_mode_known() const;
    bool automation_session_started() const;
    bool control_locked() const;
    ControlLockReason lock_reason() const;
    uint64_t heartbeat_count() const;
    Clock::time_point last_heartbeat() const;
    const std::optional<ModeRequestRecord>& latest_mode_request() const;
    Snapshot snapshot() const;

private:
    GateBlockReason authority_block_reason() const;
    GateBlockReason block_reason(const CommandRequest&, Clock::time_point) const;
    void recompute_preflight(Clock::time_point);
    bool is_known_mode(uint32_t) const;
    bool is_ambiguous_safety_mode(uint32_t) const;
    void latch(ControlLockReason);

    mutable std::mutex mutex_;
    CommandAuthority authority_{};
    PreflightPolicy preflight_policy_{};
    bool last_heartbeat_valid_ = false, heartbeat_fresh_ = false, health_ok_ = false;
    bool prearm_ok_ = false;
    bool rc_policy_ok_ = true, telemetry_fresh_ = false, yolo_ready_ = false, camera_frame_ = false;
    std::size_t heartbeat_count_ = 0, camera_frame_count_ = 0;
    Clock::time_point heartbeat_stable_since_{}, health_stable_since_{}, last_heartbeat_at_{}, last_camera_frame_at_{};
    std::vector<std::string> preflight_reasons_;
    bool preflight_ready_ = false;
    std::chrono::milliseconds mode_request_lease_ = std::chrono::seconds(5);
    bool baseline_known_ = false, have_actual_mode_ = false, actual_mode_known_ = false;
    uint32_t baseline_mode_ = 0, actual_mode_ = 0;
    std::optional<uint32_t> previous_mode_;
    uint8_t actual_system_status_ = MAV_STATE_UNINIT;
    bool session_started_ = false;
    ControlLockReason lock_reason_ = ControlLockReason::None;
    std::optional<ModeRequestRecord> latest_mode_request_;
    Clock::time_point failsafe_evidence_until_{}, last_heartbeat_{};
    Clock::time_point last_state_heartbeat_{};
    uint64_t heartbeat_total_ = 0;
};

}  // namespace safety
