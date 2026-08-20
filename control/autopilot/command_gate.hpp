#pragma once

#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <mutex>
#include <string>
#include <utility>

namespace autopilot {

enum class CommandType {
    SetMode,
    ArmDisarm,
    Takeoff,
    VelocitySetpoint,
    Land,
};

enum class GateBlockReason {
    None,
    AuthorityDisabled,
    CommandModeDisabled,
    ControlLocked,
    PreflightNotReady,
    RequestExpired,
    InvalidRequest,
};

struct CommandAuthority {
    bool automation_enabled = true;
    bool vehicle_commands_enabled = true;
    bool control_locked = false;
    std::string control_lock_reason;
    // Kept opt-in for source compatibility with offline command
    // characterization tests. Production control enables this before any
    // vehicle-affecting command is constructed.
    bool preflight_required = false;
    bool preflight_ready = true;
};

struct CommandRequest {
    using Clock = std::chrono::steady_clock;

    CommandType type = CommandType::VelocitySetpoint;
    std::string mode;
    bool arm = false;
    double altitude_m = 0.0;
    double vx = 0.0;
    double vy = 0.0;
    double vz = 0.0;
    double yaw_rate = 0.0;
    bool body_frame = true;
    Clock::time_point created_at{};
    Clock::time_point valid_until = Clock::time_point::max();

    static CommandRequest set_mode(
        std::string value,
        Clock::time_point created_at = Clock::now(),
        Clock::time_point valid_until = Clock::time_point::max()) {
        CommandRequest request;
        request.type = CommandType::SetMode;
        request.mode = std::move(value);
        request.created_at = created_at;
        request.valid_until = valid_until;
        return request;
    }

    static CommandRequest arm_disarm(
        bool value,
        Clock::time_point created_at = Clock::now(),
        Clock::time_point valid_until = Clock::time_point::max()) {
        CommandRequest request;
        request.type = CommandType::ArmDisarm;
        request.arm = value;
        request.created_at = created_at;
        request.valid_until = valid_until;
        return request;
    }

    static CommandRequest takeoff(
        double altitude,
        Clock::time_point created_at = Clock::now(),
        Clock::time_point valid_until = Clock::time_point::max()) {
        CommandRequest request;
        request.type = CommandType::Takeoff;
        request.altitude_m = altitude;
        request.created_at = created_at;
        request.valid_until = valid_until;
        return request;
    }

    static CommandRequest velocity_local(
        double x, double y, double z, double yaw,
        Clock::time_point created_at = Clock::now(),
        Clock::time_point valid_until = Clock::time_point::max()) {
        CommandRequest request = velocity(x, y, z, yaw, false, created_at, valid_until);
        return request;
    }

    static CommandRequest velocity_body(
        double x, double y, double z, double yaw,
        Clock::time_point created_at = Clock::now(),
        Clock::time_point valid_until = Clock::time_point::max()) {
        return velocity(x, y, z, yaw, true, created_at, valid_until);
    }

    static CommandRequest zero_velocity(
        Clock::time_point created_at = Clock::now(),
        Clock::time_point valid_until = Clock::time_point::max()) {
        return velocity_body(0.0, 0.0, 0.0, 0.0, created_at, valid_until);
    }

    static CommandRequest land(
        Clock::time_point created_at = Clock::now(),
        Clock::time_point valid_until = Clock::time_point::max()) {
        CommandRequest request;
        request.type = CommandType::Land;
        request.created_at = created_at;
        request.valid_until = valid_until;
        return request;
    }

private:
    static CommandRequest velocity(
        double x, double y, double z, double yaw, bool body,
        Clock::time_point created_at, Clock::time_point valid_until) {
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
};

struct CommandDecision {
    using Clock = CommandRequest::Clock;

    CommandType type = CommandType::VelocitySetpoint;
    bool allowed = false;
    bool sent = false;
    GateBlockReason block_reason = GateBlockReason::None;
    std::string control_lock_reason;
    Clock::time_point evaluated_at{};

    explicit operator bool() const { return allowed && sent; }
};

inline const char* command_type_name(CommandType type) {
    switch (type) {
        case CommandType::SetMode: return "SET_MODE";
        case CommandType::ArmDisarm: return "ARM_DISARM";
        case CommandType::Takeoff: return "TAKEOFF";
        case CommandType::VelocitySetpoint: return "VELOCITY_SETPOINT";
        case CommandType::Land: return "LAND";
    }
    return "UNKNOWN";
}

inline const char* gate_block_reason_name(GateBlockReason reason) {
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

class CommandGate {
public:
    CommandAuthority authority() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return authority_;
    }

    void set_authority(CommandAuthority authority) {
        std::lock_guard<std::mutex> lock(mutex_);
        authority_ = authority;
    }

    void set_control_locked(bool locked, std::string reason = {}) {
        std::lock_guard<std::mutex> lock(mutex_);
        authority_.control_locked = locked;
        if (locked && !reason.empty()) authority_.control_lock_reason = std::move(reason);
        if (!locked) authority_.control_lock_reason.clear();
    }

    void set_preflight_required(bool required) {
        std::lock_guard<std::mutex> lock(mutex_);
        authority_.preflight_required = required;
    }

    void set_preflight_ready(bool ready) {
        std::lock_guard<std::mutex> lock(mutex_);
        authority_.preflight_ready = ready;
    }

    template <typename SendFn>
    CommandDecision authorize_and_run(const CommandRequest& request, SendFn&& send) {
        // Keep the authority lock while the packet builder writes to the
        // transport. A lock change cannot happen between the final check and
        // the actual vehicle-affecting send.
        std::lock_guard<std::mutex> lock(mutex_);
        const auto now = CommandRequest::Clock::now();
        CommandDecision decision;
        decision.type = request.type;
        decision.evaluated_at = now;
        decision.block_reason = block_reason_locked(request, now);
        if (decision.block_reason == GateBlockReason::ControlLocked) {
            decision.control_lock_reason = authority_.control_lock_reason;
        }
        if (decision.block_reason != GateBlockReason::None) return decision;

        decision.allowed = true;
        decision.sent = static_cast<bool>(send());
        return decision;
    }

private:
    GateBlockReason block_reason_locked(
        const CommandRequest& request, CommandRequest::Clock::time_point now) const {
        if (!authority_.automation_enabled) return GateBlockReason::AuthorityDisabled;
        if (!authority_.vehicle_commands_enabled) {
            return GateBlockReason::CommandModeDisabled;
        }
        if (authority_.control_locked) return GateBlockReason::ControlLocked;
        if (authority_.preflight_required && !authority_.preflight_ready) {
            return GateBlockReason::PreflightNotReady;
        }
        if (request.valid_until != CommandRequest::Clock::time_point::max() &&
            now >= request.valid_until) {
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
        return GateBlockReason::None;
    }

    mutable std::mutex mutex_;
    CommandAuthority authority_{};
};

}  // namespace autopilot
