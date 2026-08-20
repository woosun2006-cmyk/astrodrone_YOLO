#pragma once

#include <chrono>
#include <functional>
#include <string>
#include <utility>

#include "command_gate.hpp"
#include "../drone_lib.hpp"

namespace autopilot {

// 중앙 vehicle-affecting MAVLink 송신 경계다. 패킷 생성은 기존 drone_lib
// 함수를 그대로 호출해 명령 필드와 serialization을 보존한다.
class CommandSender {
public:
    using DecisionSink = std::function<void(const CommandDecision&)>;
    using ModeRequestSink = std::function<void(const CommandRequest&, bool)>;

    explicit CommandSender(MavConnection& connection, CommandGate& gate,
                            DecisionSink decision_sink = {},
                            ModeRequestSink mode_request_sink = {})
        : connection_(connection), gate_(gate), decision_sink_(std::move(decision_sink)),
          mode_request_sink_(std::move(mode_request_sink)) {}

    CommandDecision send(const CommandRequest& request) {
        bool mode_request_started = false;
        CommandDecision decision = gate_.authorize_and_run(request, [&]() -> bool {
            if (request.type == CommandType::SetMode && mode_request_sink_) {
                // Register the attempt before the transport write. The
                // second callback records whether the write completed;
                // neither callback acknowledges the actual mode transition.
                mode_request_sink_(request, false);
                mode_request_started = true;
            }
            switch (request.type) {
                case CommandType::SetMode:
                    return drone::set_mode(connection_, request.mode);
                case CommandType::ArmDisarm:
                    drone::arm_disarm(connection_, request.arm);
                    return true;
                case CommandType::Takeoff:
                    drone::takeoff(connection_, request.altitude_m);
                    return true;
                case CommandType::VelocitySetpoint:
                    if (request.body_frame) {
                        drone::send_velocity_body(connection_, request.vx, request.vy,
                                                  request.vz, request.yaw_rate);
                    } else {
                        drone::send_velocity(connection_, request.vx, request.vy,
                                             request.vz, request.yaw_rate);
                    }
                    return true;
                case CommandType::Land:
                    drone::land(connection_);
                    return true;
            }
            return false;
        });
        if (decision_sink_) decision_sink_(decision);
        if (mode_request_started && mode_request_sink_) {
            mode_request_sink_(request, decision.sent);
        }
        return decision;
    }

    CommandDecision set_mode(const std::string& mode) {
        const auto now = CommandRequest::Clock::now();
        return send(CommandRequest::set_mode(mode, now, now + std::chrono::seconds(5)));
    }

    CommandDecision arm_disarm(bool arm) {
        return send(CommandRequest::arm_disarm(arm));
    }

    CommandDecision takeoff(double altitude) {
        return send(CommandRequest::takeoff(altitude));
    }

    CommandDecision send_velocity(double vx, double vy, double vz, double yaw_rate = 0) {
        return send(CommandRequest::velocity_local(vx, vy, vz, yaw_rate));
    }

    CommandDecision send_velocity_body(double vx, double vy, double vz, double yaw_rate = 0) {
        return send(CommandRequest::velocity_body(vx, vy, vz, yaw_rate));
    }

    CommandDecision send_zero_velocity() {
        return send(CommandRequest::zero_velocity());
    }

    CommandDecision land() { return send(CommandRequest::land()); }

    bool control_locked() const { return gate_.authority().control_locked; }

private:
    MavConnection& connection_;
    CommandGate& gate_;
    DecisionSink decision_sink_;
    ModeRequestSink mode_request_sink_;
};

}  // namespace autopilot
