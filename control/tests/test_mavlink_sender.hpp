#pragma once

// Test-only raw packet sender for characterization tests. Production MAVLink
// serialization is implemented by AutopilotMavlinkAdapter.
#include "../drone_lib.hpp"
#include "../safety/safety_monitor.hpp"

namespace test {

class TestPacketSender {
public:
    using DecisionSink = std::function<void(const safety::CommandDecision&)>;
    using ModeRequestSink =
        std::function<void(const safety::CommandRequest&, bool)>;

    explicit TestPacketSender(MavConnection& connection,
                              safety::SafetyMonitor& monitor,
                              DecisionSink decision_sink = {},
                              ModeRequestSink mode_request_sink = {})
        : connection_(connection), monitor_(monitor),
          decision_sink_(std::move(decision_sink)),
          mode_request_sink_(std::move(mode_request_sink)) {}

    safety::CommandDecision send(const safety::CommandRequest& request) {
        bool mode_started = false;
        auto decision = monitor_.authorize_and_run(request, [&]() {
            if (request.type == safety::CommandType::SetMode && mode_request_sink_) {
                mode_request_sink_(request, false);
                mode_started = true;
            }
            switch (request.type) {
                case safety::CommandType::SetMode:
                    return drone::set_mode(connection_, request.mode);
                case safety::CommandType::ArmDisarm:
                    drone::arm_disarm(connection_, request.arm);
                    return true;
                case safety::CommandType::Takeoff:
                    drone::takeoff(connection_, request.altitude_m);
                    return true;
                case safety::CommandType::VelocitySetpoint:
                    if (request.body_frame) {
                        drone::send_velocity_body(connection_, request.vx, request.vy,
                                                  request.vz, request.yaw_rate);
                    } else {
                        drone::send_velocity(connection_, request.vx, request.vy,
                                             request.vz, request.yaw_rate);
                    }
                    return true;
                case safety::CommandType::Land:
                    drone::land(connection_);
                    return true;
                case safety::CommandType::MissionProtocol:
                    return send_mission_protocol(request);
            }
            return false;
        });
        if (decision_sink_) decision_sink_(decision);
        if (mode_started && mode_request_sink_) {
            mode_request_sink_(request, decision.sent);
        }
        return decision;
    }

    safety::CommandDecision set_mode(const std::string& mode) {
        const auto now = safety::CommandRequest::Clock::now();
        return send(safety::CommandRequest::set_mode(
            mode, now, now + std::chrono::seconds(5)));
    }
    safety::CommandDecision arm_disarm(bool arm) {
        return send(safety::CommandRequest::arm_disarm(arm));
    }
    safety::CommandDecision takeoff(double altitude) {
        return send(safety::CommandRequest::takeoff(altitude));
    }
    safety::CommandDecision send_velocity(double vx, double vy, double vz,
                                           double yaw_rate = 0) {
        return send(safety::CommandRequest::velocity_local(vx, vy, vz, yaw_rate));
    }
    safety::CommandDecision send_velocity_body(double vx, double vy, double vz,
                                                double yaw_rate = 0) {
        return send(safety::CommandRequest::velocity_body(vx, vy, vz, yaw_rate));
    }
    safety::CommandDecision send_zero_velocity() {
        return send(safety::CommandRequest::zero_velocity());
    }
    safety::CommandDecision land() {
        return send(safety::CommandRequest::land());
    }
    safety::CommandDecision send_mission_clear() {
        return send(safety::CommandRequest::mission_clear());
    }
    safety::CommandDecision send_mission_count(uint16_t count) {
        return send(safety::CommandRequest::mission_count(count));
    }
    safety::CommandDecision send_mission_item(uint16_t seq, uint16_t command,
                                              uint8_t frame, int32_t x, int32_t y,
                                              float altitude, uint8_t current = 0) {
        return send(safety::CommandRequest::mission_item(
            seq, command, frame, x, y, altitude, current));
    }
    safety::CommandDecision send_mission_request_list() {
        return send(safety::CommandRequest::mission_request_list());
    }
    safety::CommandDecision send_mission_request_item(uint16_t seq) {
        return send(safety::CommandRequest::mission_request_item(seq));
    }
    safety::CommandDecision send_mission_set_current(uint16_t seq) {
        return send(safety::CommandRequest::mission_set_current(seq));
    }

private:
    bool send_mission_protocol(const safety::CommandRequest& request) {
        constexpr uint8_t source_system = 255;
        constexpr uint8_t source_component = 0;
        mavlink_message_t message{};
        const uint8_t system = connection_.target_system();
        const uint8_t component = connection_.target_component();
        switch (request.mission_operation) {
            case safety::MissionOperation::Clear:
                mavlink_msg_mission_clear_all_pack(
                    source_system, source_component, &message, system, component,
                    MAV_MISSION_TYPE_MISSION);
                break;
            case safety::MissionOperation::Count:
                mavlink_msg_mission_count_pack(
                    source_system, source_component, &message, system, component,
                    request.mission_count_value, MAV_MISSION_TYPE_MISSION, 0);
                break;
            case safety::MissionOperation::Item:
                mavlink_msg_mission_item_int_pack(
                    source_system, source_component, &message, system, component,
                    request.mission_seq, request.mission_frame, request.mission_command,
                    request.mission_current, 1, 0, 0, 0, 0, request.mission_x,
                    request.mission_y, request.mission_altitude_m,
                    MAV_MISSION_TYPE_MISSION);
                break;
            case safety::MissionOperation::RequestList:
                mavlink_msg_mission_request_list_pack(
                    source_system, source_component, &message, system, component,
                    MAV_MISSION_TYPE_MISSION);
                break;
            case safety::MissionOperation::RequestItem:
                mavlink_msg_mission_request_int_pack(
                    source_system, source_component, &message, system, component,
                    request.mission_seq, MAV_MISSION_TYPE_MISSION);
                break;
            case safety::MissionOperation::SetCurrent:
                mavlink_msg_mission_set_current_pack(
                    source_system, source_component, &message, system, component,
                    request.mission_seq);
                break;
        }
        connection_.send(message);
        return true;
    }

    MavConnection& connection_;
    safety::SafetyMonitor& monitor_;
    DecisionSink decision_sink_;
    ModeRequestSink mode_request_sink_;
};

}  // namespace test
