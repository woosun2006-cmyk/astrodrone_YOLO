#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "../third_party/mavlink/common/mavlink.h"
#include "../drone_lib.hpp"
#include "../safety/safety_monitor.hpp"
#include "autopilot_state.hpp"
#include "../app/runtime_config.hpp"

namespace autopilot {

// The sole MAVLink connection owner. SafetyMonitor authorization is injected
// by FlightMissionApp; this class only performs technical validation and
// packet serialization/transmission.
class AutopilotMavlinkAdapter {
public:
    using Clock = std::chrono::steady_clock;
    using ApprovalSink =
        std::function<safety::CommandDecision(const safety::CommandRequest&)>;
    using DecisionSink =
        std::function<void(const safety::CommandDecision&)>;
    using ModeRequestSink =
        std::function<void(const safety::CommandRequest&, bool)>;

    AutopilotMavlinkAdapter(std::unique_ptr<MavConnection> connection,
                            app::RuntimeConfig config,
                            ApprovalSink approval_sink = {},
                            DecisionSink decision_sink = {},
                            ModeRequestSink mode_request_sink = {});
    AutopilotMavlinkAdapter(std::unique_ptr<Transport> transport,
                            app::RuntimeConfig config,
                            ApprovalSink approval_sink = {},
                            DecisionSink decision_sink = {},
                            ModeRequestSink mode_request_sink = {});

    ~AutopilotMavlinkAdapter();

    AutopilotMavlinkAdapter(const AutopilotMavlinkAdapter&) = delete;
    AutopilotMavlinkAdapter& operator=(const AutopilotMavlinkAdapter&) = delete;

    bool wait_heartbeat(double timeout_sec, mavlink_heartbeat_t* out = nullptr);
    bool recv_match(const std::vector<uint32_t>& msg_ids,
                    mavlink_message_t& out, double timeout_sec);
    bool poll(double timeout_sec = 0.0);

    // Telemetry configuration is a technical packet API. Runtime policy is
    // checked by FlightMissionApp before this method is called.
    bool request_message_interval(uint32_t message_id, double frequency_hz);

    uint8_t target_system() const { return connection_->target_system(); }
    uint8_t target_component() const { return connection_->target_component(); }
    size_t tx_packet_count() const { return connection_->tx_packet_count(); }

    safety::CommandDecision set_mode(const std::string& mode);
    safety::CommandDecision arm_disarm(bool arm);
    safety::CommandDecision takeoff(double altitude_m);
    safety::CommandDecision send_velocity(double vx, double vy, double vz,
                                          double yaw_rate = 0.0);
    safety::CommandDecision send_velocity_body(double vx, double vy, double vz,
                                               double yaw_rate = 0.0);
    safety::CommandDecision send_zero_velocity();
    safety::CommandDecision land();
    safety::CommandDecision send_mission_clear();
    safety::CommandDecision send_mission_count(uint16_t count);
    safety::CommandDecision send_mission_item(uint16_t seq, uint16_t command,
                                              uint8_t frame, int32_t x, int32_t y,
                                              float altitude,
                                              uint8_t current = 0);
    safety::CommandDecision send_mission_request_list();
    safety::CommandDecision send_mission_request_item(uint16_t seq);
    safety::CommandDecision send_mission_set_current(uint16_t seq);

    const app::RuntimeConfig& config() const { return config_; }
    const AutopilotState& state() const { return state_; }
    AutopilotState& mutable_state() { return state_; }

private:
    class TelemetryTrace;
    class TelemetryFanout;

    safety::CommandDecision send_request(const safety::CommandRequest& request);
    bool serialize_and_send(const safety::CommandRequest& request);
    bool send_mission_protocol(const safety::CommandRequest& request);
    bool technical_request_is_valid(const safety::CommandRequest& request) const;

    void observe_message(const mavlink_message_t& message,
                         Clock::time_point now = Clock::now());
    void update_state(const mavlink_message_t& message, Clock::time_point now);

    std::unique_ptr<MavConnection> connection_;
    app::RuntimeConfig config_;
    std::shared_ptr<TelemetryTrace> telemetry_trace_;
    std::shared_ptr<TelemetryTrace> input_trace_;
    std::unique_ptr<TelemetryFanout> telemetry_fanout_;
    std::map<uint32_t, uint64_t> input_dequeue_counters_;
    AutopilotState state_;
    ApprovalSink approval_sink_;
    DecisionSink decision_sink_;
    ModeRequestSink mode_request_sink_;
};

}  // namespace autopilot
