#pragma once

#include <chrono>
#include <cstdint>
#include <functional>
#include <iosfwd>
#include <string>
#include <utility>
#include <vector>

#include "../target_link.hpp"

namespace app {

struct FlightMissionAppOptions {
    bool self_launch = false;
    bool auto_intercept = false;
};

// The operator approval gate accepts exactly one complete line: "FLY".
// Keeping the parser separate makes the no-command-before-approval rule
// directly testable without constructing a MAVLink transport.
bool wait_for_fly_approval(std::istream& input, std::ostream& output);

// App-owned lifecycle phases. Vehicle mode, armed state, altitude and health
// remain telemetry owned by AutopilotMavlinkAdapter::AutopilotState.
enum class FlightPhase {
    PRECHECK,
    TAKEOFF,
    TARGET_SEARCH,
    GUIDED,
    REACQUIRE,
    SAFE_HOVER,
    LANDING,
    FINISHED,
};

const char* flight_phase_name(FlightPhase phase);

struct FlightPhaseEvent {
    uint64_t sequence = 0;
    FlightPhase previous = FlightPhase::PRECHECK;
    FlightPhase current = FlightPhase::PRECHECK;
    std::string reason;
    std::chrono::steady_clock::time_point timestamp{};
};

// The target link is asynchronous: the handoff wait and the approach loop can
// be backed by different receiver instances. Keep the first fresh observation
// as an explicit gate so an empty first poll is not treated as target loss.
class TargetObservationGate {
public:
    bool update(const TargetRangeMsg& observation, bool transport_fresh) {
        if (transport_fresh && observation.valid && observation.found) {
            target_seen_ = true;
            return true;
        }
        return false;
    }

    bool target_seen() const { return target_seen_; }

private:
    bool target_seen_ = false;
};

// Handoff-only dwell gate. This is intentionally separate from each
// algorithm's center_hold_sec: it decides when AUTO may hand the vehicle to
// GUIDED, while the algorithms decide centering/descent after handoff.
class TargetHandoffGate {
public:
    using Clock = std::chrono::steady_clock;

    bool update(const TargetRangeMsg& observation, bool transport_fresh,
                Clock::time_point now, double minimum_altitude_m,
                double dwell_sec, double minimum_confidence = 0.60);
    void reset();
    bool active() const { return active_; }
    const std::string& class_name() const { return class_name_; }

private:
    bool active_ = false;
    Clock::time_point started_at_{};
    std::string class_name_;
};

// Target-loss recovery is an application handoff gate, separate from the
// algorithm's center dwell. It validates the observation and health boundary
// while leaving the algorithm object/state untouched.
class TargetReacquireGate {
public:
    using Clock = std::chrono::steady_clock;

    bool update(const TargetRangeMsg& observation, bool transport_fresh,
                bool health_ok, bool control_locked, Clock::time_point now,
                const std::string& expected_class, double minimum_confidence,
                double dwell_sec);
    void reset();
    bool active() const { return active_; }
    double elapsed_sec(Clock::time_point now) const;

private:
    bool active_ = false;
    Clock::time_point started_at_{};
};

// The legacy LOITER policy name is retained for configuration compatibility,
// but a target-loss latch now holds the already-tracked GUIDED mode. Only an
// explicit alternative policy such as LAND may request another mode.
bool target_loss_keeps_guided_hover(const std::string& resume_mode);

// The common phase transition owner belongs to the application layer. It does
// not parse telemetry or send MAVLink; it only orders application events and
// latches SAFE_HOVER until an explicit landing/finish path is observed.
class FlightPhaseController {
public:
    using Clock = std::chrono::steady_clock;
    using EventSink = std::function<void(const FlightPhaseEvent&)>;

    explicit FlightPhaseController(EventSink sink = {}) : sink_(std::move(sink)) {}

    void set_event_sink(EventSink sink) { sink_ = std::move(sink); }

    bool transition(FlightPhase next, std::string reason,
                    Clock::time_point timestamp = Clock::now(),
                    uint64_t input_sequence = 0);

    FlightPhase current() const { return current_; }
    bool safe_hover_latched() const { return safe_hover_latched_; }
    uint64_t next_sequence() const { return next_sequence_; }
    uint64_t last_input_sequence() const { return last_input_sequence_; }
    const std::vector<FlightPhaseEvent>& events() const { return events_; }

private:
    bool accept_input(Clock::time_point timestamp, uint64_t input_sequence);

    FlightPhase current_ = FlightPhase::PRECHECK;
    uint64_t next_sequence_ = 1;
    uint64_t last_input_sequence_ = 0;
    Clock::time_point last_input_timestamp_{};
    bool safe_hover_latched_ = false;
    std::vector<FlightPhaseEvent> events_;
    EventSink sink_;
};

// 실제 비행에 필요한 모듈을 생성하고 연결하는 composition root.
// 무거운 transport/socket 초기화는 run()에서 수행한다.
class FlightMissionApp {
public:
    explicit FlightMissionApp(FlightMissionAppOptions options);

    int run();

private:
    FlightMissionAppOptions options_;
    FlightPhaseController phase_controller_;
};

}  // namespace app
