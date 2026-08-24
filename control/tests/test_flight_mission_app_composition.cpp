#include "../app/flight_mission_app.hpp"

#include <chrono>
#include <cstdio>
#include <sstream>

int main() {
    const auto check = [](bool condition, int line) {
        if (condition) return true;
        std::fprintf(stderr, "composition check failed at line %d\n", line);
        return false;
    };
#define CHECK(condition) do { if (!check((condition), __LINE__)) return 1; } while (false)

    CHECK(app::target_loss_keeps_guided_hover("LOITER"));
    CHECK(app::target_loss_keeps_guided_hover("GUIDED"));
    CHECK(!app::target_loss_keeps_guided_hover("AUTO"));

    std::istringstream approval_input("fly\nFLY\n");
    std::ostringstream approval_output;
    CHECK(app::wait_for_fly_approval(approval_input, approval_output));
    CHECK(approval_output.str().find("FLY 입력이 필요합니다.") != std::string::npos);

    std::istringstream closed_input("");
    std::ostringstream closed_output;
    CHECK(!app::wait_for_fly_approval(closed_input, closed_output));

    app::TargetObservationGate gate;
    TargetRangeMsg fresh_target{};
    fresh_target.valid = 1;
    fresh_target.found = 1;
    fresh_target.altitude_valid = 1;
    fresh_target.altitude_m = 4.8F;
    fresh_target.observation_age_ms = 20.0F;
    fresh_target.target_confidence = 0.60F;
    std::snprintf(fresh_target.class_name, sizeof(fresh_target.class_name), "astro-drone");
    CHECK(gate.update(fresh_target, true) && gate.target_seen());

    TargetRangeMsg stale = fresh_target;
    stale.valid = 0;
    CHECK(!gate.update(stale, false) && gate.target_seen());

    app::TargetHandoffGate handoff;
    const auto t0 = app::TargetHandoffGate::Clock::now();
    CHECK(!handoff.update(fresh_target, true, t0, 4.7, 2.0));
    CHECK(!handoff.update(fresh_target, true, t0 + std::chrono::seconds(1), 4.7, 2.0));
    CHECK(handoff.update(fresh_target, true, t0 + std::chrono::seconds(2), 4.7, 2.0));
    TargetRangeMsg changed_class = fresh_target;
    std::snprintf(changed_class.class_name, sizeof(changed_class.class_name), "other");
    CHECK(!handoff.update(changed_class, true, t0 + std::chrono::seconds(2), 4.7, 2.0));
    CHECK(!handoff.update(fresh_target, true, t0 + std::chrono::seconds(3), 4.7, 2.0));

    TargetRangeMsg low_confidence = fresh_target;
    low_confidence.target_confidence = 0.59F;
    handoff.reset();
    CHECK(!handoff.update(low_confidence, true, t0, 4.7, 2.0));
    CHECK(!handoff.update(low_confidence, true, t0 + std::chrono::seconds(3), 4.7, 2.0));
    handoff.reset();
    CHECK(!handoff.update(fresh_target, true, t0, 4.7, 2.0));
    CHECK(!handoff.update(fresh_target, true, t0 + std::chrono::milliseconds(1900), 4.7, 2.0));
    CHECK(handoff.update(fresh_target, true, t0 + std::chrono::seconds(2), 4.7, 2.0));

    app::TargetReacquireGate reacquire;
    CHECK(!reacquire.update(fresh_target, true, true, false, t0,
                            "astro-drone", 0.60, 2.0));
    CHECK(!reacquire.update(fresh_target, true, true, false,
                            t0 + std::chrono::milliseconds(1900),
                            "astro-drone", 0.60, 2.0));
    CHECK(reacquire.update(fresh_target, true, true, false,
                           t0 + std::chrono::seconds(2),
                           "astro-drone", 0.60, 2.0));
    CHECK(!reacquire.update(low_confidence, true, true, false,
                            t0 + std::chrono::seconds(3),
                            "astro-drone", 0.60, 2.0));
    CHECK(!reacquire.update(fresh_target, true, true, false,
                            t0 + std::chrono::milliseconds(4900),
                            "astro-drone", 0.60, 2.0));
    TargetRangeMsg reacquire_changed_class = fresh_target;
    std::snprintf(reacquire_changed_class.class_name,
                  sizeof(reacquire_changed_class.class_name), "other");
    CHECK(!reacquire.update(reacquire_changed_class, true, true, false,
                            t0 + std::chrono::milliseconds(5000),
                            "astro-drone", 0.60, 2.0));
    CHECK(!reacquire.update(fresh_target, true, true, false,
                            t0 + std::chrono::milliseconds(6900),
                            "astro-drone", 0.60, 2.0));
    CHECK(reacquire.update(fresh_target, true, true, false,
                           t0 + std::chrono::milliseconds(8901),
                           "astro-drone", 0.60, 2.0));
    CHECK(!reacquire.update(fresh_target, true, false, false,
                            t0 + std::chrono::milliseconds(9000),
                            "astro-drone", 0.60, 2.0));
    CHECK(!reacquire.update(fresh_target, true, true, true,
                            t0 + std::chrono::milliseconds(9000),
                            "astro-drone", 0.60, 2.0));

    app::FlightMissionAppOptions options;
    app::FlightMissionApp application(options);
    (void)application;
    CHECK(!options.self_launch && !options.auto_intercept);
    return 0;
}
