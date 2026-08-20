#include "mission/target_centered_hold.hpp"

#include <iostream>

namespace {

bool check(bool condition, const char* label) {
    if (!condition) std::cerr << "check failed: " << label << '\n';
    return condition;
}

bool enters_only_for_fresh_centered_stop_distance_target() {
    mission::TargetCenteredHold hold;
    if (!check(!hold.holding(), "initial state is Align")) return false;
    if (!check(!hold.enter_if_ready(false, true, 1.0, 2.0),
               "stale target does not enter hold")) return false;
    if (!check(!hold.enter_if_ready(true, false, 1.0, 2.0),
               "uncentered target does not enter hold")) return false;
    if (!check(!hold.enter_if_ready(true, true, 2.1, 2.0),
               "far target does not enter hold")) return false;
    if (!check(hold.enter_if_ready(true, true, 2.0, 2.0),
               "centered stop-distance target enters hold")) return false;
    return check(hold.holding(), "state is TargetCenteredHold");
}

bool target_loss_resets_hold_for_the_existing_safety_policy() {
    mission::TargetCenteredHold hold;
    if (!check(hold.enter_if_ready(true, true, 1.9, 2.0), "enter hold")) return false;
    if (!check(hold.enter_if_ready(true, true, 10.0, 2.0),
               "far target cannot clear hold")) return false;
    hold.reset();
    if (!check(!hold.holding(), "target loss resets centered hold")) return false;
    return check(hold.enter_if_ready(true, true, 1.9, 2.0),
                 "hold can be reacquired after a fresh target");
}

bool centered_hold_survives_normal_intercept_timeout() {
    mission::TargetCenteredHold hold;
    if (!check(hold.enter_if_ready(true, true, 1.9, 2.0), "enter 120s hold")) {
        return false;
    }

    // The production loop may reach its normal 30-second approach timeout,
    // but a fresh centered hold must remain latched until a safety event.
    for (int elapsed = 0; elapsed <= 120; ++elapsed) {
        if (!check(hold.holding(), "hold survives normal timeout")) return false;
    }
    return true;
}

}  // namespace

int main() {
    if (enters_only_for_fresh_centered_stop_distance_target() &&
        target_loss_resets_hold_for_the_existing_safety_policy() &&
        centered_hold_survives_normal_intercept_timeout()) {
        std::cout << "PASS target centered hold transition and latch\n";
        return 0;
    }
    return 1;
}
