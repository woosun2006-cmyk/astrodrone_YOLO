#include <cmath>
#include <iostream>

#include "../mission/centered_vertical_descent_algorithm.hpp"

namespace {

bool check(bool condition, const char* label) {
    if (!condition) std::cerr << "check failed: " << label << '\n';
    return condition;
}

}  // namespace

int main() {
    mission::CenteredVerticalDescentConfig config;
    config.calibration = PixelCalibration{530.0, 530.0, 320.0, 240.0};
    config.gain = 1.0;
    config.max_forward_speed = 0.5;
    config.max_lateral_speed = 0.5;
    config.max_vector_speed = 0.5;
    config.max_descent_speed_mps = 0.2;
    config.desired_path_angle_rad = std::atan2(0.2, 0.5);
    config.horizontal_speed_deadband_mps = 0.01;
    config.filter_alpha = 1.0;
    config.max_acceleration_mps2 = 0.5;
    config.max_jerk_mps3 = 100.0;
    config.velocity_deadband_rad = 0.01;
    config.velocity_hysteresis_rad = 0.005;
    config.theta_safe_threshold_rad = 0.1;

    config.search_altitude_m = 5.0;
    config.search_altitude_tolerance_m = 0.3;
    config.center_hold_sec = 0.10;
    config.hold_altitude_m = 1.0;
    config.center_tolerance_px = 25.0;
    config.center_dwell_tolerance_px = 15.0;
    config.descent_realign_enter_tolerance_px = 25.0;
    config.descent_realign_exit_tolerance_px = 45.0;
    config.descent_max_forward_speed_mps = 0.15;
    config.descent_max_lateral_speed_mps = 0.15;
    config.descent_max_vector_speed_mps = 0.15;
    mission::CenteredVerticalDescentAlgorithm guidance(config);
    const auto top = guidance.update(0.0, 100.0, 5.0, true, 0.05, 20.0);
    if (!check(top.algorithm_state == "CENTERING" && top.theta_forward_rad > 0.0 &&
                   top.vx > 0.0 && top.vz == 0.0 && top.raw_vz == 0.0,
               "centering keeps search altitude")) return 1;
    if (!check(std::abs(top.vy) < 1e-9, "top -> no vy")) return 1;
    if (!check(top.vx <= config.max_acceleration_mps2 * 0.05 + 1e-9,
               "acceleration cap")) return 1;

    const auto right = guidance.update(100.0, 0.0, 5.0, true, 0.05, 20.0);
    if (!check(right.theta_right_rad > 0.0 && right.vy > 0.0, "right -> +vy")) return 1;
    const auto left = guidance.update(-100.0, 0.0, 5.0, true, 0.05, 20.0);
    if (!check(left.theta_right_rad < 0.0 && left.vy < 0.0, "left -> -vy")) return 1;
    const auto bottom = guidance.update(0.0, -100.0, 5.0, true, 0.05, 20.0);
    if (!check(bottom.theta_forward_rad < 0.0 && bottom.vx < 0.0, "bottom -> -vx")) return 1;

    const auto diagonal = guidance.update(100.0, 100.0, 5.0, true, 0.05, 20.0);
    if (!check(std::hypot(diagonal.vx, diagonal.vy) <= config.max_vector_speed + 1e-9,
               "diagonal vector cap")) return 1;

    guidance.force_zero("reset");
    auto fast = guidance.update(100.0, 0.0, 5.0, true, 0.05, 20.0);
    for (int i = 0; i < 5; ++i) {
        fast = guidance.update(100.0, 0.0, 5.0, true, 0.05, 20.0);
    }
    const auto braking = guidance.update(-30.0, 0.0, 5.0, true, 0.05, 20.0);
    if (!check(std::abs(braking.vy) < std::abs(fast.vy), "smooth braking")) return 1;
    if (!check(std::abs(braking.vy) > 0.0, "braking is not an instant jump")) return 1;

    guidance.force_zero("deadband-reset");
    const auto small = guidance.update(0.0, 4.0, 5.0, true, 0.05, 20.0);
    if (!check(std::abs(small.vx) < 1e-9, "deadband")) return 1;
    const auto large = guidance.update(0.0, 30.0, 5.0, true, 0.05, 20.0);
    if (!check(large.vx > 0.0, "deadband activation")) return 1;
    const auto hysteresis = guidance.update(0.0, 26.0, 5.0, true, 0.05, 20.0);
    if (!check(hysteresis.vx > 0.0, "hysteresis hold")) return 1;
    const auto released = guidance.update(0.0, 4.0, 5.0, true, 0.05, 20.0);
    if (!check(std::abs(released.raw_vx) < 1e-9 && released.vx < hysteresis.vx,
               "hysteresis release")) return 1;

    guidance.force_zero("safety");
    const auto unsafe = guidance.update(0.0, 100.0, 5.0, true, 0.05, 20.0);
    if (!check(!unsafe.descent_allowed, "unsafe angle blocks descent")) return 1;
    guidance.force_zero("safety");
    const auto safe = guidance.update(0.0, 10.0, 5.0, true, 0.05, 20.0);
    if (!check(!safe.descent_allowed && safe.vz == 0.0,
               "center alignment is required before descent")) return 1;

    guidance.force_zero("strict-descent-reset");
    const auto pre_centered =
        guidance.update(30.0, 10.0, 5.0, true, 0.05, 20.0);
    if (!check(pre_centered.algorithm_state == "CENTERING" &&
                   pre_centered.vx > 0.0 && pre_centered.vy > 0.0 &&
                   pre_centered.vz == 0.0,
               "centering uses horizontal correction only")) return 1;
    const auto strict_dwell =
        guidance.update(0.0, 0.0, 5.0, true, 0.05, 20.0);
    if (!check(strict_dwell.algorithm_state == "CENTER_DWELL" &&
                   strict_dwell.vx == 0.0 && strict_dwell.vy == 0.0 &&
                   strict_dwell.vz == 0.0,
               "dwell is stationary")) return 1;
    const auto vertical_only =
        guidance.update(0.0, 0.0, 5.0, true, 0.10, 20.0, true, true);
    if (!check(vertical_only.algorithm_state == "VERTICAL_DESCENT" &&
                   vertical_only.vx == 0.0 && vertical_only.vy == 0.0 &&
                   vertical_only.vz > 0.0 &&
                   vertical_only.vz <= config.max_descent_speed_mps,
               "descent starts only after stationary dwell")) return 1;
    guidance.force_zero("horizontal-deadband");
    const auto centered = guidance.update(0.0, 0.0, 5.0, true, 0.05, 20.0);
    if (!check(centered.horizontal_speed_mps == 0.0 && centered.vz == 0.0,
               "horizontal deadband blocks descent")) return 1;

    guidance.force_zero("centered-descent-reset");
    guidance.update(0.0, 0.0, 5.0, true, 0.05, 20.0, true);
    guidance.update(0.0, 0.0, 5.0, true, 0.05, 20.0, true);
    const auto centered_descent =
        guidance.update(0.0, 0.0, 5.0, true, 0.05, 20.0, true);
    if (!check(centered_descent.vz > 0.0 &&
                   centered_descent.vz <= config.max_descent_speed_mps,
               "centered target may descend with the explicit mission gate")) return 1;

    guidance.force_zero("center-lock-gate-reset");
    const auto align_only =
        guidance.update(0.0, 30.0, 5.0, true, 0.05, 20.0, false, false);
    if (!check(align_only.vx > 0.0 && align_only.vz == 0.0 &&
                   !align_only.descent_allowed,
               "centered descent holds altitude before center lock")) return 1;

    guidance.force_zero("unsafe-reset");
    const auto unsafe_descent = guidance.update(0.0, 100.0, 5.0, true, 0.05, 20.0);
    if (!check(unsafe_descent.vz == 0.0 && unsafe_descent.raw_vz == 0.0 &&
                   unsafe_descent.descent_margin == 0.0,
               "unsafe angle produces zero descent")) return 1;

    const auto stale = guidance.force_zero("target_loss", 700.0);
    if (!check(stale.vx == 0.0 && stale.vy == 0.0 && stale.vz == 0.0 &&
                   stale.safety_override == "target_loss",
               "stale immediate zero")) return 1;

    guidance.force_zero("strict-center-dwell-reset");
    const auto not_tight_enough = guidance.update(20.0, 0.0, 5.0, true, 0.05, 20.0,
                                                   true, true);
    if (!check(not_tight_enough.algorithm_state == "CENTERING" &&
                   not_tight_enough.vy > 0.0 && not_tight_enough.vz == 0.0,
               "center dwell requires tight tolerance")) return 1;
    const auto centered_dwell = guidance.update(10.0, 0.0, 5.0, true, 0.05, 20.0,
                                                 true, true);
    if (!check(centered_dwell.algorithm_state == "CENTER_DWELL" &&
                   centered_dwell.vx == 0.0 && centered_dwell.vy == 0.0 &&
                   centered_dwell.vz == 0.0,
               "center dwell holds position before descent")) return 1;
    const auto dwell_drift = guidance.update(30.0, 0.0, 5.0, true, 0.05, 20.0,
                                              true, true);
    if (!check(dwell_drift.algorithm_state == "CENTERING" &&
                   dwell_drift.vy > 0.0 && dwell_drift.vz == 0.0,
               "dwell resets when target leaves center tolerance")) return 1;

    guidance.force_zero("altitude-reset");
    const auto descent_start = guidance.update(0.0, 0.0, 5.0, true, 0.05, 20.0,
                                               true, true);
    if (!check(descent_start.algorithm_state == "CENTER_DWELL",
               "centered target starts dwell")) return 1;
    const auto descent = guidance.update(0.0, 0.0, 5.0, true, 0.10, 20.0, true, true);
    if (!check(descent.algorithm_state == "VERTICAL_DESCENT" && descent.vz > 0.0,
               "center dwell enables descent")) return 1;
    const auto descent_recenter =
        guidance.update(20.0, 0.0, 4.8, true, 0.05, 20.0, true, true);
    if (!check(descent_recenter.algorithm_state == "VERTICAL_DESCENT" &&
                   descent_recenter.vy > 0.0 && descent_recenter.vy <= 0.15 &&
                   descent_recenter.vz > 0.0,
               "descent retains horizontal correction within tracking band")) return 1;
    const auto paused_realign = guidance.update(50.0, 0.0, 4.7, true, 0.05, 20.0,
                                                 true, true);
    if (!check(paused_realign.algorithm_state == "VERTICAL_DESCENT" &&
                   paused_realign.vy > 0.0 && paused_realign.vy <= 0.15 &&
                   paused_realign.vz == 0.0,
               "large descent error pauses vertical motion")) return 1;
    const auto resumed_realign = guidance.update(20.0, 0.0, 4.7, true, 0.05, 20.0,
                                                  true, true);
    if (!check(resumed_realign.algorithm_state == "VERTICAL_DESCENT" &&
                   resumed_realign.vy > 0.0 && resumed_realign.vy <= 0.15 &&
                   resumed_realign.vz > 0.0,
               "descent resumes without center dwell")) return 1;
    const auto low_altitude = guidance.update(0.0, 0.0, 1.0, true, 0.05, 20.0, true, true);
    if (!check(low_altitude.algorithm_state == "HOLD_1M" && low_altitude.vz == 0.0 &&
                   low_altitude.vx == 0.0 && low_altitude.vy == 0.0,
               "altitude at hold threshold enters zero-velocity hold")) return 1;
    const auto lost = guidance.update(0.0, 0.0, 1.0, false, 0.05, 700.0);
    if (!check(lost.algorithm_state == "TARGET_LOST" && lost.vx == 0.0 &&
                   lost.vy == 0.0 && lost.vz == 0.0,
               "target loss reports zero command")) return 1;

    std::cout << "centered vertical descent signs, dwell, altitude hold, and safety passed\n";
    return 0;
}
