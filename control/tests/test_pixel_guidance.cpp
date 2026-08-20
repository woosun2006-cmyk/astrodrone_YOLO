#include <cmath>
#include <iostream>

#include "../mission/pixel_guidance.hpp"

namespace {

bool check(bool condition, const char* label) {
    if (!condition) std::cerr << "check failed: " << label << '\n';
    return condition;
}

}  // namespace

int main() {
    mission::PixelGuidanceConfig config;
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

    mission::PixelGuidance guidance(config);
    const auto top = guidance.update(0.0, 100.0, 5.0, true, 0.05, 20.0);
    if (!check(top.theta_forward_rad > 0.0 && top.vx > 0.0, "top -> +vx")) return 1;
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
    const auto braking = guidance.update(0.0, 0.0, 5.0, true, 0.05, 20.0);
    if (!check(std::abs(braking.vy) < std::abs(fast.vy), "smooth braking")) return 1;
    if (!check(std::abs(braking.vy) > 0.0, "braking is not an instant jump")) return 1;

    guidance.force_zero("deadband-reset");
    const auto small = guidance.update(0.0, 4.0, 5.0, true, 0.05, 20.0);
    if (!check(std::abs(small.vx) < 1e-9, "deadband")) return 1;
    const auto large = guidance.update(0.0, 20.0, 5.0, true, 0.05, 20.0);
    if (!check(large.vx > 0.0, "deadband activation")) return 1;
    const auto hysteresis = guidance.update(0.0, 7.0, 5.0, true, 0.05, 20.0);
    if (!check(hysteresis.vx > 0.0, "hysteresis hold")) return 1;
    const auto released = guidance.update(0.0, 4.0, 5.0, true, 0.05, 20.0);
    if (!check(std::abs(released.raw_vx) < 1e-9 && released.vx < hysteresis.vx,
               "hysteresis release")) return 1;

    guidance.force_zero("safety");
    const auto unsafe = guidance.update(0.0, 100.0, 5.0, true, 0.05, 20.0);
    if (!check(!unsafe.descent_allowed, "unsafe angle blocks descent")) return 1;
    guidance.force_zero("safety");
    const auto safe = guidance.update(0.0, 10.0, 5.0, true, 0.05, 20.0);
    if (!check(safe.descent_allowed, "safe angle allows descent")) return 1;

    guidance.force_zero("diagonal-reset");
    const auto forward_and_down = guidance.update(0.0, 10.0, 5.0, true, 0.05, 20.0);
    if (!check(forward_and_down.vx > 0.0 && forward_and_down.vz > 0.0,
               "forward and descent are simultaneous")) return 1;
    if (!check(forward_and_down.raw_vz > 0.0 && forward_and_down.descent_margin > 0.0,
               "descent margin produces raw vz")) return 1;
    if (!check(forward_and_down.acceleration_mps2 <= config.max_acceleration_mps2 + 1e-9,
               "3D acceleration cap")) return 1;

    guidance.force_zero("lateral-diagonal-reset");
    const auto lateral_and_down = guidance.update(20.0, 10.0, 5.0, true, 0.05, 20.0);
    if (!check(lateral_and_down.vx > 0.0 && lateral_and_down.vy > 0.0 &&
                   lateral_and_down.vz > 0.0,
               "lateral and descent are simultaneous")) return 1;
    if (!check(std::hypot(lateral_and_down.vx, lateral_and_down.vy) <=
                   config.max_vector_speed + 1e-9,
               "diagonal horizontal vector cap")) return 1;

    guidance.force_zero("path-low");
    const auto low_horizontal = guidance.update(0.0, 10.0, 5.0, true, 0.05, 20.0);
    guidance.force_zero("path-high");
    const auto high_horizontal = guidance.update(0.0, 20.0, 5.0, true, 0.05, 20.0);
    if (!check(low_horizontal.descent_margin > high_horizontal.descent_margin,
               "smooth descent margin decreases with angle")) return 1;
    if (!check(high_horizontal.horizontal_speed_mps > low_horizontal.horizontal_speed_mps &&
                   high_horizontal.raw_vz > low_horizontal.raw_vz,
               "path descent follows horizontal speed")) return 1;
    if (!check(low_horizontal.raw_vz <= low_horizontal.vz_by_path_mps + 1e-9 &&
                   low_horizontal.raw_vz <= low_horizontal.vz_by_safety_mps + 1e-9,
               "path and safety descent limits")) return 1;
    guidance.force_zero("horizontal-deadband");
    const auto centered = guidance.update(0.0, 0.0, 5.0, true, 0.05, 20.0);
    if (!check(centered.horizontal_speed_mps == 0.0 && centered.vz == 0.0,
               "horizontal deadband blocks descent")) return 1;

    guidance.force_zero("centered-descent-reset");
    const auto centered_descent =
        guidance.update(0.0, 0.0, 5.0, true, 0.05, 20.0, true);
    if (!check(centered_descent.vz > 0.0 &&
                   centered_descent.vz <= config.max_descent_speed_mps,
               "centered target may descend with the explicit mission gate")) return 1;

    guidance.force_zero("unsafe-reset");
    const auto unsafe_descent = guidance.update(0.0, 100.0, 5.0, true, 0.05, 20.0);
    if (!check(unsafe_descent.vz == 0.0 && unsafe_descent.raw_vz == 0.0 &&
                   unsafe_descent.descent_margin == 0.0,
               "unsafe angle produces zero descent")) return 1;

    const auto stale = guidance.force_zero("target_loss", 700.0);
    if (!check(stale.vx == 0.0 && stale.vy == 0.0 && stale.vz == 0.0 &&
                   stale.safety_override == "target_loss",
               "stale immediate zero")) return 1;

    std::cout << "pixel guidance signs, smoothing, vector cap, deadband, and descent gate passed\n";
    return 0;
}
