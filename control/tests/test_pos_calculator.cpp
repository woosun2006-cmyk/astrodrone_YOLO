#include <cmath>
#include <iostream>

#include "pos_calculator.hpp"

int main() {
    constexpr double focal = 530.0;
    constexpr double epsilon = 1e-9;
    const auto near = [](double actual, double expected) {
        return std::abs(actual - expected) < epsilon;
    };
    const auto check = [](bool condition, const char* label) {
        if (!condition) std::cerr << "check failed: " << label << '\n';
        return condition;
    };

    if (!check(near(pixel_offset_to_ground_m(100.0, 2.0, 500.0), 0.4), "pixel offset")) return 1;
    if (!check(near(downward_target_bearing_px(0.0, 100.0, focal), 0.0), "center bearing")) return 1;
    if (!check(near(downward_target_bearing_px(100.0, 100.0, focal), focal * M_PI / 4.0),
               "right bearing")) return 1;
    if (!check(near(downward_target_bearing_px(-100.0, 100.0, focal), -focal * M_PI / 4.0),
               "left bearing")) return 1;

    // Downward-camera guidance convention: YOLO emits +x to image-right and
    // +y to image-up. Runtime pose probing confirms image-up is
    // body-forward for this fixed downward camera.
    const auto center = body_velocity_from_downward_pixel_error(
        0.0, 0.0, 5.0, 530.0, 0.8, 0.1);
    if (!check(near(center.vx, 0.0) && near(center.vy, 0.0),
               "center -> zero velocity")) return 1;

    const auto top = body_velocity_from_downward_pixel_error(
        0.0, 100.0, 5.0, 530.0, 0.8, 0.1);
    if (!check(top.vx > 0.0 && near(top.vy, 0.0),
               "top -> positive forward velocity")) return 1;

    const auto bottom = body_velocity_from_downward_pixel_error(
        0.0, -100.0, 5.0, 530.0, 0.8, 0.1);
    if (!check(bottom.vx < 0.0 && near(bottom.vy, 0.0),
               "bottom -> negative forward velocity")) return 1;

    const auto right = body_velocity_from_downward_pixel_error(
        100.0, 0.0, 5.0, 530.0, 0.8, 0.1);
    if (!check(near(right.vx, 0.0) && right.vy > 0.0,
               "right -> positive right velocity")) return 1;

    const auto left = body_velocity_from_downward_pixel_error(
        -100.0, 0.0, 5.0, 530.0, 0.8, 0.1);
    if (!check(near(left.vx, 0.0) && left.vy < 0.0,
               "left -> negative right velocity")) return 1;

    std::cout << "synthetic signs: center=(0,0), top=vx>0, bottom=vx<0, "
                 "left=vy<0, right=vy>0\n";
    return 0;
}
