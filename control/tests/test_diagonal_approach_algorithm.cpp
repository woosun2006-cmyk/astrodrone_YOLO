#include <cmath>
#include <iostream>

#include "../mission/diagonal_approach_algorithm.hpp"

namespace {

bool check(bool condition, const char* label) {
    if (!condition) std::cerr << "check failed: " << label << '\n';
    return condition;
}

mission::DiagonalApproachAlgorithmConfig config() {
    mission::DiagonalApproachAlgorithmConfig config;
    config.calibration = PixelCalibration{530.0, 530.0, 320.0, 240.0};
    config.gain = 1.0;
    config.distance_gain = 0.2;
    config.max_forward_speed = 0.5;
    config.max_lateral_speed = 0.5;
    config.max_vector_speed = 0.5;
    config.max_descent_speed_mps = 0.2;
    config.desired_path_angle_rad = 0.7853981633974483;
    config.stop_distance_m = 1.0;
    config.minimum_approach_speed_mps = 0.05;
    config.filter_alpha = 1.0;
    config.max_acceleration_mps2 = 100.0;
    config.max_jerk_mps3 = 100.0;
    config.theta_safe_threshold_rad = 0.2;
    config.safe_fov_angle_rad = 1.0;
    config.hold_altitude_m = 1.0;
    config.center_tolerance_px = 25.0;
    return config;
}

}  // namespace

int main() {
    mission::DiagonalApproachAlgorithm guidance(config());

    mission::GuidanceDiagnostics front;
    for (int i = 0; i < 20; ++i) {
        front = guidance.update(0.0, 0.0, 5.0, 5.0, true, 0.05, 20.0);
    }
    if (!check(front.vx > 0.0 && front.vz > 0.0,
               "centered target keeps forward and descent simultaneous")) return 1;
    if (!check(std::abs(front.vy) < 1e-9, "front target has normal vy=0")) return 1;
    if (!check(std::abs(front.commanded_path_angle_rad -
                       std::atan2(std::abs(front.vz), std::hypot(front.vx, front.vy))) < 1e-9,
               "commanded path angle uses command payload")) return 1;
    if (!check(front.vx <= 0.2 + 1e-6 && front.vz <= 0.2 + 1e-6,
               "45 degree profile respects coupled speed limits")) return 1;

    guidance.force_zero("pixel-correction-reset");
    const auto opposing_pixel = guidance.update(0.0, -30.0, 5.0, 5.0, true, 0.05, 20.0);
    if (!check(opposing_pixel.vx > 0.0,
               "opposing pixel correction cannot cancel approach")) return 1;

    guidance.force_zero("direction-latch-reset");
    const auto first_direction = guidance.update(0.0, -30.0, 5.0, 5.0, true, 0.05, 20.0);
    const auto noisy_direction = guidance.update(0.0, 30.0, 5.0, 5.0, true, 0.05, 20.0);
    if (!check(first_direction.vx > 0.0 && noisy_direction.vx > 0.0,
               "approach direction remains latched across pixel sign changes")) return 1;

    guidance.force_zero("fresh-approach-reset");
    for (int i = 0; i < 25; ++i) {
        const auto persistent = guidance.update(0.0, -30.0, 5.0, 5.0, true, 0.05, 20.0);
        if (!check(persistent.vx > 0.0 && persistent.vz > 0.0,
                   "fresh target does not produce prolonged zero velocity")) return 1;
    }

    guidance.force_zero("safety-scale-reset");
    const auto safety_scaled = guidance.update(0.0, 30.0, 5.0, 5.0, true, 0.05, 20.0);
    const auto expected_config = config();
    if (!check(safety_scaled.vx > 0.0 && safety_scaled.vz > 0.0,
               "safety scale keeps a safe approach moving")) return 1;
    if (!check(std::abs(safety_scaled.commanded_path_angle_rad -
                        expected_config.desired_path_angle_rad) < 1e-6,
               "safety scale preserves the command path angle")) return 1;

    guidance.force_zero("safe-fov-reset");
    // The target is still inside the camera FOV but outside the tighter
    // centered-descent angle. Approach must not become a zero vector.
    const auto safe_fov_edge = guidance.update(0.0, 170.0, 5.0, 3.72, true,
                                               0.05, 20.0);
    if (!check(safe_fov_edge.vx > 0.0 && safe_fov_edge.vz > 0.0,
               "safe-FOV target keeps minimum diagonal approach")) return 1;
    if (!check(std::abs(safe_fov_edge.commanded_path_angle_rad -
                        expected_config.desired_path_angle_rad) < 1e-6,
               "safe-FOV minimum approach preserves path angle")) return 1;

    guidance.force_zero("lateral-reset");
    mission::GuidanceDiagnostics lateral;
    for (int i = 0; i < 20; ++i) {
        lateral = guidance.update(20.0, 10.0, 5.0, 5.0, true, 0.05, 20.0);
    }
    if (!check(lateral.vx > 0.0 && lateral.vy > 0.0 && lateral.vz > 0.0,
               "lateral target produces vx+vy+vz")) return 1;

    guidance.force_zero("no-horizontal-reset");
    auto no_horizontal_config = config();
    no_horizontal_config.max_forward_speed = 0.0;
    no_horizontal_config.max_lateral_speed = 0.0;
    no_horizontal_config.max_vector_speed = 0.0;
    mission::DiagonalApproachAlgorithm no_horizontal(no_horizontal_config);
    const auto no_horizontal_output =
        no_horizontal.update(0.0, 0.0, 5.0, 5.0, true, 0.05, 20.0);
    if (!check(no_horizontal_output.vx == 0.0 && no_horizontal_output.vy == 0.0 &&
                   no_horizontal_output.vz == 0.0,
               "no horizontal speed never sends vz-only")) return 1;

    guidance.force_zero("final-centering-reset");
    const auto final_centering = guidance.update(0.0, 80.0, 1.0, 3.0, true, 0.05, 20.0);
    if (!check(final_centering.vx > 0.0 && std::abs(final_centering.vy) < 1e-9 &&
                   final_centering.vz == 0.0,
               "final centering uses horizontal velocity without descent")) return 1;

    const auto centered_at_final_distance =
        guidance.update(0.0, 0.0, 1.0, 3.0, true, 0.05, 20.0);
    if (!check(centered_at_final_distance.vx == 0.0 &&
                   centered_at_final_distance.vy == 0.0 &&
                   centered_at_final_distance.vz == 0.0,
               "centered final-distance target can hold zero velocity")) return 1;

    guidance.force_zero("altitude-hold-reset");
    const auto distance_far_at_low_altitude =
        guidance.update(0.0, 0.0, 0.9, 5.0, true, 0.05, 20.0);
    if (!check(distance_far_at_low_altitude.algorithm_state == "HOLD_1M" &&
                   distance_far_at_low_altitude.vx == 0.0 &&
                   distance_far_at_low_altitude.vy == 0.0 &&
                   distance_far_at_low_altitude.vz == 0.0,
               "altitude, not distance, controls diagonal hold")) return 1;

    const auto stale = guidance.force_zero("target_loss", 700.0);
    if (!check(stale.vx == 0.0 && stale.vy == 0.0 && stale.vz == 0.0,
               "target loss forces zero")) return 1;

    std::cout << "diagonal approach latch, minimum speed, path-angle, and safety tests passed\n";
    return 0;
}
