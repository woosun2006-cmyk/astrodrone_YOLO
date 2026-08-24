#include "diagonal_approach_algorithm.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace mission {

const char* diagonal_approach_state_name(DiagonalApproachState state) {
    switch (state) {
        case DiagonalApproachState::SEARCH: return "SEARCH";
        case DiagonalApproachState::CENTERING: return "CENTERING";
        case DiagonalApproachState::APPROACH_45_DEG: return "APPROACH_45_DEG";
        case DiagonalApproachState::CENTER_DWELL: return "CENTER_DWELL";
        case DiagonalApproachState::HOLD_1M: return "HOLD_1M";
        case DiagonalApproachState::TARGET_LOST: return "TARGET_LOST";
        case DiagonalApproachState::FAILED: return "FAILED";
    }
    return "FAILED";
}

namespace {

constexpr double kHalfPi = 1.5707963267948966;

double clamp_value(double value, double low, double high) {
    return std::clamp(value, low, high);
}

}  // namespace

DiagonalApproachAlgorithm::DiagonalApproachAlgorithm(
    DiagonalApproachAlgorithmConfig config)
    : config_(std::move(config)) {
    config_.filter_alpha = clamp_value(config_.filter_alpha, 0.0, 1.0);
    config_.gain = std::max(0.0, config_.gain);
    config_.distance_gain = std::max(0.0, config_.distance_gain);
    config_.max_forward_speed = std::max(0.0, config_.max_forward_speed);
    config_.max_lateral_speed = std::max(0.0, config_.max_lateral_speed);
    config_.max_vector_speed = std::max(
        0.0, config_.max_vector_speed > 0.0
                 ? config_.max_vector_speed
                 : std::min(config_.max_forward_speed, config_.max_lateral_speed));
    config_.max_descent_speed_mps = std::max(0.0, config_.max_descent_speed_mps);
    config_.stop_distance_m = std::max(0.0, config_.stop_distance_m);
    config_.minimum_approach_speed_mps = std::max(0.0, config_.minimum_approach_speed_mps);
    config_.horizontal_speed_deadband_mps =
        std::max(0.0, config_.horizontal_speed_deadband_mps);
    config_.max_acceleration_mps2 = std::max(0.0, config_.max_acceleration_mps2);
    config_.max_jerk_mps3 = std::max(0.0, config_.max_jerk_mps3);
    config_.safe_fov_angle_rad = std::max(0.0, config_.safe_fov_angle_rad);
    if (config_.desired_path_angle_rad <= 0.0) {
        config_.desired_path_angle_rad = 0.7853981633974483;
    }
    config_.desired_path_angle_rad = std::clamp(
        config_.desired_path_angle_rad, 0.0, kHalfPi - 1e-6);

    const double path_limit = config_.desired_path_angle_rad > 1e-9
                                  ? config_.max_descent_speed_mps /
                                        std::tan(config_.desired_path_angle_rad)
                                  : config_.max_vector_speed;
    horizontal_limit_mps_ = std::max(0.0, std::min(config_.max_vector_speed, path_limit));
}

double DiagonalApproachAlgorithm::apply_deadband(double value, bool& active) const {
    const double magnitude = std::abs(value);
    if (active) {
        if (magnitude <= config_.velocity_deadband_rad) active = false;
    } else if (magnitude >= config_.velocity_deadband_rad +
                              config_.velocity_hysteresis_rad) {
        active = true;
    }
    return active ? value : 0.0;
}

void DiagonalApproachAlgorithm::limit_horizontal(double& vx, double& vy) const {
    vx = std::clamp(vx, -config_.max_forward_speed, config_.max_forward_speed);
    vy = std::clamp(vy, -config_.max_lateral_speed, config_.max_lateral_speed);
    const double magnitude = std::hypot(vx, vy);
    if (magnitude > horizontal_limit_mps_ && magnitude > 1e-12) {
        const double scale = horizontal_limit_mps_ / magnitude;
        vx *= scale;
        vy *= scale;
    }
}

GuidanceDiagnostics DiagonalApproachAlgorithm::update(
    double x_px, double y_px, double altitude_m, double distance_m,
    bool fresh_target, double dt_sec, double observation_age_ms) {
    const auto angles = downward_pixel_to_angles(x_px, y_px, config_.calibration);
    GuidanceDiagnostics output;
    output.theta_forward_rad = angles.forward_rad;
    output.theta_right_rad = angles.right_rad;
    output.observation_age_ms = observation_age_ms;

    if (!fresh_target || !std::isfinite(x_px) || !std::isfinite(y_px) ||
        !std::isfinite(altitude_m) || altitude_m <= 0.0 || !std::isfinite(distance_m)) {
        state_ = target_seen_ ? DiagonalApproachState::TARGET_LOST
                              : DiagonalApproachState::SEARCH;
        return force_zero("stale_or_invalid_observation", observation_age_ms);
    }
    target_seen_ = true;

    const bool centered = std::abs(x_px) <= config_.center_tolerance_px &&
                          std::abs(y_px) <= config_.center_tolerance_px;
    const bool final_centering = altitude_m <= config_.hold_altitude_m;
    if (final_centering) {
        state_ = centered ? DiagonalApproachState::HOLD_1M
                          : DiagonalApproachState::CENTERING;
    } else {
        state_ = centered ? DiagonalApproachState::APPROACH_45_DEG
                          : DiagonalApproachState::CENTERING;
    }

    const double alpha = config_.filter_alpha;
    if (!initialized_) {
        filtered_theta_forward_ = 0.0;
        filtered_theta_right_ = 0.0;
        initialized_ = true;
    }
    filtered_theta_forward_ += alpha * (angles.forward_rad - filtered_theta_forward_);
    filtered_theta_right_ += alpha * (angles.right_rad - filtered_theta_right_);
    output.filtered_theta_forward_rad = filtered_theta_forward_;
    output.filtered_theta_right_rad = filtered_theta_right_;
    output.theta_error_rad = std::hypot(filtered_theta_forward_, filtered_theta_right_);

    const bool in_safe_fov = std::abs(filtered_theta_forward_) <= config_.safe_fov_angle_rad &&
                             std::abs(filtered_theta_right_) <= config_.safe_fov_angle_rad;
    output.angular_safe = in_safe_fov;
    output.descent_allowed = !final_centering && in_safe_fov &&
                             output.theta_error_rad <= config_.theta_safe_threshold_rad;
    if (config_.theta_safe_threshold_rad > 0.0) {
        const double margin = clamp_value(
            1.0 - output.theta_error_rad / config_.theta_safe_threshold_rad, 0.0, 1.0);
        output.descent_margin = margin * margin * (3.0 - 2.0 * margin);
    }

    const double forward_theta = apply_deadband(filtered_theta_forward_, forward_active_);
    const double right_theta = apply_deadband(filtered_theta_right_, right_active_);
    output.body_forward_m = std::tan(forward_theta) * altitude_m;
    output.body_right_m = std::tan(right_theta) * altitude_m;

    // Keep the distance approach and pixel correction as separate terms. The
    // correction is deliberately bounded so a small image fluctuation cannot
    // cancel or reverse the latched approach direction.
    const double distance_error = std::max(0.0, distance_m - config_.stop_distance_m);
    if (!approach_direction_latched_) {
        approach_direction_sign_ = 1.0;
        approach_direction_latched_ = true;
    }
    const double approach_cap = std::min(config_.max_forward_speed,
                                         horizontal_limit_mps_);
    const double minimum_approach = std::min(config_.minimum_approach_speed_mps,
                                              approach_cap);
    const double approach_speed = final_centering
                                      ? 0.0
                                      : std::min(config_.max_forward_speed,
                                                 std::max(minimum_approach,
                                                          config_.distance_gain * distance_error));
    const double pixel_correction_limit = final_centering
                                               ? config_.max_forward_speed
                                               : 0.75 * approach_speed;
    const double pixel_forward_correction = std::clamp(
        config_.gain * output.body_forward_m,
        -pixel_correction_limit, pixel_correction_limit);
    output.raw_vx = approach_direction_sign_ * approach_speed + pixel_forward_correction;
    if (output.raw_vx * approach_direction_sign_ < 0.25 * approach_speed) {
        output.raw_vx = approach_direction_sign_ * (0.25 * approach_speed);
    }
    output.raw_vy = config_.gain * output.body_right_m;
    limit_horizontal(output.raw_vx, output.raw_vy);
    output.desired_path_angle_rad = config_.desired_path_angle_rad;

    // Keep approaching while the fresh target remains inside the camera's
    // safe FOV. The angular margin may be zero while the target is still far
    // from the center; using zero here would incorrectly stop horizontal
    // approach before stop_distance. The floor only preserves the configured
    // minimum approach component. Once the target leaves the safe FOV, all
    // axes are still stopped together.
    const double horizontal_before_safety =
        std::hypot(output.raw_vx, output.raw_vy);
    const double minimum_safety_scale =
        !final_centering && horizontal_before_safety > 1e-12
            ? std::min(1.0, minimum_approach / horizontal_before_safety)
            : 0.0;
    const double safety_scale = final_centering
                                    ? (in_safe_fov ? 1.0 : 0.0)
                                    : (in_safe_fov
                                           ? std::max(output.descent_margin,
                                                      minimum_safety_scale)
                                           : 0.0);
    output.raw_vx *= safety_scale;
    output.raw_vy *= safety_scale;
    output.filtered_vx = output.raw_vx;
    output.filtered_vy = output.raw_vy;
    output.horizontal_speed_mps = std::hypot(output.raw_vx, output.raw_vy);
    output.vz_by_path_mps = !final_centering && output.horizontal_speed_mps >
                                    config_.horizontal_speed_deadband_mps
                                ? std::tan(config_.desired_path_angle_rad) *
                                      output.horizontal_speed_mps
                                : 0.0;
    output.vz_by_safety_mps = config_.max_descent_speed_mps * safety_scale;
    output.raw_vz = output.vz_by_path_mps;

    // Smooth one scalar speed and then reconstruct all three components from
    // the same direction vector. This avoids per-axis filtering changing a
    // 45-degree command into a mostly vertical command.
    const double horizontal_speed = output.horizontal_speed_mps;
    const double path_tangent = std::tan(config_.desired_path_angle_rad);
    const double target_vz = !final_centering &&
                                     horizontal_speed > config_.horizontal_speed_deadband_mps
                                 ? std::min(config_.max_descent_speed_mps,
                                            path_tangent * horizontal_speed)
                                 : 0.0;
    const double target_speed = std::hypot(horizontal_speed, target_vz);
    const double direction_norm = std::hypot(horizontal_speed, target_vz);
    const double ux = direction_norm > 1e-12 ? output.raw_vx / direction_norm : 0.0;
    const double uy = direction_norm > 1e-12 ? output.raw_vy / direction_norm : 0.0;
    const double uz = direction_norm > 1e-12 ? target_vz / direction_norm : 0.0;

    const double dt = std::clamp(dt_sec, 0.001, 0.25);
    double scalar_accel = (target_speed - previous_speed_mps_) / dt;
    if (config_.max_acceleration_mps2 > 0.0) {
        scalar_accel = std::clamp(scalar_accel, -config_.max_acceleration_mps2,
                                  config_.max_acceleration_mps2);
    }
    if (config_.max_jerk_mps3 > 0.0) {
        const double max_accel_delta = config_.max_jerk_mps3 * dt;
        scalar_accel = std::clamp(
            scalar_accel, previous_scalar_accel_mps2_ - max_accel_delta,
            previous_scalar_accel_mps2_ + max_accel_delta);
    }
    double speed = std::max(0.0, previous_speed_mps_ + scalar_accel * dt);
    const double max_total_speed = std::hypot(horizontal_limit_mps_,
                                              config_.max_descent_speed_mps);
    speed = std::min(speed, max_total_speed);

    double vx = ux * speed;
    double vy = uy * speed;
    double vz = uz * speed;
    limit_horizontal(vx, vy);
    if (std::hypot(vx, vy) <= config_.horizontal_speed_deadband_mps) {
        vx = 0.0;
        vy = 0.0;
        vz = 0.0;
        speed = 0.0;
    }
    output.vx = vx;
    output.vy = vy;
    output.vz = vz;
    output.filtered_vz = vz;
    output.horizontal_speed_mps = std::hypot(vx, vy);
    output.commanded_path_angle_rad = output.horizontal_speed_mps >
                                               config_.horizontal_speed_deadband_mps
                                           ? std::atan2(std::abs(vz), output.horizontal_speed_mps)
                                           : 0.0;
    output.acceleration_mps2 = std::abs(scalar_accel);
    output.jerk_mps3 = std::abs(scalar_accel - previous_scalar_accel_mps2_) / dt;

    if (state_ == DiagonalApproachState::HOLD_1M) {
        output.vx = 0.0;
        output.vy = 0.0;
        output.vz = 0.0;
        output.raw_vx = 0.0;
        output.raw_vy = 0.0;
        output.raw_vz = 0.0;
        output.filtered_vx = 0.0;
        output.filtered_vy = 0.0;
        output.filtered_vz = 0.0;
        output.horizontal_speed_mps = 0.0;
        output.commanded_path_angle_rad = 0.0;
    }
    output.algorithm_state = diagonal_approach_state_name(state_);

    previous_speed_mps_ = speed;
    previous_scalar_accel_mps2_ = scalar_accel;
    return output;
}

GuidanceDiagnostics DiagonalApproachAlgorithm::force_zero(
    std::string reason, double observation_age_ms) {
    if (reason.find("target_loss") != std::string::npos ||
        reason.find("stale") != std::string::npos) {
        state_ = target_seen_ ? DiagonalApproachState::TARGET_LOST
                              : DiagonalApproachState::SEARCH;
    }
    reset_history();
    GuidanceDiagnostics output;
    output.observation_age_ms = observation_age_ms;
    output.safety_override = std::move(reason);
    output.algorithm_state = diagonal_approach_state_name(state_);
    return output;
}

void DiagonalApproachAlgorithm::reset_history() {
    initialized_ = false;
    approach_direction_latched_ = false;
    approach_direction_sign_ = 1.0;
    forward_active_ = false;
    right_active_ = false;
    filtered_theta_forward_ = 0.0;
    filtered_theta_right_ = 0.0;
    previous_speed_mps_ = 0.0;
    previous_scalar_accel_mps2_ = 0.0;
}

}  // namespace mission
