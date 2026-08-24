#include "centered_vertical_descent_algorithm.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace mission {

const char* centered_vertical_descent_state_name(CenteredVerticalDescentState state) {
    switch (state) {
        case CenteredVerticalDescentState::SEARCH: return "SEARCH";
        case CenteredVerticalDescentState::CENTERING: return "CENTERING";
        case CenteredVerticalDescentState::CENTER_DWELL: return "CENTER_DWELL";
        case CenteredVerticalDescentState::VERTICAL_DESCENT: return "VERTICAL_DESCENT";
        case CenteredVerticalDescentState::HOLD_1M: return "HOLD_1M";
        case CenteredVerticalDescentState::TARGET_LOST: return "TARGET_LOST";
        case CenteredVerticalDescentState::FAILED: return "FAILED";
    }
    return "FAILED";
}

namespace {

double clamp_value(double value, double low, double high) {
    return std::clamp(value, low, high);
}

bool is_search_hold_state(CenteredVerticalDescentState state) {
    return state == CenteredVerticalDescentState::CENTERING ||
           state == CenteredVerticalDescentState::CENTER_DWELL;
}

}  // namespace

CenteredVerticalDescentAlgorithm::CenteredVerticalDescentAlgorithm(
    CenteredVerticalDescentConfig config) : config_(std::move(config)) {
    config_.filter_alpha = clamp_value(config_.filter_alpha, 0.0, 1.0);
    config_.max_forward_speed = std::max(0.0, config_.max_forward_speed);
    config_.max_lateral_speed = std::max(0.0, config_.max_lateral_speed);
    if (config_.max_vector_speed <= 0.0) {
        config_.max_vector_speed = std::min(config_.max_forward_speed,
                                            config_.max_lateral_speed);
    }
    config_.max_vector_speed = std::max(0.0, config_.max_vector_speed);
    config_.max_descent_speed_mps = std::max(0.0, config_.max_descent_speed_mps);
    config_.center_tolerance_px = std::max(0.0, config_.center_tolerance_px);
    config_.center_dwell_tolerance_px = std::clamp(
        config_.center_dwell_tolerance_px, 0.0, config_.center_tolerance_px);
    config_.descent_realign_enter_tolerance_px =
        std::max(0.0, config_.descent_realign_enter_tolerance_px);
    config_.descent_realign_exit_tolerance_px = std::max(
        config_.descent_realign_enter_tolerance_px,
        config_.descent_realign_exit_tolerance_px);
    config_.descent_max_forward_speed_mps =
        std::max(0.0, config_.descent_max_forward_speed_mps);
    config_.descent_max_lateral_speed_mps =
        std::max(0.0, config_.descent_max_lateral_speed_mps);
    if (config_.descent_max_vector_speed_mps <= 0.0) {
        config_.descent_max_vector_speed_mps = std::min(
            config_.descent_max_forward_speed_mps,
            config_.descent_max_lateral_speed_mps);
    }
    config_.descent_max_vector_speed_mps = std::max(
        0.0, config_.descent_max_vector_speed_mps);
    config_.horizontal_speed_deadband_mps =
        std::max(0.0, config_.horizontal_speed_deadband_mps);
    if (config_.desired_path_angle_rad <= 0.0) {
        const double horizontal_limit = config_.max_vector_speed;
        config_.desired_path_angle_rad = horizontal_limit > 1e-12
                                             ? std::atan2(config_.max_descent_speed_mps,
                                                          horizontal_limit)
                                             : 0.0;
    }
    constexpr double kHalfPi = 1.5707963267948966;
    config_.desired_path_angle_rad = std::clamp(
        config_.desired_path_angle_rad, 0.0, kHalfPi - 1e-6);
    config_.max_acceleration_mps2 = std::max(0.0, config_.max_acceleration_mps2);
    config_.max_jerk_mps3 = std::max(0.0, config_.max_jerk_mps3);
}

double CenteredVerticalDescentAlgorithm::apply_deadband(double value, bool& active) const {
    const double magnitude = std::abs(value);
    if (active) {
        if (magnitude <= config_.velocity_deadband_rad) active = false;
    } else if (magnitude >= config_.velocity_deadband_rad +
                              config_.velocity_hysteresis_rad) {
        active = true;
    }
    return active ? value : 0.0;
}

void CenteredVerticalDescentAlgorithm::limit_velocity(double& vx, double& vy) const {
    vx = std::clamp(vx, -config_.max_forward_speed, config_.max_forward_speed);
    vy = std::clamp(vy, -config_.max_lateral_speed, config_.max_lateral_speed);
    const double magnitude = std::hypot(vx, vy);
    if (magnitude > config_.max_vector_speed && magnitude > 1e-12) {
        const double scale = config_.max_vector_speed / magnitude;
        vx *= scale;
        vy *= scale;
    }
}

void CenteredVerticalDescentAlgorithm::limit_descent_velocity(double& vx,
                                                               double& vy) const {
    vx = std::clamp(vx, -config_.descent_max_forward_speed_mps,
                    config_.descent_max_forward_speed_mps);
    vy = std::clamp(vy, -config_.descent_max_lateral_speed_mps,
                    config_.descent_max_lateral_speed_mps);
    const double magnitude = std::hypot(vx, vy);
    if (magnitude > config_.descent_max_vector_speed_mps && magnitude > 1e-12) {
        const double scale = config_.descent_max_vector_speed_mps / magnitude;
        vx *= scale;
        vy *= scale;
    }
}

GuidanceDiagnostics CenteredVerticalDescentAlgorithm::update(double x_px, double y_px, double altitude_m,
                                          bool fresh_target, double dt_sec,
                                          double observation_age_ms,
                                          bool allow_descent_without_horizontal,
                                          bool allow_descent) {
    const auto angles = downward_pixel_to_angles(x_px, y_px, config_.calibration);
    GuidanceDiagnostics output;
    output.theta_forward_rad = angles.forward_rad;
    output.theta_right_rad = angles.right_rad;
    output.observation_age_ms = observation_age_ms;

    if (!fresh_target || !std::isfinite(x_px) || !std::isfinite(y_px) ||
        !std::isfinite(altitude_m) || altitude_m <= 0.0) {
        state_ = target_seen_ ? CenteredVerticalDescentState::TARGET_LOST
                              : CenteredVerticalDescentState::SEARCH;
        center_dwell_started_ = false;
        return force_zero("stale_or_invalid_observation", observation_age_ms);
    }
    target_seen_ = true;

    const bool centered = std::abs(x_px) <= config_.center_tolerance_px &&
                          std::abs(y_px) <= config_.center_tolerance_px;
    const bool centered_for_dwell =
        std::abs(x_px) <= config_.center_dwell_tolerance_px &&
        std::abs(y_px) <= config_.center_dwell_tolerance_px;
    const double center_error_px = std::max(std::abs(x_px), std::abs(y_px));
    const double dt = std::clamp(dt_sec, 0.001, 0.25);
    if (altitude_m <= config_.hold_altitude_m) {
        state_ = CenteredVerticalDescentState::HOLD_1M;
        descent_realigning_ = false;
    } else if (state_ == CenteredVerticalDescentState::VERTICAL_DESCENT) {
        // Descent uses hysteresis. Small errors are corrected while descending;
        // only a large error pauses vz, and recovery returns directly to descent.
        if (descent_realigning_) {
            if (center_error_px <= config_.descent_realign_enter_tolerance_px) {
                descent_realigning_ = false;
            }
        } else if (center_error_px >= config_.descent_realign_exit_tolerance_px) {
            descent_realigning_ = true;
        }
        state_ = CenteredVerticalDescentState::VERTICAL_DESCENT;
    } else if (!centered_for_dwell) {
        state_ = CenteredVerticalDescentState::CENTERING;
        center_dwell_started_ = false;
        centered_since_sec_ = 0.0;
    } else {
        if (!center_dwell_started_) {
            center_dwell_started_ = true;
            centered_since_sec_ = 0.0;
        } else {
            centered_since_sec_ += dt;
        }
        state_ = centered_since_sec_ >= config_.center_hold_sec
                     ? CenteredVerticalDescentState::VERTICAL_DESCENT
                     : CenteredVerticalDescentState::CENTER_DWELL;
        if (state_ == CenteredVerticalDescentState::VERTICAL_DESCENT) {
            descent_realigning_ = false;
        }
    }

    // A transition back to alignment must not inherit vertical descent from a
    // previous VERTICAL_DESCENT cycle. Horizontal filtering remains intact,
    // while the vertical filter is reset for the 5m search/centering phase.
    const bool search_hold_state = is_search_hold_state(state_);
    const bool center_dwell_state =
        state_ == CenteredVerticalDescentState::CENTER_DWELL;
    const bool descent_realign_state =
        state_ == CenteredVerticalDescentState::VERTICAL_DESCENT &&
        descent_realigning_;
    if (search_hold_state) {
        previous_vz_ = 0.0;
        previous_accel_z_ = 0.0;
    }
    if (center_dwell_state) {
        // Dwell is a completed alignment check, not another centering pass.
        // Clear horizontal filter history so the dwell command is stationary.
        previous_vx_ = 0.0;
        previous_vy_ = 0.0;
        previous_accel_x_ = 0.0;
        previous_accel_y_ = 0.0;
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
    const double angular_error = std::hypot(filtered_theta_forward_, filtered_theta_right_);
    output.theta_error_rad = angular_error;
    if (angular_safe_) {
        if (angular_error > config_.theta_safe_threshold_rad +
                            config_.velocity_hysteresis_rad) {
            angular_safe_ = false;
        }
    } else if (angular_error < config_.theta_safe_threshold_rad -
                                    config_.velocity_hysteresis_rad) {
        angular_safe_ = true;
    }
    output.angular_safe = angular_safe_;
    output.descent_allowed = allow_descent &&
                             state_ == CenteredVerticalDescentState::VERTICAL_DESCENT &&
                             altitude_m > config_.hold_altitude_m &&
                             fresh_target && angular_safe_ &&
                             angular_error < config_.theta_safe_threshold_rad;

    if (config_.theta_safe_threshold_rad > 0.0) {
        const double margin = clamp_value(
            1.0 - angular_error / config_.theta_safe_threshold_rad, 0.0, 1.0);
        output.descent_margin = margin * margin * (3.0 - 2.0 * margin);
    }

    const double forward_theta = apply_deadband(filtered_theta_forward_, forward_active_);
    const double right_theta = apply_deadband(filtered_theta_right_, right_active_);
    const bool vertical_descent_active =
        state_ == CenteredVerticalDescentState::VERTICAL_DESCENT;
    output.body_forward_m = std::tan(forward_theta) * altitude_m;
    output.body_right_m = std::tan(right_theta) * altitude_m;
    output.raw_vx = config_.gain * output.body_forward_m;
    output.raw_vy = config_.gain * output.body_right_m;
    limit_velocity(output.raw_vx, output.raw_vy);
    if (vertical_descent_active) {
        limit_descent_velocity(output.raw_vx, output.raw_vy);
    }
    output.filtered_vx = output.raw_vx;
    output.filtered_vy = output.raw_vy;
    output.horizontal_speed_mps = std::hypot(output.filtered_vx, output.filtered_vy);
    if (center_dwell_state) {
        output.raw_vx = 0.0;
        output.raw_vy = 0.0;
        output.filtered_vx = 0.0;
        output.filtered_vy = 0.0;
        output.horizontal_speed_mps = 0.0;
    }
    output.desired_path_angle_rad = config_.desired_path_angle_rad;
    output.vz_by_safety_mps = output.descent_allowed
                                  ? config_.max_descent_speed_mps * output.descent_margin
                                  : 0.0;
    output.vz_by_path_mps = vertical_descent_active && output.horizontal_speed_mps >
                                    config_.horizontal_speed_deadband_mps
                                ? std::tan(config_.desired_path_angle_rad) *
                                      output.horizontal_speed_mps
                                : 0.0;
    output.raw_vz = vertical_descent_active && output.descent_allowed
                       ? ((output.horizontal_speed_mps >
                           config_.horizontal_speed_deadband_mps ||
                           allow_descent_without_horizontal)
                              ? std::min(output.vz_by_path_mps > 0.0
                                             ? output.vz_by_path_mps
                                             : output.vz_by_safety_mps,
                                         output.vz_by_safety_mps)
                              : 0.0)
                       : 0.0;
    if (descent_realign_state) {
        output.raw_vz = 0.0;
        output.vz_by_path_mps = 0.0;
        output.vz_by_safety_mps = 0.0;
        output.descent_allowed = false;
        output.descent_margin = 0.0;
        previous_vz_ = 0.0;
        previous_accel_z_ = 0.0;
    }
    output.filtered_vz = output.raw_vz;

    double desired_accel_x = (output.raw_vx - previous_vx_) / dt;
    double desired_accel_y = (output.raw_vy - previous_vy_) / dt;
    double desired_accel_z = (output.raw_vz - previous_vz_) / dt;
    const double desired_accel = std::sqrt(desired_accel_x * desired_accel_x +
                                           desired_accel_y * desired_accel_y +
                                           desired_accel_z * desired_accel_z);
    if (desired_accel > config_.max_acceleration_mps2 && desired_accel > 1e-12) {
        const double scale = config_.max_acceleration_mps2 / desired_accel;
        desired_accel_x *= scale;
        desired_accel_y *= scale;
        desired_accel_z *= scale;
    }

    double accel_x = desired_accel_x;
    double accel_y = desired_accel_y;
    double accel_z = desired_accel_z;
    const double accel_delta_x = accel_x - previous_accel_x_;
    const double accel_delta_y = accel_y - previous_accel_y_;
    const double accel_delta_z = accel_z - previous_accel_z_;
    const double accel_delta = std::sqrt(accel_delta_x * accel_delta_x +
                                         accel_delta_y * accel_delta_y +
                                         accel_delta_z * accel_delta_z);
    const double max_accel_delta = config_.max_jerk_mps3 * dt;
    if (accel_delta > max_accel_delta && accel_delta > 1e-12) {
        const double scale = max_accel_delta / accel_delta;
        accel_x = previous_accel_x_ + accel_delta_x * scale;
        accel_y = previous_accel_y_ + accel_delta_y * scale;
        accel_z = previous_accel_z_ + accel_delta_z * scale;
    }
    const double applied_accel = std::sqrt(accel_x * accel_x + accel_y * accel_y +
                                           accel_z * accel_z);
    if (applied_accel > config_.max_acceleration_mps2 && applied_accel > 1e-12) {
        const double scale = config_.max_acceleration_mps2 / applied_accel;
        accel_x *= scale;
        accel_y *= scale;
        accel_z *= scale;
    }

    double vx = previous_vx_ + accel_x * dt;
    double vy = previous_vy_ + accel_y * dt;
    double vz = previous_vz_ + accel_z * dt;
    limit_velocity(vx, vy);
    if (vertical_descent_active) {
        limit_descent_velocity(vx, vy);
    }
    vz = std::clamp(vz, 0.0, config_.max_descent_speed_mps);
    output.vx = vx;
    output.vy = vy;
    output.vz = vz;
    if (search_hold_state) {
        // CENTERING and CENTER_DWELL only position the target at the search
        // altitude. They never intentionally descend.
        output.raw_vz = 0.0;
        output.filtered_vz = 0.0;
        output.vz_by_path_mps = 0.0;
        output.vz_by_safety_mps = 0.0;
        output.vz = 0.0;
    }
    if (descent_realign_state) {
        output.filtered_vz = 0.0;
        output.vz = 0.0;
        output.commanded_path_angle_rad = 0.0;
    }
    const double commanded_horizontal_speed = std::hypot(vx, vy);
    output.commanded_path_angle_rad =
        commanded_horizontal_speed > config_.horizontal_speed_deadband_mps
            ? std::atan2(std::abs(vz), commanded_horizontal_speed)
            : 0.0;
    output.acceleration_mps2 = std::sqrt(accel_x * accel_x + accel_y * accel_y +
                                         accel_z * accel_z);
    output.jerk_mps3 = std::sqrt(
                           (accel_x - previous_accel_x_) * (accel_x - previous_accel_x_) +
                           (accel_y - previous_accel_y_) * (accel_y - previous_accel_y_) +
                           (accel_z - previous_accel_z_) * (accel_z - previous_accel_z_)) /
                       dt;
    if (state_ == CenteredVerticalDescentState::HOLD_1M) {
        output.vx = 0.0;
        output.vy = 0.0;
        output.vz = 0.0;
        output.filtered_vx = 0.0;
        output.filtered_vy = 0.0;
        output.filtered_vz = 0.0;
        output.raw_vx = 0.0;
        output.raw_vy = 0.0;
        output.raw_vz = 0.0;
        output.horizontal_speed_mps = 0.0;
        output.commanded_path_angle_rad = 0.0;
    }
    output.algorithm_state = centered_vertical_descent_state_name(state_);
    previous_vx_ = vx;
    previous_vy_ = vy;
    previous_vz_ = vz;
    previous_accel_x_ = accel_x;
    previous_accel_y_ = accel_y;
    previous_accel_z_ = accel_z;
    return output;
}

GuidanceDiagnostics CenteredVerticalDescentAlgorithm::force_zero(std::string reason, double observation_age_ms) {
    if (reason.find("target_loss") != std::string::npos ||
        reason.find("stale") != std::string::npos) {
        state_ = target_seen_ ? CenteredVerticalDescentState::TARGET_LOST
                              : CenteredVerticalDescentState::SEARCH;
    }
    initialized_ = false;
    forward_active_ = false;
    right_active_ = false;
    angular_safe_ = false;
    filtered_theta_forward_ = 0.0;
    filtered_theta_right_ = 0.0;
    previous_vx_ = 0.0;
    previous_vy_ = 0.0;
    previous_vz_ = 0.0;
    previous_accel_x_ = 0.0;
    previous_accel_y_ = 0.0;
    previous_accel_z_ = 0.0;
    centered_since_sec_ = 0.0;
    center_dwell_started_ = false;
    descent_realigning_ = false;
    GuidanceDiagnostics output;
    output.observation_age_ms = observation_age_ms;
    output.safety_override = std::move(reason);
    output.algorithm_state = centered_vertical_descent_state_name(state_);
    return output;
}

void CenteredVerticalDescentAlgorithm::reset_descent_state() {
    previous_vz_ = 0.0;
    previous_accel_z_ = 0.0;
}

}  // namespace mission
