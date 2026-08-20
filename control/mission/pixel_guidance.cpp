#include "pixel_guidance.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace mission {

namespace {

double clamp_value(double value, double low, double high) {
    return std::clamp(value, low, high);
}

}  // namespace

PixelGuidance::PixelGuidance(PixelGuidanceConfig config) : config_(std::move(config)) {
    config_.filter_alpha = clamp_value(config_.filter_alpha, 0.0, 1.0);
    config_.max_forward_speed = std::max(0.0, config_.max_forward_speed);
    config_.max_lateral_speed = std::max(0.0, config_.max_lateral_speed);
    if (config_.max_vector_speed <= 0.0) {
        config_.max_vector_speed = std::min(config_.max_forward_speed,
                                            config_.max_lateral_speed);
    }
    config_.max_vector_speed = std::max(0.0, config_.max_vector_speed);
    config_.max_descent_speed_mps = std::max(0.0, config_.max_descent_speed_mps);
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

double PixelGuidance::apply_deadband(double value, bool& active) const {
    const double magnitude = std::abs(value);
    if (active) {
        if (magnitude <= config_.velocity_deadband_rad) active = false;
    } else if (magnitude >= config_.velocity_deadband_rad +
                              config_.velocity_hysteresis_rad) {
        active = true;
    }
    return active ? value : 0.0;
}

void PixelGuidance::limit_velocity(double& vx, double& vy) const {
    vx = std::clamp(vx, -config_.max_forward_speed, config_.max_forward_speed);
    vy = std::clamp(vy, -config_.max_lateral_speed, config_.max_lateral_speed);
    const double magnitude = std::hypot(vx, vy);
    if (magnitude > config_.max_vector_speed && magnitude > 1e-12) {
        const double scale = config_.max_vector_speed / magnitude;
        vx *= scale;
        vy *= scale;
    }
}

GuidanceDiagnostics PixelGuidance::update(double x_px, double y_px, double altitude_m,
                                          bool fresh_confirmed_target, double dt_sec,
                                          double observation_age_ms,
                                          bool allow_descent_without_horizontal) {
    const auto angles = downward_pixel_to_angles(x_px, y_px, config_.calibration);
    GuidanceDiagnostics output;
    output.theta_forward_rad = angles.forward_rad;
    output.theta_right_rad = angles.right_rad;
    output.observation_age_ms = observation_age_ms;

    if (!fresh_confirmed_target || !std::isfinite(x_px) || !std::isfinite(y_px) ||
        !std::isfinite(altitude_m) || altitude_m <= 0.0) {
        return force_zero("stale_or_invalid_observation", observation_age_ms);
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
    output.descent_allowed = fresh_confirmed_target && angular_safe_ &&
                             angular_error < config_.theta_safe_threshold_rad;

    if (config_.theta_safe_threshold_rad > 0.0) {
        const double margin = clamp_value(
            1.0 - angular_error / config_.theta_safe_threshold_rad, 0.0, 1.0);
        output.descent_margin = margin * margin * (3.0 - 2.0 * margin);
    }

    const double forward_theta = apply_deadband(filtered_theta_forward_, forward_active_);
    const double right_theta = apply_deadband(filtered_theta_right_, right_active_);
    output.body_forward_m = std::tan(forward_theta) * altitude_m;
    output.body_right_m = std::tan(right_theta) * altitude_m;
    output.raw_vx = config_.gain * output.body_forward_m;
    output.raw_vy = config_.gain * output.body_right_m;
    limit_velocity(output.raw_vx, output.raw_vy);
    output.filtered_vx = output.raw_vx;
    output.filtered_vy = output.raw_vy;
    output.horizontal_speed_mps = std::hypot(output.filtered_vx, output.filtered_vy);
    output.desired_path_angle_rad = config_.desired_path_angle_rad;
    output.vz_by_safety_mps = output.descent_allowed
                                  ? config_.max_descent_speed_mps * output.descent_margin
                                  : 0.0;
    output.vz_by_path_mps = output.horizontal_speed_mps >
                                    config_.horizontal_speed_deadband_mps
                                ? std::tan(config_.desired_path_angle_rad) *
                                      output.horizontal_speed_mps
                                : 0.0;
    output.raw_vz = output.descent_allowed
                       ? ((output.horizontal_speed_mps >
                           config_.horizontal_speed_deadband_mps ||
                           allow_descent_without_horizontal)
                              ? std::min(output.vz_by_path_mps > 0.0
                                             ? output.vz_by_path_mps
                                             : output.vz_by_safety_mps,
                                         output.vz_by_safety_mps)
                              : 0.0)
                       : 0.0;
    output.filtered_vz = output.raw_vz;

    const double dt = std::clamp(dt_sec, 0.001, 0.25);
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
    vz = std::clamp(vz, 0.0, config_.max_descent_speed_mps);
    output.vx = vx;
    output.vy = vy;
    output.vz = vz;
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
    previous_vx_ = vx;
    previous_vy_ = vy;
    previous_vz_ = vz;
    previous_accel_x_ = accel_x;
    previous_accel_y_ = accel_y;
    previous_accel_z_ = accel_z;
    return output;
}

GuidanceDiagnostics PixelGuidance::force_zero(std::string reason, double observation_age_ms) {
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
    GuidanceDiagnostics output;
    output.observation_age_ms = observation_age_ms;
    output.safety_override = std::move(reason);
    return output;
}

void PixelGuidance::reset_descent_state() {
    previous_vz_ = 0.0;
    previous_accel_z_ = 0.0;
}

}  // namespace mission
