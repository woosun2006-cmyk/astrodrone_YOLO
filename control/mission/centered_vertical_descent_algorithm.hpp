#pragma once

#include <string>

#include "../pos_calculator.hpp"

namespace mission {

enum class CenteredVerticalDescentState {
    SEARCH,
    CENTERING,
    CENTER_DWELL,
    VERTICAL_DESCENT,
    HOLD_1M,
    TARGET_LOST,
    FAILED,
};

const char* centered_vertical_descent_state_name(CenteredVerticalDescentState state);

struct CenteredVerticalDescentConfig {
    double search_altitude_m = 5.0;
    double search_altitude_tolerance_m = 0.3;
    double center_hold_sec = 1.0;
    double hold_altitude_m = 1.0;
    double center_tolerance_px = 25.0;
    // A tighter tolerance is required before the pre-descent dwell starts.
    // The wider center_tolerance_px remains the tracking/re-centering band.
    double center_dwell_tolerance_px = 15.0;
    // Hysteresis for horizontal correction after descent has started.
    double descent_realign_enter_tolerance_px = 25.0;
    double descent_realign_exit_tolerance_px = 45.0;
    double descent_max_forward_speed_mps = 0.15;
    double descent_max_lateral_speed_mps = 0.15;
    double descent_max_vector_speed_mps = 0.15;
    PixelCalibration calibration{};
    double gain = 0.8;
    double max_forward_speed = 0.5;
    double max_lateral_speed = 0.5;
    double max_vector_speed = 0.5;
    double max_descent_speed_mps = 0.2;
    // <= 0 derives the steepest path angle that can use the configured
    // horizontal and descent speed limits together.
    double desired_path_angle_rad = 0.0;
    double horizontal_speed_deadband_mps = 0.01;
    double filter_alpha = 0.25;
    double max_acceleration_mps2 = 1.0;
    double max_jerk_mps3 = 5.0;
    double velocity_deadband_rad = 0.01;
    double velocity_hysteresis_rad = 0.005;
    double theta_safe_threshold_rad = 0.0872664626;  // 5 degrees
};

struct GuidanceDiagnostics {
    std::string algorithm_state;
    double theta_forward_rad = 0.0;
    double theta_right_rad = 0.0;
    double filtered_theta_forward_rad = 0.0;
    double filtered_theta_right_rad = 0.0;
    double body_forward_m = 0.0;
    double body_right_m = 0.0;
    double raw_vx = 0.0;
    double raw_vy = 0.0;
    double filtered_vx = 0.0;
    double filtered_vy = 0.0;
    double raw_vz = 0.0;
    double filtered_vz = 0.0;
    double horizontal_speed_mps = 0.0;
    double desired_path_angle_rad = 0.0;
    double vz_by_path_mps = 0.0;
    double vz_by_safety_mps = 0.0;
    double vx = 0.0;
    double vy = 0.0;
    double vz = 0.0;
    double theta_error_rad = 0.0;
    double descent_margin = 0.0;
    double commanded_path_angle_rad = 0.0;
    double acceleration_mps2 = 0.0;
    double jerk_mps3 = 0.0;
    double observation_age_ms = -1.0;
    bool angular_safe = false;
    bool descent_allowed = false;
    std::string safety_override;
};

class CenteredVerticalDescentAlgorithm {
public:
    explicit CenteredVerticalDescentAlgorithm(CenteredVerticalDescentConfig config);

    GuidanceDiagnostics update(double x_px, double y_px, double altitude_m,
                                bool fresh_target, double dt_sec,
                                double observation_age_ms,
                                bool allow_descent_without_horizontal = false,
                                bool allow_descent = true);

    // Safety and normal hold states bypass the filter and reset its history.
    GuidanceDiagnostics force_zero(std::string reason, double observation_age_ms = -1.0);

    // Center lock is an additional mission-level descent gate. Keep the
    // vertical filter from accumulating speed while horizontal alignment is
    // still being established.
    void reset_descent_state();

    CenteredVerticalDescentState state() const { return state_; }
    bool holding() const { return state_ == CenteredVerticalDescentState::HOLD_1M; }

private:
    double apply_deadband(double value, bool& active) const;
    void limit_velocity(double& vx, double& vy) const;
    void limit_descent_velocity(double& vx, double& vy) const;

    CenteredVerticalDescentConfig config_;
    CenteredVerticalDescentState state_ = CenteredVerticalDescentState::SEARCH;
    bool target_seen_ = false;
    double centered_since_sec_ = 0.0;
    bool center_dwell_started_ = false;
    bool initialized_ = false;
    bool forward_active_ = false;
    bool right_active_ = false;
    bool angular_safe_ = false;
    bool descent_realigning_ = false;
    double filtered_theta_forward_ = 0.0;
    double filtered_theta_right_ = 0.0;
    double previous_vx_ = 0.0;
    double previous_vy_ = 0.0;
    double previous_vz_ = 0.0;
    double previous_accel_x_ = 0.0;
    double previous_accel_y_ = 0.0;
    double previous_accel_z_ = 0.0;
};

}  // namespace mission
