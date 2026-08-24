#pragma once

#include <string>

#include "centered_vertical_descent_algorithm.hpp"

namespace mission {

enum class DiagonalApproachState {
    SEARCH,
    CENTERING,
    APPROACH_45_DEG,
    CENTER_DWELL,
    HOLD_1M,
    TARGET_LOST,
    FAILED,
};

const char* diagonal_approach_state_name(DiagonalApproachState state);

// Configuration for the opt-in distance-plus-pixel approach. The centered
// vertical descent algorithm is the default baseline.
struct DiagonalApproachAlgorithmConfig {
    PixelCalibration calibration{};
    double gain = 0.8;
    double distance_gain = 0.15;
    double max_forward_speed = 0.5;
    double max_lateral_speed = 0.5;
    double max_vector_speed = 0.5;
    double max_descent_speed_mps = 0.2;
    double desired_path_angle_rad = 0.7853981633974483;
    double center_tolerance_px = 25.0;
    // Distance remains an approach input; altitude controls final descent stop.
    double stop_distance_m = 1.0;
    double hold_altitude_m = 1.0;
    double minimum_approach_speed_mps = 0.05;
    double horizontal_speed_deadband_mps = 0.01;
    double filter_alpha = 0.25;
    double max_acceleration_mps2 = 1.0;
    double max_jerk_mps3 = 5.0;
    double velocity_deadband_rad = 0.01;
    double velocity_hysteresis_rad = 0.005;
    double theta_safe_threshold_rad = 0.0872664626;
    double safe_fov_angle_rad = 0.6;
};

class DiagonalApproachAlgorithm {
public:
    explicit DiagonalApproachAlgorithm(DiagonalApproachAlgorithmConfig config);

    GuidanceDiagnostics update(double x_px, double y_px, double altitude_m,
                               double distance_m, bool fresh_target,
                               double dt_sec, double observation_age_ms);

    GuidanceDiagnostics force_zero(std::string reason, double observation_age_ms = -1.0);
    DiagonalApproachState state() const { return state_; }
    bool holding() const { return state_ == DiagonalApproachState::HOLD_1M; }

private:
    double apply_deadband(double value, bool& active) const;
    void limit_horizontal(double& vx, double& vy) const;
    void reset_history();

    DiagonalApproachAlgorithmConfig config_;
    DiagonalApproachState state_ = DiagonalApproachState::SEARCH;
    bool target_seen_ = false;
    double horizontal_limit_mps_ = 0.0;
    bool initialized_ = false;
    bool approach_direction_latched_ = false;
    double approach_direction_sign_ = 1.0;
    bool forward_active_ = false;
    bool right_active_ = false;
    double filtered_theta_forward_ = 0.0;
    double filtered_theta_right_ = 0.0;
    double previous_speed_mps_ = 0.0;
    double previous_scalar_accel_mps2_ = 0.0;
};

}  // namespace mission
