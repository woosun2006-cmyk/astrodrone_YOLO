#include "pos_calculator.hpp"

#include <algorithm>
#include <cmath>

double pixel_offset_to_ground_m(double pixel_offset, double altitude_m, double focal_length_px) {
    return pixel_offset / focal_length_px * altitude_m;
}

double downward_target_bearing_px(double x_px, double y_px, double focal_length_px) {
    return focal_length_px * std::atan2(x_px, y_px);
}

DownwardPixelAngles downward_pixel_to_angles(
    double x_px, double y_px, const PixelCalibration& calibration) {
    // x_px/y_px are already offsets from the principal point. Reconstructing
    // u/v here makes the calibrated cx/cy explicit without changing the
    // established +image-up/+image-right convention.
    const double u_px = calibration.cx_px + x_px;
    const double v_px = calibration.cy_px - y_px;
    return {
        std::atan2(calibration.cy_px - v_px, calibration.fy_px),
        std::atan2(u_px - calibration.cx_px, calibration.fx_px),
    };
}

DownwardTargetPixels fixed_ned_target_to_pixels(
    double target_north_m,
    double target_east_m,
    double vehicle_north_m,
    double vehicle_east_m,
    double vehicle_yaw_rad,
    double altitude_m,
    double focal_length_px) {
    return fixed_ned_target_to_pixels(target_north_m, target_east_m, vehicle_north_m,
                                      vehicle_east_m, vehicle_yaw_rad, altitude_m,
                                      focal_length_px, focal_length_px);
}

DownwardTargetPixels fixed_ned_target_to_pixels(
    double target_north_m,
    double target_east_m,
    double vehicle_north_m,
    double vehicle_east_m,
    double vehicle_yaw_rad,
    double altitude_m,
    double focal_x_px,
    double focal_y_px) {
    double north = target_north_m - vehicle_north_m;
    double east = target_east_m - vehicle_east_m;
    double forward = std::cos(vehicle_yaw_rad) * north +
                     std::sin(vehicle_yaw_rad) * east;
    double right = -std::sin(vehicle_yaw_rad) * north +
                   std::cos(vehicle_yaw_rad) * east;
    double projection_altitude = std::max(altitude_m, 0.05);
    return {
        right / projection_altitude * focal_x_px,
        forward / projection_altitude * focal_y_px,
    };
}

DownwardBodyOffset downward_pixel_to_body_offset(
    double x_px, double y_px, double altitude_m, double focal_length_px) {
    return downward_pixel_to_body_offset(
        x_px, y_px, altitude_m,
        PixelCalibration{focal_length_px, focal_length_px, 0.0, 0.0});
}

DownwardBodyOffset downward_pixel_to_body_offset(
    double x_px, double y_px, double altitude_m, const PixelCalibration& calibration) {
    const double projection_altitude = std::max(altitude_m, 0.05);
    const auto angles = downward_pixel_to_angles(x_px, y_px, calibration);
    return {
        std::tan(angles.forward_rad) * projection_altitude,
        std::tan(angles.right_rad) * projection_altitude,
    };
}

BodyVelocityCorrection body_velocity_from_downward_pixel_error(
    double x_px, double y_px, double altitude_m, double focal_length_px,
    double gain, double max_speed) {
    const auto offset = downward_pixel_to_body_offset(
        x_px, y_px, altitude_m, focal_length_px);
    return {
        std::clamp(gain * offset.forward_m, -max_speed, max_speed),
        std::clamp(gain * offset.right_m, -max_speed, max_speed),
    };
}
