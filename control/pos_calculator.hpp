#pragma once

// Stage 1 (distance calc) of the target-approach pipeline: converts a
// target's pixel offset from image center into a ground-plane offset in
// meters, using the vehicle's altitude - see target_distance.cpp for how
// this combines with altitude via Pythagoras to get a straight-line range.
//
// Pinhole-camera model instead of a flat multiplier: the same pixel offset
// covers more real-world distance the higher up the camera is, so the
// conversion must scale with altitude (see control/README.md's prior note
// on this). focal_length_px is the camera's focal length in pixel units;
// pass setting/MAVLink.yaml's target_track.pixel_focal_length_px.
//
//   거리 = 픽셀 / focal_length_px * 고도
double pixel_offset_to_ground_m(double pixel_offset, double altitude_m, double focal_length_px);

// Converts a downward-camera target offset into the horizontal pixel error
// convention emitted by YOLO (+x right, +y image-up).
double downward_target_bearing_px(double x_px, double y_px, double focal_length_px);

struct PixelCalibration {
    double fx_px = 530.0;
    double fy_px = 530.0;
    double cx_px = 320.0;
    double cy_px = 240.0;
};

struct DownwardPixelAngles {
    double forward_rad = 0.0;  // +up in the image -> +body-forward
    double right_rad = 0.0;    // +right in the image -> +body-right
};

DownwardPixelAngles downward_pixel_to_angles(
    double x_px, double y_px, const PixelCalibration& calibration);

struct DownwardTargetPixels {
    double x_px = 0.0;  // +right
    double y_px = 0.0;  // +image-up; body-forward is the opposite sign
};

struct DownwardBodyOffset {
    double forward_m = 0.0;  // +body-forward; image-up is +body-forward here
    double right_m = 0.0;    // +body-right; image-right is +body-right here
};

// Convert the YOLO image convention (+x right, +y image-up) to the fixed
// downward camera's body-frame ground offset. The SDF sensor pose is
// optical +X -> body -Z, sensor +Z -> body +X, and sensor +Y -> body +Y.
DownwardBodyOffset downward_pixel_to_body_offset(
    double x_px, double y_px, double altitude_m, double focal_length_px);

DownwardBodyOffset downward_pixel_to_body_offset(
    double x_px, double y_px, double altitude_m, const PixelCalibration& calibration);

struct BodyVelocityCorrection {
    double vx = 0.0;  // +body-forward
    double vy = 0.0;  // +body-right
};

// Apply the existing proportional correction and cap without hiding the
// axis/sign conversion inside control.cpp. This is intentionally a pure
// helper so synthetic sign tests can exercise the exact production mapping.
BodyVelocityCorrection body_velocity_from_downward_pixel_error(
    double x_px, double y_px, double altitude_m, double focal_length_px,
    double gain, double max_speed);

// Projects a fixed target in the ArduPilot LOCAL_NED frame into the same
// ideal downward-camera pixel coordinates produced by YOLO. Image-up is
// body-forward for this fixed camera. This lets the
// normal range and control pipeline be exercised without detector dropouts.
DownwardTargetPixels fixed_ned_target_to_pixels(
    double target_north_m,
    double target_east_m,
    double vehicle_north_m,
    double vehicle_east_m,
    double vehicle_yaw_rad,
    double altitude_m,
    double focal_length_px);

DownwardTargetPixels fixed_ned_target_to_pixels(
    double target_north_m,
    double target_east_m,
    double vehicle_north_m,
    double vehicle_east_m,
    double vehicle_yaw_rad,
    double altitude_m,
    double focal_x_px,
    double focal_y_px);
