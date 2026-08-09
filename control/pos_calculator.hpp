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
