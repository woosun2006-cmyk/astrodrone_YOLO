#pragma once

#include <chrono>

#include "../target_link.hpp"

// Coordinate-based approach guidance - see README.md in this folder and
// Document/algorithm-renewer.md section 20 (2026-08-14 revision) for the
// design discussion behind this shape:
//
//   cruise (distance_m > shell_radius_m): constant cruise_speed_mps,
//     straight at the target - forward, lateral AND vertical components are
//     commanded in the SAME cycle (a true 3D diagonal), not "turn to face
//     it, then go forward" like control.cpp's approach_target(). The two
//     horizontal legs are split out of target.ground_offset_m against the
//     x_px-derived lateral offset; the vertical leg is target.altitude_m.
//     Their resultant is exactly distance_m, so the command points along the
//     real line of sight - see hybrid_guidance.cpp.
//
//     The vertical leg matters, it is not decoration: distance_m is a slant
//     range (hypot(ground_offset, altitude)), so it can never fall below the
//     vehicle's altitude. With a horizontal-only command the vehicle closes
//     the ground offset, distance_m bottoms out at the flight altitude, and
//     stop_radius_m is unreachable for any altitude above it. Descending
//     along the line of sight is what actually shrinks the slant range.
//   reverify (just crossed inside shell_radius_m): hold position, require
//     `tracking` to stay true for reverify_hold_sec before proceeding -
//     this is the control-layer's half of the two-gate false-positive
//     defense discussed in algorithm-renewer.md (gate 1 is the existing
//     YOLO_MODEL TARGET_CONFIRM_FRAMES streak upstream in TargetRangeMsg's
//     found/confirmed; the ROI-crop re-detection that would strengthen
//     gate 1 itself is NOT implemented here - that's Jetson-side perception
//     work, out of scope for this control-layer file).
//   decel (stop_radius_m < distance_m <= shell_radius_m): same diagonal
//     shape, but speed follows an exponential that's ~cruise_speed_mps
//     near the shell and drops steeply in the last meter or two before
//     stop_radius_m.
//   stopped (distance_m <= stop_radius_m): zero velocity, hard floor - the
//     exponential above never reaches exactly zero on its own, so this
//     clamp is what actually stops the vehicle.
//
// What this does NOT do: it does not freeze a fixed GPS/local-NED target
// point at "lock" time. Every cycle re-reads the latest x_px/distance_m
// from TargetRangeMsg and re-aims from there - simpler and self-correcting
// if the target or the vehicle's position estimate drifts, at the cost of
// needing the target to stay in frame (already true of the existing
// pipeline). It also does not implement adaptive ROI cropping - that is
// YOLO_MODEL's (perception) responsibility, upstream of target_distance.cpp.

struct HybridGuidanceConfig {
    double shell_radius_m = 5.0;  // outer boundary: cruise ends, re-verify gate, decel begins
    double stop_radius_m = 1.0;   // inner boundary: hard stop

    double cruise_speed_mps = 0.5;    // constant speed while distance_m > shell_radius_m
    double decel_rate_per_m = 0.536;  // k in v(d) = cruise_speed_mps * exp(-k*(shell_radius_m - d));
                                       // default tuned so v(shell_radius_m)=cruise_speed_mps and
                                       // v(shell_radius_m - 3m) ~= cruise_speed_mps/5 (0.5 -> 0.1 over 5m->2m)
    double reverify_hold_sec = 1.0;   // continuous `tracking` required after crossing shell_radius_m
                                       // before decel is allowed to start moving inward

    double max_yaw_rate = 0.6;       // rad/s, keeps the target roughly centered in frame throughout
    double k_yaw_px = 0.006;         // (rad/s) per pixel of x_px offset
    double focal_length_px = 530.0;  // reused from MAVLink.yaml's target_track.pixel_focal_length_px
};

enum class GuidanceMode {
    kHold,      // no fresh track - zero everything
    kCruise,    // distance_m > shell_radius_m: constant-speed diagonal approach
    kReverify,  // just inside shell_radius_m, holding position for the re-verify gate
    kDecel,     // stop_radius_m < distance_m <= shell_radius_m: exponential-decay diagonal approach
    kStopped,   // distance_m <= stop_radius_m: hard stop
};

struct GuidanceCommand {
    GuidanceMode mode = GuidanceMode::kHold;
    double vx = 0.0;        // body-frame forward speed (m/s), +forward
    double vy = 0.0;        // body-frame lateral speed (m/s), +right
    double vz = 0.0;        // body-frame vertical speed (m/s), +DOWN (NED), so a
                            // positive value descends - same sign convention
                            // hybrid_guidance_main.cpp's altitude-limit law uses.
    double yaw_rate = 0.0;  // +clockwise (right), same sign convention as control.cpp
};

// Caller-owned state carried across calls - tracks whether the reverify
// hold has been satisfied yet for the current approach (reset automatically
// whenever the vehicle drifts back outside shell_radius_m).
struct GuidanceState {
    bool reverified = false;
    bool reverify_timer_running = false;
    std::chrono::steady_clock::time_point reverify_since;
};

// Pure function apart from reading the clock for the reverify hold timer:
// no MAVLink here, hybrid_guidance_main.cpp owns sending.
GuidanceCommand compute_guidance(const TargetRangeMsg& target, bool tracking,
                                  const HybridGuidanceConfig& cfg, GuidanceState& state,
                                  std::chrono::steady_clock::time_point now);
