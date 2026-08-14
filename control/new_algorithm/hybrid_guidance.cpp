#include "hybrid_guidance.hpp"

#include "../pos_calculator.hpp"

#include <algorithm>
#include <cmath>

namespace {

// Decomposes the already-known scalar range (target.distance_m) and the
// already-known lateral offset (x_px converted to meters) into orthogonal
// forward/lateral legs via Pythagoras - lateral^2 + forward^2 = distance^2.
// This is what lets one cycle's velocity command point diagonally straight
// at the target instead of turning to face it first and only moving
// forward afterward (control.cpp's approach_target() does the latter).
struct BodyVector {
    double forward_m;
    double lateral_m;
};

BodyVector decompose(double distance_m, double lateral_offset_m) {
    double lateral_clamped = std::clamp(lateral_offset_m, -distance_m, distance_m);
    double forward = std::sqrt(std::max(0.0, distance_m * distance_m - lateral_clamped * lateral_clamped));
    return {forward, lateral_clamped};
}

}  // namespace

GuidanceCommand compute_guidance(const TargetRangeMsg& target, bool tracking,
                                  const HybridGuidanceConfig& cfg, GuidanceState& state,
                                  std::chrono::steady_clock::time_point now) {
    GuidanceCommand cmd;
    if (!tracking) {
        // Losing track resets the reverify hold - a target that disappears
        // and reappears right at the shell boundary must re-earn it, not
        // resume a stale timer.
        state.reverify_timer_running = false;
        cmd.mode = GuidanceMode::kHold;
        return cmd;
    }

    double distance_m = target.distance_m;
    double px_offset = static_cast<double>(target.x_px);  // +right, same as control.cpp
    double lateral_offset_m = pixel_offset_to_ground_m(px_offset, target.altitude_m, cfg.focal_length_px);

    // Keep the target roughly centered throughout, independent of mode -
    // this is what keeps it in frame for the reverify gate below.
    cmd.yaw_rate = std::clamp(cfg.k_yaw_px * px_offset, -cfg.max_yaw_rate, cfg.max_yaw_rate);

    if (distance_m <= cfg.stop_radius_m) {
        // Hard floor: the exponential decel law below approaches zero
        // asymptotically but never reaches it, so this is what actually
        // stops the vehicle (Document/algorithm-renewer.md 2026-08-14
        // discussion on the 2m->1m residual).
        cmd.mode = GuidanceMode::kStopped;
        cmd.vx = 0.0;
        cmd.vy = 0.0;
        return cmd;
    }

    if (distance_m > cfg.shell_radius_m) {
        // Back outside the shell - the reverify hold has to be earned
        // again next time it's crossed.
        state.reverify_timer_running = false;
        state.reverified = false;
        cmd.mode = GuidanceMode::kCruise;
        BodyVector bv = decompose(distance_m, lateral_offset_m);
        double scale = cfg.cruise_speed_mps / distance_m;  // resultant magnitude == cruise_speed_mps
        cmd.vx = bv.forward_m * scale;
        cmd.vy = bv.lateral_m * scale;
        return cmd;
    }

    // Inside the shell: hold here until re-verified continuously for
    // reverify_hold_sec - the control-layer half of the two-gate
    // false-positive defense (see hybrid_guidance.hpp's top comment).
    if (!state.reverified) {
        if (!state.reverify_timer_running) {
            state.reverify_timer_running = true;
            state.reverify_since = now;
        }
        double held = std::chrono::duration<double>(now - state.reverify_since).count();
        if (held < cfg.reverify_hold_sec) {
            cmd.mode = GuidanceMode::kReverify;
            cmd.vx = 0.0;
            cmd.vy = 0.0;
            return cmd;
        }
        state.reverified = true;
    }

    // Decel band: v(shell_radius_m) == cruise_speed_mps, decaying
    // exponentially as distance_m falls toward stop_radius_m (hard-floored
    // to exactly 0 by the stop_radius_m check above).
    cmd.mode = GuidanceMode::kDecel;
    double speed = cfg.cruise_speed_mps * std::exp(-cfg.decel_rate_per_m * (cfg.shell_radius_m - distance_m));
    BodyVector bv = decompose(distance_m, lateral_offset_m);
    double scale = speed / distance_m;
    cmd.vx = bv.forward_m * scale;
    cmd.vy = bv.lateral_m * scale;
    return cmd;
}
