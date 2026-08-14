# control/new_algorithm/

Coordinate-based approach guidance law from `Document/algorithm-renewer.md`
section 20 (2026-08-14 revision): constant-speed cruise toward the target
until a 5m "shell" radius, a re-verify hold right at that boundary,
exponential-decay speed from there down to a 1m hard stop. Every cycle's
forward/lateral velocity split is a Pythagorean decomposition of the known
range and the known lateral pixel offset, so the approach is diagonal
(straight at the target) rather than "turn to face it, then go forward"
like `control.cpp`'s `approach_target()`. See `Document/algorithm.md` for
the full narrative writeup of the whole flight (takeoff through final
approach).

Nothing outside this folder is modified. Everything reusable from `control/`
is used as-is:

| reused as-is | new here |
|---|---|
| `drone_lib.hpp/.cpp` (`MavConnection`, `drone::connect/set_mode/arm_disarm/takeoff/land/send_velocity_body`) | `hybrid_guidance.hpp/.cpp` - the cruise/reverify/decel/stop state machine |
| `target_link.hpp/.cpp` (`TargetRangeReceiver`/`TargetRangeMsg` - already published by the unmodified `target_distance.cpp`) | `hybrid_guidance_main.cpp` - entry point wiring it together |
| `pos_calculator.hpp/.cpp` (pixel-offset -> ground-meters pinhole model) | `CMakeLists.txt` - standalone build (see below) |
| `yaml_settings.hpp/.cpp` (`parse_yaml_file`, `with_port`) | `setting/hybrid_guidance.yaml` - new tunables file |
| `setting/MAVLink.yaml` / `safety.yaml` / `port.yaml` (read only, via `drone::load_*_settings()`) | |

`control.cpp` itself is not reused/called - it is a full alternative binary,
not something this links against. This program self-launches (GUIDED -> arm
-> takeoff -> approach loop) the same way `control.cpp`'s default flow does;
it does **not** use `control.cpp`'s `run_auto_intercept()`/`wait_for_lock()`
path, so it is unaffected by (and does not fix) the `HealthState` race
`control/README.md` and `gazeboSim/README.md` document there.

**Do not run this alongside `control`** - both bind `target_track.udp_port`
(15020) as a `TargetRangeReceiver`, and only one process can receive that
UDP stream at a time.

## Why a standalone CMakeLists.txt

`control/CMakeLists.txt` is not touched (no `add_subdirectory`), so this
folder has its own, which compiles `control/`'s existing `.cpp` sources
again (not copies with changes - the same files, referenced by relative
path) into a separate `hybrid_guidance` binary. That binary's output
directory is pinned to `control/build_new_algorithm/` - two levels below
the repo root, the same depth as `control/build/` - because both
`yaml_settings.cpp`'s built-in settings loader and this folder's own
`load_hybrid_guidance_settings()` resolve `setting/*.yaml` relative to
*this binary's own path* (`/proc/self/exe`), not the source tree.

## Build

```bash
cd control/new_algorithm
mkdir -p build && cd build
cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo ..
make -j"$(nproc)"
# binary lands at control/build_new_algorithm/hybrid_guidance
```

`scripts/run_new_algorithm.sh` does this for you.

## Tuning

See `setting/hybrid_guidance.yaml`: `shell_radius_m`/`stop_radius_m` set the
outer/inner boundaries, `cruise_speed_mps`/`decel_rate_per_m` set the speed
profile (0.5 m/s cruise, exponential decay inside the shell, hard-floored
to 0 at `stop_radius_m`), `reverify_hold_sec` sets how long `tracking` must
hold after crossing the shell before the decel phase is allowed to move
inward, `max_yaw_rate`/`k_yaw_px` keep the target centered throughout.

## Known limitation

Adaptive ROI cropping / re-detection cadence (`Document/algorithm-renewer.md`
sections 12-13, and the quadrant-scan + 680x680 crop design discussed
2026-08-14) is **not implemented here** - that is perception work
(`YOLO_MODEL/`), needs the real Jetson camera/TensorRT pipeline to validate,
and is out of scope for this control-layer folder. This guidance layer only
consumes the `found`/`confirmed` signal already carried in `TargetRangeMsg`
(YOLO_MODEL's existing `TARGET_CONFIRM_FRAMES`-in-a-row streak) as the first
false-positive gate, and adds its own `reverify_hold_sec` hold at the
5m-shell crossing as a second gate - see `Document/algorithm.md`.
