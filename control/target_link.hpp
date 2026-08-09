#pragma once

#include <cstdint>

// Loopback-UDP handoff from target_distance.cpp (stage 1: range calc) to
// control.cpp (stage 2/3: velocity calc + send to Pixhawk). Both run on the
// same Jetson, built by the same compiler, so the struct's raw bytes are
// sent as-is - no wire encoding needed, this never leaves localhost.
struct TargetRangeMsg {
    uint32_t seq = 0;
    uint8_t valid = 0;  // altitude AND a fresh YOLO reading were both available this cycle
    uint8_t found = 0;  // a target is currently detected (subset of valid)
    // A fresh ALTITUDE reading from the flight controller arrived within
    // target_track.altitude_stale_ms - independent of `found`/`valid` above,
    // which also require a target detection. altitude_m below holds the
    // last known reading either way (for logging), but only trust it as
    // current when this is 1: target_distance.cpp does NOT zero it out or
    // stop sending on a stale reading, it just stops asserting freshness.
    uint8_t altitude_valid = 0;
    float x_px = 0;     // target offset from image center, +right (setting/cam_sets.yaml coord_origin)
    float y_px = 0;     // target offset from image center, +up
    float altitude_m = 0;       // ALTITUDE.altitude_relative, m above home (same as check_alt.cpp)
    float ground_offset_m = 0;  // hypot(x_px, y_px) scaled by target_track.pixel_to_meter
    float distance_m = 0;       // hypot(ground_offset_m, altitude_m) - the Pythagorean combine
};

// Sends TargetRangeMsg datagrams to 127.0.0.1:port. Fire-and-forget: a
// dropped datagram is superseded by the next one a cycle later, so send()
// does not retry.
class TargetRangeSender {
public:
    explicit TargetRangeSender(int port);
    ~TargetRangeSender();
    TargetRangeSender(const TargetRangeSender&) = delete;
    TargetRangeSender& operator=(const TargetRangeSender&) = delete;

    void send(const TargetRangeMsg& msg);

private:
    int fd_ = -1;
};

// Binds 127.0.0.1:port and reads whatever TargetRangeMsg datagrams have
// arrived since the last poll(), keeping only the newest.
class TargetRangeReceiver {
public:
    explicit TargetRangeReceiver(int port);
    ~TargetRangeReceiver();
    TargetRangeReceiver(const TargetRangeReceiver&) = delete;
    TargetRangeReceiver& operator=(const TargetRangeReceiver&) = delete;

    // Returns true and fills `out` if at least one datagram arrived since
    // the last call. Never blocks.
    bool poll(TargetRangeMsg& out);

private:
    int fd_ = -1;
};
