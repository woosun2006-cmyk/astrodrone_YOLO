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
    // Diagnostic fields appended after the original payload. The two local
    // UDP peers are rebuilt together, so existing field offsets are retained.
    float normalized_x = 0;     // x_px / (frame_width / 2)
    float normalized_y = 0;     // y_px / (frame_height / 2)
    float body_forward_m = 0;   // +body-forward ground offset
    float body_right_m = 0;     // +body-right ground offset
    float observation_age_ms = -1;  // YOLO /target age; -1 means unavailable
    float theta_forward_rad = 0;  // signed bearing, +body-forward
    float theta_right_rad = 0;    // signed bearing, +body-right
    // Additive vision metadata for the onboard-to-GCS telemetry publisher.
    // These fields remain local to the two Repo A processes and are appended
    // so the existing range/guidance fields keep their offsets.
    float bbox_x_px = 0;
    float bbox_y_px = 0;
    float bbox_width_px = 0;
    float bbox_height_px = 0;
    float target_confidence = -1;
    uint8_t confirmed = 0;  // YOLO confirmation state, additive metadata
    uint64_t frame_sequence = 0;
    int64_t frame_timestamp_ns = -1;
    char class_name[32] = {};
    char source[96] = {};
    uint32_t frame_width = 640;
    uint32_t frame_height = 480;
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

// Local, non-MAVLink command metadata. control publishes the setpoint it has
// computed after CommandGate evaluation so the GCS telemetry publisher can
// display it without opening the command endpoint or decoding control logs.
struct GcsCommandStateMsg {
    uint32_t seq = 0;
    float vx = 0;
    float vy = 0;
    float vz = 0;
    float path_angle_rad = 0;
    uint8_t allowed = 0;
    char state[32] = {};
    char safety_reason[96] = {};
};

class GcsCommandStateSender {
public:
    explicit GcsCommandStateSender(int port);
    ~GcsCommandStateSender();
    GcsCommandStateSender(const GcsCommandStateSender&) = delete;
    GcsCommandStateSender& operator=(const GcsCommandStateSender&) = delete;

    void send(float vx, float vy, float vz, float path_angle_rad, bool allowed,
              const char* state, const char* safety_reason);

private:
    int fd_ = -1;
    uint32_t seq_ = 0;
};

class GcsCommandStateReceiver {
public:
    explicit GcsCommandStateReceiver(int port);
    ~GcsCommandStateReceiver();
    GcsCommandStateReceiver(const GcsCommandStateReceiver&) = delete;
    GcsCommandStateReceiver& operator=(const GcsCommandStateReceiver&) = delete;

    bool poll(GcsCommandStateMsg& out);

private:
    int fd_ = -1;
};
