#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "mav_transport.hpp"
#include "yaml_settings.hpp"

// Common dialect only: MAV_CMD_*, HEARTBEAT, SET_POSITION_TARGET_LOCAL_NED,
// etc. are all defined here; ArduCopter custom-mode numbers below are not
// part of any dialect (ArduPilot assigns them at the firmware level), so
// they're hardcoded to match ArduCopter's Mode::Number enum.
#include "third_party/mavlink/common/mavlink.h"

// Thin wrapper around a MAVLink byte stream: owns the transport, tracks
// parser state, and remembers the target system/component learned from the
// first heartbeat -- mirrors what pymavlink's mavutil.MAVLink connection
// object gives the scripts in this project.
class MavConnection {
public:
    explicit MavConnection(std::unique_ptr<Transport> transport);

    // Blocks until a HEARTBEAT is seen (updating target_system/component) or
    // timeout_sec elapses. Returns false on timeout, matching
    // master.wait_heartbeat(timeout=...) returning None.
    bool wait_heartbeat(double timeout_sec, mavlink_heartbeat_t* out = nullptr);

    // Blocks until a message whose msgid is in msg_ids (or any message, if
    // msg_ids is empty) arrives, or timeout_sec elapses.
    bool recv_match(const std::vector<uint32_t>& msg_ids, mavlink_message_t& out,
                     double timeout_sec);

    void send(const mavlink_message_t& msg);

    uint8_t target_system() const { return target_system_; }
    uint8_t target_component() const { return target_component_; }

private:
    std::unique_ptr<Transport> transport_;
    uint8_t target_system_ = 0;
    uint8_t target_component_ = 0;
};

// Opens a connection without waiting for a heartbeat, mirroring
// mavutil.mavlink_connection(address, baud=...).
std::unique_ptr<MavConnection> open_connection(const std::string& address, int baud = 115200);

// ArduCopter custom-mode name -> number (STABILIZE, GUIDED, AUTO, LAND, ...).
const std::map<std::string, uint32_t>& copter_mode_mapping();

bool is_armed_from_heartbeat(const mavlink_heartbeat_t& hb);

namespace drone {

YamlValue load_mavlink_settings();

// connect/set_mode/arm_disarm/... operate on a single module-level
// connection, mirroring drone_lib.py's global `master`.
MavConnection& connect(const std::string& address, double heartbeat_timeout = 20.0);
MavConnection& require_connection();

bool set_mode(const std::string& mode);
void arm_disarm(bool arm);
void takeoff(double altitude);
void send_velocity(double vx, double vy, double vz, double yaw_rate = 0.0);
// Same as send_velocity(), but in MAV_FRAME_BODY_OFFSET_NED: vx/vy/vz are
// forward/right/down relative to the vehicle's current heading instead of
// north/east/down. Use this for camera-relative guidance (e.g.
// target_distance.cpp's pixel offsets), since send_velocity()'s world-frame
// axes only line up with "forward" when the vehicle happens to face north.
void send_velocity_body(double vx, double vy, double vz, double yaw_rate = 0.0);
void land();

}  // namespace drone
