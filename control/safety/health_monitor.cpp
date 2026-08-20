#include "health_monitor.hpp"

#include <iomanip>
#include <sstream>

#include "../drone_lib.hpp"

namespace safety {

void apply_health_message(const mavlink_message_t& message, HealthState& state,
                          std::chrono::steady_clock::time_point now) {
    switch (message.msgid) {
        case MAVLINK_MSG_ID_SYS_STATUS: {
            mavlink_sys_status_t status;
            mavlink_msg_sys_status_decode(&message, &status);
            state.battery_percent = status.battery_remaining;
            state.battery_voltage_v =
                (status.voltage_battery == 0 || status.voltage_battery == 65535)
                    ? -1
                    : status.voltage_battery / 1000.0;
            state.battery_valid = status.voltage_battery > 0 &&
                                  status.voltage_battery != 65535 &&
                                  status.battery_remaining >= 0;
            state.prearm_healthy =
                (status.onboard_control_sensors_health & MAV_SYS_STATUS_PREARM_CHECK) != 0;
            state.have_battery = true;
            state.have_sys_status = true;
            state.battery_updated_at = now;
            break;
        }
        case MAVLINK_MSG_ID_ALTITUDE: {
            mavlink_altitude_t altitude;
            mavlink_msg_altitude_decode(&message, &altitude);
            state.altitude_m = altitude.altitude_relative;
            state.have_altitude = true;
            state.altitude_updated_at = now;
            break;
        }
        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
            mavlink_global_position_int_t position;
            mavlink_msg_global_position_int_decode(&message, &position);
            state.altitude_m = static_cast<double>(position.relative_alt) / 1000.0;
            state.have_altitude = true;
            state.altitude_updated_at = now;
            break;
        }
        case MAVLINK_MSG_ID_GPS_RAW_INT: {
            mavlink_gps_raw_int_t gps;
            mavlink_msg_gps_raw_int_decode(&message, &gps);
            state.fix_type = gps.fix_type;
            state.satellites = gps.satellites_visible;
            state.have_gps = true;
            state.gps_updated_at = now;
            if (gps.fix_type >= 2) {
                state.lat = gps.lat / 1e7;
                state.lon = gps.lon / 1e7;
                state.have_position = true;
            }
            break;
        }
        case MAVLINK_MSG_ID_EKF_STATUS_REPORT: {
            mavlink_ekf_status_report_t ekf;
            mavlink_msg_ekf_status_report_decode(&message, &ekf);
            state.ekf_pos_horiz_variance = ekf.pos_horiz_variance;
            state.ekf_velocity_variance = ekf.velocity_variance;
            state.have_ekf = true;
            state.ekf_updated_at = now;
            break;
        }
        case MAVLINK_MSG_ID_HEARTBEAT: {
            mavlink_heartbeat_t heartbeat;
            if (!is_valid_ardupilot_heartbeat(message, &heartbeat)) return;
            state.system_status = heartbeat.system_status;
            state.custom_mode = heartbeat.custom_mode;
            state.armed =
                (heartbeat.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0;
            state.have_armed = true;
            state.last_heartbeat = now;
            state.mode_updated_at = now;
            break;
        }
        default:
            break;
    }
}

std::vector<std::string> evaluate_health_breach(
    const HealthState& state, const HealthLimit& limits,
    std::chrono::steady_clock::time_point now) {
    std::vector<std::string> reasons;

    double heartbeat_gap = std::chrono::duration<double>(now - state.last_heartbeat).count();
    if (heartbeat_gap > limits.max_heartbeat_gap_sec) {
        std::ostringstream oss;
        oss << "heartbeat gap " << std::fixed << std::setprecision(1) << heartbeat_gap
            << "s > " << limits.max_heartbeat_gap_sec << "s";
        reasons.push_back(oss.str());
    }
    if (state.have_battery && state.battery_percent >= 0 &&
        state.battery_percent < limits.min_battery_percent) {
        reasons.push_back("battery " + std::to_string(state.battery_percent) + "% < " +
                          std::to_string(limits.min_battery_percent) + "%");
    }
    if (state.have_battery && state.battery_voltage_v >= 0 &&
        state.battery_voltage_v < limits.min_battery_voltage_v) {
        std::ostringstream oss;
        oss << "battery " << std::fixed << std::setprecision(2) << state.battery_voltage_v
            << "V < " << limits.min_battery_voltage_v << "V";
        reasons.push_back(oss.str());
    }
    if (state.have_gps && state.satellites != 255 &&
        (state.fix_type < limits.min_fix_type || state.satellites < limits.min_satellites)) {
        reasons.push_back("gps fix_type=" + std::to_string(static_cast<int>(state.fix_type)) +
                          " satellites=" + std::to_string(static_cast<int>(state.satellites)));
    }
    if (state.have_sys_status && limits.require_prearm_healthy && !state.prearm_healthy) {
        reasons.push_back("prearm check unhealthy");
    }
    if (state.have_sys_status && limits.require_normal_state &&
        (state.system_status == MAV_STATE_CRITICAL ||
         state.system_status == MAV_STATE_EMERGENCY)) {
        reasons.push_back("system_status=" +
                          std::to_string(static_cast<int>(state.system_status)) +
                          " (CRITICAL/EMERGENCY - ArduPilot internal failsafe, e.g. EKF)");
    }
    if (state.have_ekf &&
        (state.ekf_pos_horiz_variance > limits.ekf_pos_horiz_variance_max ||
         state.ekf_velocity_variance > limits.ekf_velocity_variance_max)) {
        std::ostringstream oss;
        oss << "ekf pos_horiz_variance=" << std::fixed << std::setprecision(2)
            << state.ekf_pos_horiz_variance << " velocity_variance="
            << state.ekf_velocity_variance;
        reasons.push_back(oss.str());
    }
    return reasons;
}

void HealthMonitor::update(const mavlink_message_t& message,
                           std::chrono::steady_clock::time_point now) {
    apply_health_message(message, state_, now);
}

SafetyStatus HealthMonitor::status(std::chrono::steady_clock::time_point now) const {
    SafetyStatus result;
    result.reasons = evaluate_health_breach(state_, limits_, now);
    result.healthy = result.reasons.empty();
    return result;
}

}  // namespace safety
