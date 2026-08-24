#include <chrono>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>
#include <unistd.h>

#include "autopilot/autopilot_mavlink_adapter.hpp"
#include "app/runtime_config.hpp"
#include "drone_lib.hpp"

namespace {

constexpr uint16_t ARM_COMMAND = MAV_CMD_COMPONENT_ARM_DISARM;

std::string result_name(uint8_t result) {
    switch (result) {
        case MAV_RESULT_ACCEPTED: return "MAV_RESULT_ACCEPTED";
        case MAV_RESULT_TEMPORARILY_REJECTED: return "MAV_RESULT_TEMPORARILY_REJECTED";
        case MAV_RESULT_DENIED: return "MAV_RESULT_DENIED";
        case MAV_RESULT_UNSUPPORTED: return "MAV_RESULT_UNSUPPORTED";
        case MAV_RESULT_FAILED: return "MAV_RESULT_FAILED";
        case MAV_RESULT_IN_PROGRESS: return "MAV_RESULT_IN_PROGRESS";
        case MAV_RESULT_CANCELLED: return "MAV_RESULT_CANCELLED";
        default: return std::to_string(result);
    }
}

std::string mode_string(uint32_t custom_mode) {
    for (const auto& entry : copter_mode_mapping()) {
        if (entry.second == custom_mode) return entry.first;
    }
    return "Mode(" + std::to_string(custom_mode) + ")";
}

std::string statustext_to_string(const mavlink_statustext_t& st) {
    return std::string(st.text, strnlen(st.text, sizeof(st.text)));
}

std::unique_ptr<autopilot::AutopilotMavlinkAdapter> connect(
    const app::RuntimeConfig& config, double timeout,
    safety::SafetyMonitor& safety_monitor) {
    auto master = std::make_unique<autopilot::AutopilotMavlinkAdapter>(
        open_transport(config.endpoint, config.baud), config,
        [&](const safety::CommandRequest& request) {
            return safety_monitor.authorize(request);
        });
    std::cout << "MAVLink heartbeat 대기 중: " << config.endpoint << " (" << timeout << "초 제한)"
              << std::endl;

    mavlink_heartbeat_t heartbeat{};
    if (!master->wait_heartbeat(timeout, &heartbeat)) {
        throw std::runtime_error("heartbeat를 받지 못했습니다. 연결 주소/포트를 확인하세요.");
    }

    std::cout << "연결됨: system=" << static_cast<int>(master->target_system())
              << ", component=" << static_cast<int>(master->target_component())
              << ", mode=" << mode_string(heartbeat.custom_mode)
              << ", armed=" << (is_armed_from_heartbeat(heartbeat) ? "true" : "false")
              << std::endl;
    return master;
}

bool set_mode(autopilot::AutopilotMavlinkAdapter& master,
              const std::string& mode, double timeout) {
    const auto& mapping = copter_mode_mapping();
    auto it = mapping.find(mode);
    if (it == mapping.end()) {
        std::cout << "모드 변경 건너뜀: 지원하지 않는 모드 " << mode << std::endl;
        return false;
    }

    std::cout << mode << " 모드 변경 명령 전송" << std::endl;
    if (!master.set_mode(mode)) return false;

    auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout);
    while (std::chrono::steady_clock::now() < deadline) {
        mavlink_message_t msg;
        if (!master.recv_match({MAVLINK_MSG_ID_HEARTBEAT, MAVLINK_MSG_ID_STATUSTEXT}, msg, 1.0)) {
            continue;
        }
        if (msg.msgid == MAVLINK_MSG_ID_STATUSTEXT) {
            mavlink_statustext_t st;
            mavlink_msg_statustext_decode(&msg, &st);
            std::cout << "STATUSTEXT: " << statustext_to_string(st) << std::endl;
            continue;
        }

        mavlink_heartbeat_t hb;
        mavlink_msg_heartbeat_decode(&msg, &hb);
        std::string current_mode = mode_string(hb.custom_mode);
        bool armed = is_armed_from_heartbeat(hb);
        std::cout << "현재 상태: mode=" << current_mode
                  << ", armed=" << (armed ? "true" : "false") << std::endl;
        if (current_mode == mode) {
            std::cout << mode << " 모드 확인됨" << std::endl;
            return true;
        }
    }

    std::cout << mode << " 모드 확인 실패" << std::endl;
    return false;
}

void send_arm(autopilot::AutopilotMavlinkAdapter& master, bool should_arm) {
    std::cout << (should_arm ? "ARM" : "DISARM") << " 명령 전송" << std::endl;
    master.arm_disarm(should_arm);
}

struct ArmResult {
    bool ok;
    bool ack_seen;
    bool last_armed_known;
    bool last_armed;
};

ArmResult wait_arm_result(autopilot::AutopilotMavlinkAdapter& master,
                          bool expected_armed, double timeout) {
    auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout);
    bool ack_seen = false;
    bool last_armed_known = false;
    bool last_armed = false;

    while (std::chrono::steady_clock::now() < deadline) {
        mavlink_message_t msg;
        bool got = master.recv_match(
            {MAVLINK_MSG_ID_COMMAND_ACK, MAVLINK_MSG_ID_HEARTBEAT, MAVLINK_MSG_ID_STATUSTEXT}, msg,
            1.0);
        if (!got) continue;

        if (msg.msgid == MAVLINK_MSG_ID_STATUSTEXT) {
            mavlink_statustext_t st;
            mavlink_msg_statustext_decode(&msg, &st);
            std::cout << "STATUSTEXT: " << statustext_to_string(st) << std::endl;
            continue;
        }

        if (msg.msgid == MAVLINK_MSG_ID_COMMAND_ACK) {
            mavlink_command_ack_t ack;
            mavlink_msg_command_ack_decode(&msg, &ack);
            if (ack.command == ARM_COMMAND) {
                ack_seen = true;
                std::cout << "ARM ACK: " << result_name(ack.result) << " ("
                          << static_cast<int>(ack.result) << ")" << std::endl;
            }
            continue;
        }

        if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
            mavlink_heartbeat_t hb;
            mavlink_msg_heartbeat_decode(&msg, &hb);
            last_armed = is_armed_from_heartbeat(hb);
            last_armed_known = true;
            std::cout << "현재 상태: mode=" << mode_string(hb.custom_mode)
                      << ", armed=" << (last_armed ? "true" : "false") << std::endl;
            if (last_armed == expected_armed) {
                return {true, ack_seen, last_armed_known, last_armed};
            }
        }
    }

    return {false, ack_seen, last_armed_known, last_armed};
}

}  // namespace

int run(int argc, char** argv) {
    YamlValue settings = drone::load_mavlink_settings();

    std::string address = settings["sitl"]["local_tcp"]["address"].as_string();
    double heartbeat_timeout = 20;
    std::string mode = "GUIDED";
    bool no_mode = false;
    double arm_timeout = 15;
    double hold = 3;
    bool keep_armed = false;
    std::string target;
    std::string serial_endpoint;
    std::string serial_owner;
    bool allow_arm_flag = false;
    bool confirm_real_flight = false;
    bool commands_enabled_flag = false;
    bool serial_owner_confirmed = false;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        auto next = [&](const char* name) -> std::string {
            if (i + 1 >= argc) throw std::runtime_error(std::string(name) + " requires a value");
            return argv[++i];
        };
        if (arg == "--address") {
            address = next("--address");
        } else if (arg == "--target") {
            target = next("--target");
        } else if (arg == "--connect") {
            serial_endpoint = next("--connect");
        } else if (arg == "--serial-owner") {
            serial_owner = next("--serial-owner");
        } else if (arg == "--allow-arm") {
            allow_arm_flag = true;
        } else if (arg == "--confirm-real-flight") {
            confirm_real_flight = true;
        } else if (arg == "--commands-enabled") {
            commands_enabled_flag = true;
        } else if (arg == "--serial-owner-confirmed") {
            serial_owner_confirmed = true;
        } else if (arg == "--heartbeat-timeout") {
            heartbeat_timeout = std::stod(next("--heartbeat-timeout"));
        } else if (arg == "--mode") {
            mode = next("--mode");
        } else if (arg == "--no-mode") {
            no_mode = true;
        } else if (arg == "--arm-timeout") {
            arm_timeout = std::stod(next("--arm-timeout"));
        } else if (arg == "--hold") {
            hold = std::stod(next("--hold"));
        } else if (arg == "--keep-armed") {
            keep_armed = true;
        }
    }

    if (!target.empty()) setenv("ASTRODRONE_TARGET", target.c_str(), 1);
    const auto runtime_target = app::runtime_target_from_environment();
    if (runtime_target == app::RuntimeTarget::Real) {
        if (serial_owner != "onboard" || serial_endpoint.empty()) {
            throw std::runtime_error(
                "test_arm real guard: --connect /dev/serial/by-id/... and --serial-owner onboard are required");
        }
        setenv("ASTRODRONE_SERIAL_ENDPOINT", serial_endpoint.c_str(), 1);
        if (allow_arm_flag && confirm_real_flight && commands_enabled_flag &&
            serial_owner_confirmed) {
            setenv("ASTRODRONE_COMMAND_MODE", "flight", 1);
            setenv("ASTRODRONE_ALLOW_MAVLINK_WRITES", "1", 1);
            setenv("ASTRODRONE_ALLOW_VEHICLE_COMMANDS", "1", 1);
            setenv("ASTRODRONE_ALLOW_ARM", "1", 1);
            setenv("ASTRODRONE_COMMANDS_ENABLED", "1", 1);
            setenv("ASTRODRONE_SERIAL_OWNER_CONFIRMED", "1", 1);
            setenv("ASTRODRONE_ALLOW_TELEMETRY_CONFIGURATION", "1", 1);
        }
        const auto flight_config = app::load_runtime_config(
            runtime_target, app::TransportRole::CommandOwner);
        app::RealFlightOptions options;
        options.serial_endpoint = serial_endpoint;
        options.telemetry_endpoint = app::load_runtime_config(
            runtime_target, app::TransportRole::TelemetrySubscriber).endpoint;
        options.allow_arm = allow_arm_flag;
        options.confirm_real_flight = confirm_real_flight;
        options.commands_enabled = commands_enabled_flag;
        options.serial_owner_confirmed = serial_owner_confirmed;
        if (access(serial_endpoint.c_str(), F_OK) != 0) {
            throw std::runtime_error(
                "test_arm real guard: serial path must exist and be supplied explicitly");
        }
        const auto decision = app::validate_real_flight(flight_config, options);
        if (!decision.allowed) throw std::runtime_error("test_arm real guard: " + decision.reason);
        address = flight_config.endpoint;
    } else {
        // SITL uses the configured UDP endpoint unless --address overrides it.
    }

    auto config = app::load_runtime_config(
        runtime_target, app::TransportRole::CommandOwner);
    if (runtime_target == app::RuntimeTarget::Sitl && !address.empty()) {
        config.endpoint = address;
    }
    safety::SafetyMonitor safety_monitor;
    safety::CommandAuthority command_authority;
    command_authority.vehicle_commands_enabled = config.allow_vehicle_commands;
    command_authority.real_flight_approved =
        runtime_target != app::RuntimeTarget::Real ||
        config.command_mode == app::CommandMode::Flight;
    safety_monitor.set_authority(command_authority);
    auto master = connect(config, heartbeat_timeout, safety_monitor);

    if (!no_mode) {
        set_mode(*master, mode, 8);
    }

    send_arm(*master, true);
    ArmResult armed_result = wait_arm_result(*master, true, arm_timeout);

    if (armed_result.ok) {
        std::cout << "ARM 성공: heartbeat에서 armed=True 확인" << std::endl;
    } else {
        std::cout << "ARM 실패 또는 확인 실패: ack_seen=" << (armed_result.ack_seen ? "true" : "false")
                  << ", last_armed="
                  << (armed_result.last_armed_known ? (armed_result.last_armed ? "true" : "false")
                                                     : "None")
                  << std::endl;
    }

    if (keep_armed) {
        std::cout << "keep-armed 옵션 때문에 DISARM은 보내지 않습니다." << std::endl;
        return 0;
    }

    if (hold > 0) {
        std::cout << hold << "초 대기 후 DISARM" << std::endl;
        std::this_thread::sleep_for(std::chrono::duration<double>(hold));
    }

    send_arm(*master, false);
    ArmResult disarmed_result = wait_arm_result(*master, false, arm_timeout);

    if (disarmed_result.ok) {
        std::cout << "DISARM 성공: heartbeat에서 armed=False 확인" << std::endl;
    } else {
        std::cout << "DISARM 실패 또는 확인 실패: ack_seen="
                  << (disarmed_result.ack_seen ? "true" : "false") << ", last_armed="
                  << (disarmed_result.last_armed_known
                          ? (disarmed_result.last_armed ? "true" : "false")
                          : "None")
                  << std::endl;
    }

    return 0;
}

int main(int argc, char** argv) {
    try {
        return run(argc, argv);
    } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return 1;
    }
}
