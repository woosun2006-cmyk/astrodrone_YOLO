#include "fake_transport.hpp"

#include "drone_lib.hpp"

#include <chrono>
#include <cmath>
#include <functional>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <vector>

namespace {

bool expect(bool condition, const char* expression, int line) {
    if (condition) return true;
    std::cerr << "line " << line << ": check failed: " << expression << std::endl;
    return false;
}

#define CHECK(expression) \
    do { \
        if (!expect(static_cast<bool>(expression), #expression, __LINE__)) return false; \
    } while (false)

mavlink_message_t heartbeat(uint8_t system_id, uint8_t component_id, uint32_t custom_mode = 0,
                            uint8_t type = MAV_TYPE_QUADROTOR,
                            uint8_t autopilot = MAV_AUTOPILOT_ARDUPILOTMEGA) {
    mavlink_message_t message{};
    mavlink_msg_heartbeat_pack(system_id, component_id, &message, type, autopilot, 0, custom_mode,
                               MAV_STATE_ACTIVE);
    return message;
}

mavlink_message_t sys_status(uint8_t system_id, uint8_t component_id, uint16_t voltage_mv) {
    mavlink_message_t message{};
    mavlink_msg_sys_status_pack(system_id, component_id, &message, 0, 0, 0, 0, voltage_mv, -1, 50,
                                0, 0, 0, 0, 0, 0, 0, 0, 0);
    return message;
}

std::vector<uint8_t> encode_message(const mavlink_message_t& message) {
    std::vector<uint8_t> bytes(MAVLINK_MAX_PACKET_LEN);
    mavlink_message_t copy = message;
    const uint16_t length = mavlink_msg_to_send_buffer(bytes.data(), &copy);
    bytes.resize(length);
    return bytes;
}

bool receives_heartbeat_and_records_target() {
    mavlink_reset_channel_status(MAVLINK_COMM_0);
    auto transport = std::make_unique<FakeTransport>();
    transport->queue_incoming_message(heartbeat(1, MAV_COMP_ID_AUTOPILOT1, 3));
    MavConnection connection(std::move(transport));

    mavlink_heartbeat_t decoded{};
    CHECK(connection.wait_heartbeat(0.1, &decoded));
    CHECK(connection.target_system() == 1);
    CHECK(connection.target_component() == MAV_COMP_ID_AUTOPILOT1);
    CHECK(decoded.custom_mode == 3);
    return true;
}

bool timeout_returns_without_device_or_sleep() {
    mavlink_reset_channel_status(MAVLINK_COMM_0);
    auto transport = std::make_unique<FakeTransport>();
    FakeTransport* fake = transport.get();
    MavConnection connection(std::move(transport));
    mavlink_message_t message{};

    auto started = std::chrono::steady_clock::now();
    bool received = connection.recv_match({MAVLINK_MSG_ID_HEARTBEAT}, message, 0.001);
    auto elapsed = std::chrono::steady_clock::now() - started;

    CHECK(!received);
    CHECK(fake->read_call_count() > 0);
    CHECK(elapsed < std::chrono::milliseconds(100));
    return true;
}

bool send_before_target_discovery_is_rejected() {
    mavlink_reset_channel_status(MAVLINK_COMM_0);
    auto transport = std::make_unique<FakeTransport>();
    FakeTransport* fake = transport.get();
    MavConnection connection(std::move(transport));

    mavlink_message_t outgoing{};
    mavlink_msg_set_mode_pack(255, 0, &outgoing, 1, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 4);

    bool rejected = false;
    try {
        connection.send(outgoing);
    } catch (const std::runtime_error&) {
        rejected = true;
    }
    CHECK(rejected);
    CHECK(fake->write_call_count() == 0);
    CHECK(fake->written_bytes().empty());
    return true;
}

bool non_autopilot_heartbeat_does_not_set_target() {
    mavlink_reset_channel_status(MAVLINK_COMM_0);
    auto transport = std::make_unique<FakeTransport>();
    transport->queue_incoming_message(
        heartbeat(7, MAV_COMP_ID_AUTOPILOT1, 0, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_INVALID));
    MavConnection connection(std::move(transport));

    mavlink_message_t received{};
    CHECK(connection.recv_match({MAVLINK_MSG_ID_HEARTBEAT}, received, 0.1));
    CHECK(connection.target_system() == 0);
    CHECK(connection.target_component() == 0);
    return true;
}

bool wait_heartbeat_skips_non_autopilot_heartbeat() {
    mavlink_reset_channel_status(MAVLINK_COMM_0);
    auto transport = std::make_unique<FakeTransport>();
    transport->queue_incoming_message(
        heartbeat(7, MAV_COMP_ID_AUTOPILOT1, 0, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_INVALID));
    transport->queue_incoming_message(heartbeat(2, MAV_COMP_ID_AUTOPILOT1, 6));
    MavConnection connection(std::move(transport));

    mavlink_heartbeat_t decoded{};
    CHECK(connection.wait_heartbeat(0.1, &decoded));
    CHECK(connection.target_system() == 2);
    CHECK(connection.target_component() == MAV_COMP_ID_AUTOPILOT1);
    CHECK(decoded.custom_mode == 6);
    return true;
}

bool later_autopilot_heartbeat_does_not_switch_target() {
    mavlink_reset_channel_status(MAVLINK_COMM_0);
    auto transport = std::make_unique<FakeTransport>();
    transport->queue_incoming_message(heartbeat(1, MAV_COMP_ID_AUTOPILOT1));
    transport->queue_incoming_message(heartbeat(9, MAV_COMP_ID_AUTOPILOT1));
    MavConnection connection(std::move(transport));

    CHECK(connection.wait_heartbeat(0.1));
    mavlink_message_t received{};
    CHECK(connection.recv_match({MAVLINK_MSG_ID_HEARTBEAT}, received, 0.1));
    CHECK(received.sysid == 9);
    CHECK(connection.target_system() == 1);
    CHECK(connection.target_component() == MAV_COMP_ID_AUTOPILOT1);
    return true;
}

bool send_records_decodable_packet() {
    mavlink_reset_channel_status(MAVLINK_COMM_0);
    auto transport = std::make_unique<FakeTransport>();
    FakeTransport* fake = transport.get();
    transport->queue_incoming_message(heartbeat(1, MAV_COMP_ID_AUTOPILOT1));
    MavConnection connection(std::move(transport));
    CHECK(connection.wait_heartbeat(0.1));

    mavlink_message_t outgoing{};
    mavlink_msg_set_mode_pack(255, 0, &outgoing, 1, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 4);
    connection.send(outgoing);

    CHECK(fake->write_call_count() == 1);
    CHECK(!fake->written_bytes().empty());
    auto decoded_messages = fake->decode_written_messages();
    CHECK(decoded_messages.size() == 1);
    CHECK(decoded_messages[0].msgid == MAVLINK_MSG_ID_SET_MODE);

    mavlink_set_mode_t set_mode{};
    mavlink_msg_set_mode_decode(&decoded_messages[0], &set_mode);
    CHECK(set_mode.target_system == 1);
    CHECK(set_mode.base_mode == MAV_MODE_FLAG_CUSTOM_MODE_ENABLED);
    CHECK(set_mode.custom_mode == 4);

    fake->clear_written();
    CHECK(fake->written_bytes().empty());
    CHECK(fake->write_call_count() == 0);
    return true;
}

bool receives_multiple_messages_in_order() {
    mavlink_reset_channel_status(MAVLINK_COMM_0);
    auto transport = std::make_unique<FakeTransport>();
    transport->queue_incoming_message(heartbeat(1, 1));
    transport->queue_incoming_message(sys_status(1, 1, 15200));
    MavConnection connection(std::move(transport));

    mavlink_message_t first{};
    mavlink_message_t second{};
    CHECK(connection.recv_match({}, first, 0.1));
    CHECK(connection.recv_match({}, second, 0.1));
    CHECK(first.msgid == MAVLINK_MSG_ID_HEARTBEAT);
    CHECK(second.msgid == MAVLINK_MSG_ID_SYS_STATUS);

    mavlink_sys_status_t status{};
    mavlink_msg_sys_status_decode(&second, &status);
    CHECK(status.voltage_battery == 15200);
    return true;
}

bool receives_coalesced_messages_without_byte_loss() {
    mavlink_reset_channel_status(MAVLINK_COMM_0);
    auto transport = std::make_unique<FakeTransport>();
    std::vector<uint8_t> combined = encode_message(heartbeat(1, 1));
    const std::vector<uint8_t> status_bytes = encode_message(sys_status(1, 1, 15200));
    combined.insert(combined.end(), status_bytes.begin(), status_bytes.end());
    transport->queue_incoming_bytes(combined);
    MavConnection connection(std::move(transport));

    mavlink_message_t first{};
    mavlink_message_t second{};
    CHECK(connection.recv_match({}, first, 0.1));
    CHECK(first.msgid == MAVLINK_MSG_ID_HEARTBEAT);
    CHECK(connection.recv_match({}, second, 0.01));
    CHECK(second.msgid == MAVLINK_MSG_ID_SYS_STATUS);
    return true;
}

bool filter_discards_earlier_mismatch_and_preserves_later_message() {
    mavlink_reset_channel_status(MAVLINK_COMM_0);
    auto transport = std::make_unique<FakeTransport>();
    std::vector<uint8_t> combined = encode_message(sys_status(1, 1, 15000));
    const std::vector<uint8_t> heartbeat_bytes = encode_message(heartbeat(1, 1));
    const std::vector<uint8_t> later_status_bytes = encode_message(sys_status(1, 1, 16000));
    combined.insert(combined.end(), heartbeat_bytes.begin(), heartbeat_bytes.end());
    combined.insert(combined.end(), later_status_bytes.begin(), later_status_bytes.end());
    transport->queue_incoming_bytes(combined);
    MavConnection connection(std::move(transport));

    mavlink_message_t received{};
    CHECK(connection.recv_match({MAVLINK_MSG_ID_HEARTBEAT}, received, 0.1));
    CHECK(received.msgid == MAVLINK_MSG_ID_HEARTBEAT);
    CHECK(connection.recv_match({MAVLINK_MSG_ID_SYS_STATUS}, received, 0.01));

    mavlink_sys_status_t status{};
    mavlink_msg_sys_status_decode(&received, &status);
    CHECK(status.voltage_battery == 16000);
    return true;
}

bool reassembles_message_split_across_reads() {
    mavlink_reset_channel_status(MAVLINK_COMM_0);
    auto transport = std::make_unique<FakeTransport>();
    const std::vector<uint8_t> bytes = encode_message(heartbeat(3, MAV_COMP_ID_AUTOPILOT1, 7));
    const std::size_t first_split = bytes.size() / 3;
    const std::size_t second_split = (bytes.size() * 2) / 3;
    transport->queue_incoming_bytes(
        std::vector<uint8_t>(bytes.begin(), bytes.begin() + first_split));
    transport->queue_incoming_bytes(
        std::vector<uint8_t>(bytes.begin() + first_split, bytes.begin() + second_split));
    transport->queue_incoming_bytes(
        std::vector<uint8_t>(bytes.begin() + second_split, bytes.end()));
    MavConnection connection(std::move(transport));

    mavlink_heartbeat_t decoded{};
    CHECK(connection.wait_heartbeat(0.1, &decoded));
    CHECK(connection.target_system() == 3);
    CHECK(connection.target_component() == MAV_COMP_ID_AUTOPILOT1);
    CHECK(decoded.custom_mode == 7);
    return true;
}

bool empty_read_does_not_hide_following_message() {
    mavlink_reset_channel_status(MAVLINK_COMM_0);
    auto transport = std::make_unique<FakeTransport>();
    FakeTransport* fake = transport.get();
    transport->queue_incoming_bytes({});
    transport->queue_incoming_message(heartbeat(5, MAV_COMP_ID_AUTOPILOT1, 8));
    MavConnection connection(std::move(transport));

    mavlink_heartbeat_t decoded{};
    CHECK(connection.wait_heartbeat(0.1, &decoded));
    CHECK(fake->read_call_count() >= 2);
    CHECK(connection.target_system() == 5);
    CHECK(connection.target_component() == MAV_COMP_ID_AUTOPILOT1);
    return true;
}

bool command_target_is_stable_after_other_component_message() {
    mavlink_reset_channel_status(MAVLINK_COMM_0);
    auto transport = std::make_unique<FakeTransport>();
    transport->queue_incoming_message(heartbeat(1, 1));
    transport->queue_incoming_message(sys_status(1, 42, 15200));
    MavConnection connection(std::move(transport));

    CHECK(connection.wait_heartbeat(0.1));
    CHECK(connection.target_system() == 1);
    CHECK(connection.target_component() == 1);

    mavlink_message_t received{};
    CHECK(connection.recv_match({MAVLINK_MSG_ID_SYS_STATUS}, received, 0.1));
    CHECK(connection.target_system() == 1);
    CHECK(connection.target_component() == 1);
    return true;
}

bool command_target_is_stable_after_other_system_message() {
    mavlink_reset_channel_status(MAVLINK_COMM_0);
    auto transport = std::make_unique<FakeTransport>();
    transport->queue_incoming_message(heartbeat(1, 1));
    transport->queue_incoming_message(sys_status(9, 1, 15200));
    MavConnection connection(std::move(transport));

    CHECK(connection.wait_heartbeat(0.1));
    CHECK(connection.target_system() == 1);
    CHECK(connection.target_component() == 1);

    mavlink_message_t received{};
    CHECK(connection.recv_match({MAVLINK_MSG_ID_SYS_STATUS}, received, 0.1));
    CHECK(connection.target_system() == 1);
    CHECK(connection.target_component() == 1);
    return true;
}

bool malformed_and_incomplete_input_does_not_abort() {
    mavlink_reset_channel_status(MAVLINK_COMM_0);
    auto garbage_transport = std::make_unique<FakeTransport>();
    garbage_transport->queue_incoming_bytes({0x00, 0x7f, 0x55, 0xaa});
    garbage_transport->queue_incoming_message(heartbeat(7, MAV_COMP_ID_AUTOPILOT1, 4));
    MavConnection garbage_connection(std::move(garbage_transport));

    mavlink_message_t received{};
    CHECK(garbage_connection.recv_match({MAVLINK_MSG_ID_HEARTBEAT}, received, 0.1));
    CHECK(received.msgid == MAVLINK_MSG_ID_HEARTBEAT);
    CHECK(garbage_connection.target_system() == 7);
    CHECK(garbage_connection.target_component() == MAV_COMP_ID_AUTOPILOT1);

    mavlink_reset_channel_status(MAVLINK_COMM_0);
    mavlink_message_t complete = heartbeat(9, 10, 3);
    std::vector<uint8_t> bytes(MAVLINK_MAX_PACKET_LEN);
    uint16_t length = mavlink_msg_to_send_buffer(bytes.data(), &complete);
    bytes.resize(length / 2);

    auto incomplete_transport = std::make_unique<FakeTransport>();
    incomplete_transport->queue_incoming_bytes(bytes);
    MavConnection incomplete_connection(std::move(incomplete_transport));
    CHECK(!incomplete_connection.recv_match({MAVLINK_MSG_ID_HEARTBEAT}, received, 0.01));
    return true;
}

}  // namespace

int main() {
    const std::vector<std::pair<const char*, std::function<bool()>>> tests = {
        {"receives_heartbeat_and_records_target", receives_heartbeat_and_records_target},
        {"timeout_returns_without_device_or_sleep", timeout_returns_without_device_or_sleep},
        {"send_records_decodable_packet", send_records_decodable_packet},
        {"send_before_target_discovery_is_rejected", send_before_target_discovery_is_rejected},
        {"non_autopilot_heartbeat_does_not_set_target",
         non_autopilot_heartbeat_does_not_set_target},
        {"wait_heartbeat_skips_non_autopilot_heartbeat",
         wait_heartbeat_skips_non_autopilot_heartbeat},
        {"later_autopilot_heartbeat_does_not_switch_target",
         later_autopilot_heartbeat_does_not_switch_target},
        {"receives_multiple_messages_in_order", receives_multiple_messages_in_order},
        {"receives_coalesced_messages_without_byte_loss",
         receives_coalesced_messages_without_byte_loss},
        {"filter_discards_earlier_mismatch_and_preserves_later_message",
         filter_discards_earlier_mismatch_and_preserves_later_message},
        {"reassembles_message_split_across_reads", reassembles_message_split_across_reads},
        {"empty_read_does_not_hide_following_message", empty_read_does_not_hide_following_message},
        {"command_target_is_stable_after_other_component_message",
         command_target_is_stable_after_other_component_message},
        {"command_target_is_stable_after_other_system_message",
         command_target_is_stable_after_other_system_message},
        {"malformed_and_incomplete_input_does_not_abort", malformed_and_incomplete_input_does_not_abort},
    };

    int failed = 0;
    for (const auto& test : tests) {
        if (test.second()) {
            std::cout << "PASS " << test.first << std::endl;
        } else {
            std::cerr << "FAIL " << test.first << std::endl;
            ++failed;
        }
    }
    return failed == 0 ? 0 : 1;
}
