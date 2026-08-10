#include "fake_transport.hpp"

#include <algorithm>
#include <cstring>

void FakeTransport::write_bytes(const uint8_t* data, size_t len) {
    written_bytes_.insert(written_bytes_.end(), data, data + len);
    ++write_call_count_;
}

size_t FakeTransport::read_bytes(uint8_t* buffer, size_t maxlen, int timeout_ms) {
    (void)timeout_ms;
    ++read_call_count_;
    if (incoming_chunks_.empty() || maxlen == 0) return 0;

    const auto& chunk = incoming_chunks_.front();
    size_t available = chunk.size() - incoming_offset_;
    size_t count = std::min(maxlen, available);
    std::memcpy(buffer, chunk.data() + incoming_offset_, count);
    incoming_offset_ += count;
    if (incoming_offset_ == chunk.size()) {
        incoming_chunks_.pop_front();
        incoming_offset_ = 0;
    }
    return count;
}

void FakeTransport::queue_incoming_bytes(const std::vector<uint8_t>& bytes) {
    incoming_chunks_.push_back(bytes);
}

void FakeTransport::queue_incoming_message(const mavlink_message_t& message) {
    std::vector<uint8_t> bytes(MAVLINK_MAX_PACKET_LEN);
    uint16_t length = mavlink_msg_to_send_buffer(bytes.data(), &message);
    bytes.resize(length);
    queue_incoming_bytes(bytes);
}

std::vector<mavlink_message_t> FakeTransport::decode_written_messages() const {
    mavlink_reset_channel_status(MAVLINK_COMM_1);
    std::vector<mavlink_message_t> messages;
    for (uint8_t byte : written_bytes_) {
        mavlink_message_t message{};
        mavlink_status_t status{};
        if (mavlink_parse_char(MAVLINK_COMM_1, byte, &message, &status)) {
            messages.push_back(message);
        }
    }
    return messages;
}

void FakeTransport::clear_written() {
    written_bytes_.clear();
    write_call_count_ = 0;
}
