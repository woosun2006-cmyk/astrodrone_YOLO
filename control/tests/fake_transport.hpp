#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <vector>

#include "mav_transport.hpp"
#include "third_party/mavlink/common/mavlink.h"

// Test-only in-memory Transport. It never opens a socket or serial device,
// never sleeps, and returns 0 immediately when no incoming bytes are queued.
class FakeTransport final : public Transport {
public:
    void write_bytes(const uint8_t* data, size_t len) override;
    size_t read_bytes(uint8_t* buffer, size_t maxlen, int timeout_ms) override;

    void queue_incoming_bytes(const std::vector<uint8_t>& bytes);
    void queue_incoming_message(const mavlink_message_t& message);

    const std::vector<uint8_t>& written_bytes() const { return written_bytes_; }
    std::vector<mavlink_message_t> decode_written_messages() const;
    void clear_written();
    size_t write_call_count() const { return write_call_count_; }
    size_t read_call_count() const { return read_call_count_; }

private:
    std::deque<std::vector<uint8_t>> incoming_chunks_;
    size_t incoming_offset_ = 0;
    std::vector<uint8_t> written_bytes_;
    size_t write_call_count_ = 0;
    size_t read_call_count_ = 0;
};
