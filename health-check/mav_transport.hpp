#pragma once

#include <cstdint>
#include <cstddef>
#include <memory>
#include <string>

// Byte-level transport used to carry a MAVLink stream. Mirrors the subset of
// address formats pymavlink's mavutil.mavlink_connection() accepts that this
// project actually uses: "tcp:host:port" (client), "udp:host:port" (server --
// binds and replies to whichever peer last sent a datagram, matching
// pymavlink's legacy "udp:" alias), and a bare path for a serial device.
class Transport {
public:
    virtual ~Transport() = default;
    virtual void write_bytes(const uint8_t* data, size_t len) = 0;
    // Blocks up to timeout_ms for at least one byte. Returns the number of
    // bytes read into buffer (0 on timeout).
    virtual size_t read_bytes(uint8_t* buffer, size_t maxlen, int timeout_ms) = 0;
};

std::unique_ptr<Transport> open_transport(const std::string& address, int baud);
