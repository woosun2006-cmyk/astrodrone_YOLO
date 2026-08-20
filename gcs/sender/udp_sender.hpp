#pragma once

// Fire-and-forget UDP sender. No ack, no retry -- loss recovery is FecEncoder's
// job, not this class's; this is deliberately a thin sendto() wrapper.

#include <cstddef>
#include <cstdint>
#include <string>

namespace gcs {

class UdpSender {
public:
    UdpSender(const std::string& dest_host, uint16_t dest_port);
    ~UdpSender();

    UdpSender(const UdpSender&) = delete;
    UdpSender& operator=(const UdpSender&) = delete;

    void send(const uint8_t* data, size_t len);

private:
    int fd_ = -1;
};

}  // namespace gcs
