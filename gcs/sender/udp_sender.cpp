#include "udp_sender.hpp"

#include <netdb.h>
#include <unistd.h>

#include <cstring>
#include <stdexcept>

namespace gcs {

UdpSender::UdpSender(const std::string& dest_host, uint16_t dest_port) {
    addrinfo hints{};
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_DGRAM;

    addrinfo* res = nullptr;
    const std::string port_str = std::to_string(dest_port);
    const int rc = getaddrinfo(dest_host.c_str(), port_str.c_str(), &hints, &res);
    if (rc != 0 || res == nullptr) {
        throw std::runtime_error("UdpSender: getaddrinfo failed for " + dest_host + ": " +
                                  gai_strerror(rc));
    }

    fd_ = socket(res->ai_family, res->ai_socktype, res->ai_protocol);
    if (fd_ < 0) {
        freeaddrinfo(res);
        throw std::runtime_error("UdpSender: socket() failed: " + std::string(strerror(errno)));
    }

    // connect() on a UDP socket just fixes the peer address so send() can be
    // used instead of sendto() -- no handshake, still connectionless.
    if (connect(fd_, res->ai_addr, res->ai_addrlen) != 0) {
        freeaddrinfo(res);
        close(fd_);
        fd_ = -1;
        throw std::runtime_error("UdpSender: connect() failed: " + std::string(strerror(errno)));
    }
    freeaddrinfo(res);
}

UdpSender::~UdpSender() {
    if (fd_ >= 0) {
        close(fd_);
    }
}

void UdpSender::send(const uint8_t* data, size_t len) {
    if (fd_ < 0) {
        return;
    }
    ::send(fd_, data, len, 0);
}

}  // namespace gcs
