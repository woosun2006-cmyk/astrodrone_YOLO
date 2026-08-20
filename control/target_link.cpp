#include "target_link.hpp"

#include <arpa/inet.h>
#include <cerrno>
#include <cstring>
#include <cstdio>
#include <fcntl.h>
#include <netinet/in.h>
#include <stdexcept>
#include <sys/socket.h>
#include <unistd.h>

namespace {

void throw_errno(const std::string& what) {
    throw std::runtime_error(what + ": " + std::strerror(errno));
}

}  // namespace

TargetRangeSender::TargetRangeSender(int port) {
    fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (fd_ < 0) throw_errno("create UDP socket (target range sender)");

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<uint16_t>(port));
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    // connect() on a UDP socket just fixes the destination for send(), so
    // datagrams to a not-yet-running (or crashed) receiver simply fail
    // locally instead of raising SIGPIPE or blocking.
    if (::connect(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
        throw_errno("connect UDP 127.0.0.1:" + std::to_string(port));
    }
}

TargetRangeSender::~TargetRangeSender() {
    if (fd_ >= 0) ::close(fd_);
}

void TargetRangeSender::send(const TargetRangeMsg& msg) {
    ::send(fd_, &msg, sizeof(msg), 0);
}

TargetRangeReceiver::TargetRangeReceiver(int port) {
    fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (fd_ < 0) throw_errno("create UDP socket (target range receiver)");

    int reuse = 1;
    setsockopt(fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<uint16_t>(port));
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    if (::bind(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
        throw_errno("bind UDP 127.0.0.1:" + std::to_string(port));
    }

    int flags = fcntl(fd_, F_GETFL, 0);
    fcntl(fd_, F_SETFL, flags | O_NONBLOCK);
}

TargetRangeReceiver::~TargetRangeReceiver() {
    if (fd_ >= 0) ::close(fd_);
}

bool TargetRangeReceiver::poll(TargetRangeMsg& out) {
    bool got_any = false;
    TargetRangeMsg tmp;
    while (true) {
        ssize_t n = ::recv(fd_, &tmp, sizeof(tmp), 0);
        if (n <= 0) break;  // EAGAIN/EWOULDBLOCK (no more pending) or error
        if (static_cast<size_t>(n) == sizeof(tmp)) {
            out = tmp;
            got_any = true;
        }
    }
    return got_any;
}

GcsCommandStateSender::GcsCommandStateSender(int port) {
    fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (fd_ < 0) return;
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_port = htons(static_cast<uint16_t>(port));
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    if (::connect(fd_, reinterpret_cast<sockaddr*>(&address), sizeof(address)) != 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

GcsCommandStateSender::~GcsCommandStateSender() {
    if (fd_ >= 0) ::close(fd_);
}

void GcsCommandStateSender::send(float vx, float vy, float vz, float path_angle_rad,
                                  bool allowed, const char* state,
                                  const char* safety_reason) {
    if (fd_ < 0) return;
    GcsCommandStateMsg message{};
    message.seq = seq_++;
    message.vx = vx;
    message.vy = vy;
    message.vz = vz;
    message.path_angle_rad = path_angle_rad;
    message.allowed = allowed ? 1 : 0;
    std::snprintf(message.state, sizeof(message.state), "%s", state ? state : "");
    std::snprintf(message.safety_reason, sizeof(message.safety_reason), "%s",
                  safety_reason ? safety_reason : "");
    ::send(fd_, &message, sizeof(message), 0);
}

GcsCommandStateReceiver::GcsCommandStateReceiver(int port) {
    fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (fd_ < 0) throw_errno("create UDP socket (GCS command state receiver)");
    int reuse = 1;
    setsockopt(fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_port = htons(static_cast<uint16_t>(port));
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    if (::bind(fd_, reinterpret_cast<sockaddr*>(&address), sizeof(address)) != 0) {
        throw_errno("bind UDP 127.0.0.1:" + std::to_string(port));
    }
    const int flags = fcntl(fd_, F_GETFL, 0);
    fcntl(fd_, F_SETFL, flags | O_NONBLOCK);
}

GcsCommandStateReceiver::~GcsCommandStateReceiver() {
    if (fd_ >= 0) ::close(fd_);
}

bool GcsCommandStateReceiver::poll(GcsCommandStateMsg& out) {
    bool received = false;
    GcsCommandStateMsg message{};
    while (recv(fd_, &message, sizeof(message), 0) == static_cast<ssize_t>(sizeof(message))) {
        out = message;
        received = true;
    }
    return received;
}
