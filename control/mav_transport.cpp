#include "mav_transport.hpp"

#include <arpa/inet.h>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <netinet/in.h>
#include <poll.h>
#include <stdexcept>
#include <sys/socket.h>
#include <sys/types.h>
#include <termios.h>
#include <tuple>
#include <unistd.h>

namespace {

struct ParsedAddress {
    enum class Kind { Serial, Tcp, Udp } kind;
    std::string host;
    int port = 0;
    std::string path;
};

ParsedAddress parse_address(const std::string& address) {
    ParsedAddress result;
    auto split_host_port = [](const std::string& s) {
        size_t sep = s.find_last_of(':');
        if (sep == std::string::npos) {
            throw std::runtime_error("expected host:port in address: " + s);
        }
        return std::make_pair(s.substr(0, sep), std::stoi(s.substr(sep + 1)));
    };

    if (address.rfind("tcp:", 0) == 0) {
        result.kind = ParsedAddress::Kind::Tcp;
        std::tie(result.host, result.port) = split_host_port(address.substr(4));
    } else if (address.rfind("udp:", 0) == 0) {
        result.kind = ParsedAddress::Kind::Udp;
        std::tie(result.host, result.port) = split_host_port(address.substr(4));
    } else {
        result.kind = ParsedAddress::Kind::Serial;
        result.path = address;
    }
    return result;
}

speed_t baud_to_speed(int baud) {
    switch (baud) {
        case 9600: return B9600;
        case 19200: return B19200;
        case 38400: return B38400;
        case 57600: return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
        case 460800: return B460800;
        case 921600: return B921600;
        default:
            throw std::runtime_error("unsupported baud rate: " + std::to_string(baud));
    }
}

void throw_errno(const std::string& what) {
    throw std::runtime_error(what + ": " + std::strerror(errno));
}

int wait_readable(int fd, int timeout_ms) {
    struct pollfd pfd { fd, POLLIN, 0 };
    return poll(&pfd, 1, timeout_ms);
}

class SerialTransport : public Transport {
public:
    SerialTransport(const std::string& path, int baud) {
        fd_ = ::open(path.c_str(), O_RDWR | O_NOCTTY);
        if (fd_ < 0) throw_errno("open serial port " + path);

        struct termios tty {};
        if (tcgetattr(fd_, &tty) != 0) throw_errno("tcgetattr");

        speed_t speed = baud_to_speed(baud);
        cfsetispeed(&tty, speed);
        cfsetospeed(&tty, speed);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
        tty.c_cflag |= CLOCAL | CREAD;
        tty.c_cflag &= ~PARENB;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        tty.c_lflag = 0;  // raw
        tty.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL | INLCR);
        tty.c_oflag = 0;

        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 0;

        if (tcsetattr(fd_, TCSANOW, &tty) != 0) throw_errno("tcsetattr");
    }

    ~SerialTransport() override {
        if (fd_ >= 0) ::close(fd_);
    }

    void write_bytes(const uint8_t* data, size_t len) override {
        size_t written = 0;
        while (written < len) {
            ssize_t n = ::write(fd_, data + written, len - written);
            if (n < 0) throw_errno("write serial port");
            written += static_cast<size_t>(n);
        }
    }

    size_t read_bytes(uint8_t* buffer, size_t maxlen, int timeout_ms) override {
        if (wait_readable(fd_, timeout_ms) <= 0) return 0;
        ssize_t n = ::read(fd_, buffer, maxlen);
        if (n < 0) throw_errno("read serial port");
        return static_cast<size_t>(n);
    }

private:
    int fd_ = -1;
};

class TcpTransport : public Transport {
public:
    TcpTransport(const std::string& host, int port) {
        fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
        if (fd_ < 0) throw_errno("create TCP socket");

        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(static_cast<uint16_t>(port));
        if (inet_pton(AF_INET, host.c_str(), &addr.sin_addr) != 1) {
            throw std::runtime_error("invalid TCP host: " + host);
        }
        if (::connect(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
            throw_errno("connect TCP " + host + ":" + std::to_string(port));
        }
    }

    ~TcpTransport() override {
        if (fd_ >= 0) ::close(fd_);
    }

    void write_bytes(const uint8_t* data, size_t len) override {
        size_t written = 0;
        while (written < len) {
            ssize_t n = ::send(fd_, data + written, len - written, 0);
            if (n < 0) throw_errno("send TCP");
            written += static_cast<size_t>(n);
        }
    }

    size_t read_bytes(uint8_t* buffer, size_t maxlen, int timeout_ms) override {
        if (wait_readable(fd_, timeout_ms) <= 0) return 0;
        ssize_t n = ::recv(fd_, buffer, maxlen, 0);
        if (n < 0) throw_errno("recv TCP");
        return static_cast<size_t>(n);
    }

private:
    int fd_ = -1;
};

// Binds to host:port and, once a datagram has been received, replies to
// whichever peer sent it -- this is what pymavlink's legacy "udp:" alias
// does, and is what lets Mission Planner's SITL UDP output reach us without
// the Jetson knowing the sender's ephemeral port up front.
class UdpTransport : public Transport {
public:
    UdpTransport(const std::string& host, int port) {
        fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
        if (fd_ < 0) throw_errno("create UDP socket");

        int reuse = 1;
        setsockopt(fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
        int receive_buffer = 1024 * 1024;
        setsockopt(fd_, SOL_SOCKET, SO_RCVBUF, &receive_buffer, sizeof(receive_buffer));

        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(static_cast<uint16_t>(port));
        if (inet_pton(AF_INET, host.c_str(), &addr.sin_addr) != 1) {
            throw std::runtime_error("invalid UDP host: " + host);
        }
        if (::bind(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
            throw_errno("bind UDP " + host + ":" + std::to_string(port));
        }
    }

    ~UdpTransport() override {
        if (fd_ >= 0) ::close(fd_);
    }

    void write_bytes(const uint8_t* data, size_t len) override {
        if (!have_peer_) return;  // nothing to reply to yet
        ssize_t n = ::sendto(fd_, data, len, 0,
                              reinterpret_cast<sockaddr*>(&peer_), sizeof(peer_));
        if (n < 0) throw_errno("sendto UDP");
    }

    size_t read_bytes(uint8_t* buffer, size_t maxlen, int timeout_ms) override {
        if (wait_readable(fd_, timeout_ms) <= 0) return 0;
        socklen_t peer_len = sizeof(peer_);
        ssize_t n = ::recvfrom(fd_, buffer, maxlen, 0,
                                reinterpret_cast<sockaddr*>(&peer_), &peer_len);
        if (n < 0) throw_errno("recvfrom UDP");
        have_peer_ = true;
        return static_cast<size_t>(n);
    }

private:
    int fd_ = -1;
    sockaddr_in peer_{};
    bool have_peer_ = false;
};

}  // namespace

std::unique_ptr<Transport> open_transport(const std::string& address, int baud) {
    ParsedAddress parsed = parse_address(address);
    switch (parsed.kind) {
        case ParsedAddress::Kind::Tcp:
            return std::make_unique<TcpTransport>(parsed.host, parsed.port);
        case ParsedAddress::Kind::Udp:
            return std::make_unique<UdpTransport>(parsed.host, parsed.port);
        case ParsedAddress::Kind::Serial:
            return std::make_unique<SerialTransport>(parsed.path, baud);
    }
    throw std::runtime_error("unreachable");
}
