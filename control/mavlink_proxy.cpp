// Serial <-> multi-UDP MAVLink fan-out relay for setting/port.yaml's
// mavlink_control/mavlink_sensor/mavlink_gcs/mavlink_motor_test ports.
//
// control.cpp, target_distance.cpp, emergency.cpp, gcs/sender's
// telem_sender, and control/motor_test.cpp all connect via
// setting/MAVLink.yaml's real.proxy_udp address (a bind-and-reply-to-last-
// peer "udp:" socket - see mav_transport.cpp's UdpTransport) instead of
// opening the Pixhawk's serial port directly, because only one process can
// reliably own that port at a time. setting/port.yaml has always documented that something
// external ("mavproxy/mavlink-router, started outside this repo") is
// supposed to be doing that serial->UDP fan-out - but nothing in this repo
// ever set that up, and on this Jetson mavproxy.py segfaults on startup
// and mavlink-routerd isn't installed (no working system package manager
// access either). This is a minimal stand-in: open the serial port once,
// relay every byte to all three UDP ports, and relay each port's outgoing
// bytes back to the serial port.
//
// Deliberately dumb: this doesn't parse MAVLink at all, just moves raw
// bytes both directions. mavlink_parse_char() on the receiving end (every
// downstream process already does this) resyncs on its own if a read
// happens to split a packet, so byte-level relaying is enough - no need to
// buffer/frame here.
//
// Fan-out ports use a connect()ed UDP client socket, NOT mav_transport.cpp's
// UdpTransport (which binds). control.cpp/target_distance.cpp/telem_sender
// are the ones that bind those ports and reply to whoever sent them a
// datagram first (see UdpTransport's own doc comment) - if this relay also
// bound the same port, the two bound sockets would race for the same
// traffic instead of this process ever reaching the other one. connect()ing
// instead fixes the destination for send() (same trick target_link.hpp's
// TargetRangeSender uses) while still letting recv() pick up whatever that
// destination replies with, from an OS-assigned ephemeral local port - no
// bind conflict.
#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "mav_transport.hpp"
#include "yaml_settings.hpp"

namespace {

volatile std::sig_atomic_t g_stop = 0;
void on_signal(int) { g_stop = 1; }

class UdpClientPort {
public:
    UdpClientPort(const std::string& host, int port) {
        fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
        if (fd_ < 0) throw std::runtime_error("create UDP socket failed");
        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(static_cast<uint16_t>(port));
        if (::inet_pton(AF_INET, host.c_str(), &addr.sin_addr) != 1) {
            ::close(fd_);
            throw std::runtime_error("invalid UDP host: " + host);
        }
        if (::connect(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
            ::close(fd_);
            throw std::runtime_error("connect UDP " + host + ":" + std::to_string(port) + " failed: " +
                                      std::strerror(errno));
        }
    }
    ~UdpClientPort() {
        if (fd_ >= 0) ::close(fd_);
    }
    UdpClientPort(const UdpClientPort&) = delete;
    UdpClientPort& operator=(const UdpClientPort&) = delete;

    // Fire-and-forget, same posture as TargetRangeSender - the other end is
    // a bind-and-reply UdpTransport that only cares about the latest data.
    void write_bytes(const uint8_t* data, size_t len) { ::send(fd_, data, len, 0); }

    size_t read_bytes(uint8_t* buffer, size_t maxlen, int timeout_ms) {
        pollfd pfd{fd_, POLLIN, 0};
        if (::poll(&pfd, 1, timeout_ms) <= 0) return 0;
        ssize_t n = ::recv(fd_, buffer, maxlen, 0);
        return n > 0 ? static_cast<size_t>(n) : 0;
    }

private:
    int fd_ = -1;
};

// Splits "udp:host:port" into ("host", port) - same subset mav_transport.cpp's
// (unexported) parse_address() understands, needed here only for the host
// part since with_port() already handles swapping the port.
std::string udp_host_of(const std::string& address) {
    if (address.rfind("udp:", 0) != 0) {
        throw std::runtime_error("expected a udp: address: " + address);
    }
    std::string rest = address.substr(4);
    size_t sep = rest.find_last_of(':');
    if (sep == std::string::npos) {
        throw std::runtime_error("expected udp:host:port in address: " + address);
    }
    return rest.substr(0, sep);
}

struct FanoutPort {
    std::string name;
    std::unique_ptr<UdpClientPort> client;
    uint64_t bytes_out = 0;  // serial -> this UDP port
    uint64_t bytes_in = 0;   // this UDP port -> serial
};

}  // namespace

int main() {
    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);

    YamlValue mav_cfg = load_mavlink_settings();
    YamlValue serial_cfg = mav_cfg["real"]["serial"];
    std::string serial_address = serial_cfg["address"].as_string();
    int baud = static_cast<int>(serial_cfg["baud"].as_long());
    std::string proxy_udp_address = mav_cfg["real"]["proxy_udp"]["address"].as_string();

    YamlValue ports = load_port_settings();
    std::vector<std::pair<std::string, long>> port_defs = {
        {"mavlink_control", ports.get_long_or("mavlink_control", 14550)},
        {"mavlink_sensor", ports.get_long_or("mavlink_sensor", 14551)},
        {"mavlink_gcs", ports.get_long_or("mavlink_gcs", 14552)},
        {"mavlink_motor_test", ports.get_long_or("mavlink_motor_test", 14553)},
    };

    std::fprintf(stderr, "mavlink_proxy: opening serial %s @ %d baud...\n", serial_address.c_str(), baud);
    std::unique_ptr<Transport> serial = open_transport(serial_address, baud);
    std::fprintf(stderr, "mavlink_proxy: serial open.\n");

    std::string proxy_host = udp_host_of(proxy_udp_address);
    std::vector<FanoutPort> udp_ports;
    for (const auto& [name, port] : port_defs) {
        std::fprintf(stderr, "mavlink_proxy: fanning out to udp:%s:%ld (%s)\n", proxy_host.c_str(), port,
                     name.c_str());
        FanoutPort fp;
        fp.name = name;
        fp.client = std::make_unique<UdpClientPort>(proxy_host, static_cast<int>(port));
        udp_ports.push_back(std::move(fp));
    }

    std::fprintf(stderr,
                 "mavlink_proxy: relaying. this stands in for the external mavlink-router/mavproxy "
                 "setting/port.yaml expects - see this file's top comment.\n");

    constexpr int kSerialPollMs = 5;   // paces the loop; also the max added latency per direction
    constexpr int kUdpPollMs = 0;      // non-blocking - don't let one silent UDP port stall the others
    uint8_t buf[512];
    uint64_t serial_bytes_in = 0, serial_bytes_out = 0;
    auto last_report = std::chrono::steady_clock::now();

    while (!g_stop) {
        size_t n = serial->read_bytes(buf, sizeof(buf), kSerialPollMs);
        if (n > 0) {
            serial_bytes_in += n;
            for (auto& p : udp_ports) {
                p.client->write_bytes(buf, n);
                p.bytes_out += n;
            }
        }

        for (auto& p : udp_ports) {
            size_t m = p.client->read_bytes(buf, sizeof(buf), kUdpPollMs);
            if (m > 0) {
                p.bytes_in += m;
                serial_bytes_out += m;
                serial->write_bytes(buf, m);
            }
        }

        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration<double>(now - last_report).count() >= 30.0) {
            std::fprintf(stderr, "mavlink_proxy: serial in=%lu out=%lu bytes", serial_bytes_in,
                         serial_bytes_out);
            for (auto& p : udp_ports) {
                std::fprintf(stderr, " | %s out=%lu in=%lu", p.name.c_str(), p.bytes_out, p.bytes_in);
            }
            std::fprintf(stderr, "\n");
            last_report = now;
        }
    }

    std::fprintf(stderr, "mavlink_proxy: stopping.\n");
    return 0;
}
