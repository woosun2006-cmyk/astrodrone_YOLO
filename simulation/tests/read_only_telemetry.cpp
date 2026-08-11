// Receive-only MAVLink smoke-test client. This file intentionally contains no
// network write/send call and no MAVLink command/setpoint packer.
#include <arpa/inet.h>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <map>
#include <netinet/in.h>
#include <poll.h>
#include <set>
#include <stdexcept>
#include <string>
#include <sys/socket.h>
#include <unistd.h>

#include "third_party/mavlink/common/mavlink.h"

namespace {

using Clock = std::chrono::steady_clock;

struct Endpoint {
    enum class Kind { Tcp, Udp } kind;
    std::string host;
    uint16_t port;
};

Endpoint parse_endpoint(const std::string& value) {
    Endpoint endpoint;
    std::string remainder;
    if (value.rfind("tcp:", 0) == 0) {
        endpoint.kind = Endpoint::Kind::Tcp;
        remainder = value.substr(4);
    } else if (value.rfind("udp:", 0) == 0) {
        endpoint.kind = Endpoint::Kind::Udp;
        remainder = value.substr(4);
    } else {
        throw std::runtime_error("only tcp: or udp: loopback endpoints are accepted");
    }
    const auto separator = remainder.find_last_of(':');
    if (separator == std::string::npos) throw std::runtime_error("expected host:port");
    endpoint.host = remainder.substr(0, separator);
    if (endpoint.host == "localhost") endpoint.host = "127.0.0.1";
    if (endpoint.host != "127.0.0.1") throw std::runtime_error("only loopback is accepted");
    const int port = std::stoi(remainder.substr(separator + 1));
    if (port < 1 || port > 65535) throw std::runtime_error("port must be in 1..65535");
    endpoint.port = static_cast<uint16_t>(port);
    return endpoint;
}

class ReceiveSocket {
public:
    explicit ReceiveSocket(const Endpoint& endpoint) {
        const int type = endpoint.kind == Endpoint::Kind::Tcp ? SOCK_STREAM : SOCK_DGRAM;
        fd_ = ::socket(AF_INET, type, 0);
        if (fd_ < 0) fail("socket");
        sockaddr_in address{};
        address.sin_family = AF_INET;
        address.sin_port = htons(endpoint.port);
        inet_pton(AF_INET, endpoint.host.c_str(), &address.sin_addr);
        if (endpoint.kind == Endpoint::Kind::Tcp) {
            if (::connect(fd_, reinterpret_cast<sockaddr*>(&address), sizeof(address)) != 0) fail("connect");
        } else if (::bind(fd_, reinterpret_cast<sockaddr*>(&address), sizeof(address)) != 0) {
            fail("bind");
        }
    }
    ~ReceiveSocket() { if (fd_ >= 0) ::close(fd_); }
    int receive(uint8_t* buffer, size_t size, int timeout_ms) const {
        pollfd descriptor{fd_, POLLIN, 0};
        const int ready = ::poll(&descriptor, 1, timeout_ms);
        if (ready < 0) fail("poll");
        if (ready == 0) return 0;
        const ssize_t count = ::recv(fd_, buffer, size, 0);
        if (count < 0) fail("recv");
        if (count == 0) throw std::runtime_error("MAVLink stream closed");
        return static_cast<int>(count);
    }
private:
    [[noreturn]] static void fail(const char* operation) {
        throw std::runtime_error(std::string(operation) + ": " + std::strerror(errno));
    }
    int fd_ = -1;
};

double elapsed(const Clock::time_point& start, const Clock::time_point& value) {
    return std::chrono::duration<double>(value - start).count();
}

bool valid_lat_lon(int32_t lat, int32_t lon) {
    return lat >= -900000000 && lat <= 900000000 && lon >= -1800000000 && lon <= 1800000000 &&
           !(lat == 0 && lon == 0);
}

}  // namespace

int main(int argc, char** argv) {
    try {
        std::string address = "udp:127.0.0.1:14551";
        double timeout_seconds = 30.0;
        double freshness_seconds = 3.0;
        for (int i = 1; i < argc; ++i) {
            const std::string argument = argv[i];
            auto next = [&](const char* name) {
                if (++i >= argc) throw std::runtime_error(std::string(name) + " requires a value");
                return std::string(argv[i]);
            };
            if (argument == "--address") address = next("--address");
            else if (argument == "--timeout") timeout_seconds = std::stod(next("--timeout"));
            else if (argument == "--freshness") freshness_seconds = std::stod(next("--freshness"));
            else throw std::runtime_error("unknown argument: " + argument);
        }
        if (timeout_seconds <= 0 || timeout_seconds > 60) throw std::runtime_error("timeout must be in (0, 60]");
        if (freshness_seconds <= 0 || freshness_seconds > timeout_seconds) {
            throw std::runtime_error("freshness must be positive and no greater than timeout");
        }

        ReceiveSocket input(parse_endpoint(address));
        std::cout << "RECEIVE_ONLY endpoint=" << address << " timeout_s=" << timeout_seconds
                  << " freshness_s=" << freshness_seconds << std::endl;

        const auto start = Clock::now();
        const auto deadline = start + std::chrono::duration<double>(timeout_seconds);
        std::map<uint32_t, uint64_t> counts;
        std::set<std::pair<uint8_t, uint8_t>> sources;
        bool have_any = false;
        bool armed_seen = false;
        bool gps_valid = false;
        bool global_valid = false;
        bool local_valid = false;
        auto first_receive = start;
        auto last_receive = start;
        auto first_heartbeat = start;
        auto last_heartbeat = start;
        auto first_gps = start;
        auto last_gps = start;
        auto first_global = start;
        auto last_global = start;
        auto first_local = start;
        auto last_local = start;
        uint8_t gps_fix_type = 0;
        int32_t gps_lat = 0, gps_lon = 0;
        int32_t global_lat = 0, global_lon = 0, global_relative_alt = 0;
        float local_x = 0, local_y = 0, local_z = 0;
        mavlink_message_t message{};
        mavlink_status_t status{};
        uint8_t buffer[4096];

        while (Clock::now() < deadline) {
            const int byte_count = input.receive(buffer, sizeof(buffer), 250);
            for (int i = 0; i < byte_count; ++i) {
                if (!mavlink_parse_char(MAVLINK_COMM_0, buffer[i], &message, &status)) continue;
                const auto now = Clock::now();
                if (!have_any) first_receive = now;
                have_any = true;
                last_receive = now;
                counts[message.msgid]++;
                sources.insert({message.sysid, message.compid});

                if (message.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
                    mavlink_heartbeat_t value{};
                    mavlink_msg_heartbeat_decode(&message, &value);
                    if (counts[message.msgid] == 1) first_heartbeat = now;
                    last_heartbeat = now;
                    armed_seen = armed_seen || ((value.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0);
                } else if (message.msgid == MAVLINK_MSG_ID_GPS_RAW_INT) {
                    mavlink_gps_raw_int_t value{};
                    mavlink_msg_gps_raw_int_decode(&message, &value);
                    if (counts[message.msgid] == 1) first_gps = now;
                    last_gps = now;
                    gps_fix_type = value.fix_type;
                    gps_lat = value.lat;
                    gps_lon = value.lon;
                    gps_valid = value.fix_type >= 3 && valid_lat_lon(value.lat, value.lon);
                } else if (message.msgid == MAVLINK_MSG_ID_GLOBAL_POSITION_INT) {
                    mavlink_global_position_int_t value{};
                    mavlink_msg_global_position_int_decode(&message, &value);
                    if (counts[message.msgid] == 1) first_global = now;
                    last_global = now;
                    global_lat = value.lat;
                    global_lon = value.lon;
                    global_relative_alt = value.relative_alt;
                    global_valid = valid_lat_lon(value.lat, value.lon);
                } else if (message.msgid == MAVLINK_MSG_ID_LOCAL_POSITION_NED) {
                    mavlink_local_position_ned_t value{};
                    mavlink_msg_local_position_ned_decode(&message, &value);
                    if (counts[message.msgid] == 1) first_local = now;
                    last_local = now;
                    local_x = value.x;
                    local_y = value.y;
                    local_z = value.z;
                    local_valid = std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
                }
            }
        }

        const auto finish = Clock::now();
        auto count = [&](uint32_t id) { auto it = counts.find(id); return it == counts.end() ? 0ULL : it->second; };
        auto age = [&](uint64_t value_count, const Clock::time_point& last) {
            return value_count == 0 ? -1.0 : std::chrono::duration<double>(finish - last).count();
        };
        const double heartbeat_age = age(count(MAVLINK_MSG_ID_HEARTBEAT), last_heartbeat);
        const double global_age = age(count(MAVLINK_MSG_ID_GLOBAL_POSITION_INT), last_global);
        const double local_age = age(count(MAVLINK_MSG_ID_LOCAL_POSITION_NED), last_local);
        const bool heartbeat_fresh = heartbeat_age >= 0 && heartbeat_age <= freshness_seconds;
        const bool position_fresh = (global_valid && global_age <= freshness_seconds) ||
                                    (local_valid && local_age <= freshness_seconds);

        std::cout << std::fixed << std::setprecision(3);
        std::cout << "RECEIVE_WINDOW first_s=" << (have_any ? elapsed(start, first_receive) : -1)
                  << " last_s=" << (have_any ? elapsed(start, last_receive) : -1)
                  << " duration_s=" << elapsed(start, finish) << std::endl;
        std::cout << "FIRST_TIMES heartbeat_s=" << (count(MAVLINK_MSG_ID_HEARTBEAT) ? elapsed(start, first_heartbeat) : -1)
                  << " gps_s=" << (count(MAVLINK_MSG_ID_GPS_RAW_INT) ? elapsed(start, first_gps) : -1)
                  << " global_s=" << (count(MAVLINK_MSG_ID_GLOBAL_POSITION_INT) ? elapsed(start, first_global) : -1)
                  << " local_s=" << (count(MAVLINK_MSG_ID_LOCAL_POSITION_NED) ? elapsed(start, first_local) : -1) << std::endl;
        std::cout << "COUNTS HEARTBEAT=" << count(MAVLINK_MSG_ID_HEARTBEAT)
                  << " SYS_STATUS=" << count(MAVLINK_MSG_ID_SYS_STATUS)
                  << " GPS_RAW_INT=" << count(MAVLINK_MSG_ID_GPS_RAW_INT)
                  << " GLOBAL_POSITION_INT=" << count(MAVLINK_MSG_ID_GLOBAL_POSITION_INT)
                  << " LOCAL_POSITION_NED=" << count(MAVLINK_MSG_ID_LOCAL_POSITION_NED)
                  << " ATTITUDE=" << count(MAVLINK_MSG_ID_ATTITUDE) << std::endl;
        std::cout << "MSGID_COUNTS";
        for (const auto& entry : counts) std::cout << ' ' << entry.first << '=' << entry.second;
        std::cout << std::endl << "SOURCES";
        for (const auto& source : sources) std::cout << " sysid=" << static_cast<int>(source.first)
                                                     << "/compid=" << static_cast<int>(source.second);
        std::cout << std::endl;
        std::cout << "GPS fix_type=" << static_cast<int>(gps_fix_type)
                  << " lat=" << gps_lat / 1.0e7 << " lon=" << gps_lon / 1.0e7
                  << " valid=" << (gps_valid ? "true" : "false") << std::endl;
        std::cout << "GLOBAL_POSITION lat=" << global_lat / 1.0e7 << " lon=" << global_lon / 1.0e7
                  << " relative_alt_m=" << global_relative_alt / 1000.0
                  << " valid=" << (global_valid ? "true" : "false") << " age_s=" << global_age << std::endl;
        std::cout << "LOCAL_POSITION_NED x=" << local_x << " y=" << local_y << " z=" << local_z
                  << " valid=" << (local_valid ? "true" : "false") << " age_s=" << local_age << std::endl;
        std::cout << "SAFETY armed_seen=" << (armed_seen ? "true" : "false") << std::endl;
        std::cout << "RESULT heartbeat_fresh=" << (heartbeat_fresh ? "PASS" : "FAIL")
                  << " position_fresh=" << (position_fresh ? "PASS" : "FAIL")
                  << " overall=" << (heartbeat_fresh && position_fresh && !armed_seen ? "PASS" : "FAIL") << std::endl;
        return heartbeat_fresh && position_fresh && !armed_seen ? 0 : 1;
    } catch (const std::exception& error) {
        std::cerr << error.what() << std::endl;
        return 1;
    }
}
