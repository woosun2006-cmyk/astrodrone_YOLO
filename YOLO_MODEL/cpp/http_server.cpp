#include "http_server.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstring>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <thread>

namespace {

std::string percent_decode(const std::string& s) {
    std::string out;
    out.reserve(s.size());
    for (size_t i = 0; i < s.size(); ++i) {
        if (s[i] == '%' && i + 2 < s.size()) {
            auto hex = [](char c) -> int {
                if (c >= '0' && c <= '9') return c - '0';
                if (c >= 'a' && c <= 'f') return c - 'a' + 10;
                if (c >= 'A' && c <= 'F') return c - 'A' + 10;
                return -1;
            };
            int hi = hex(s[i + 1]), lo = hex(s[i + 2]);
            if (hi >= 0 && lo >= 0) {
                out.push_back(static_cast<char>((hi << 4) | lo));
                i += 2;
                continue;
            }
        }
        out.push_back(s[i] == '+' ? ' ' : s[i]);
    }
    return out;
}

std::map<std::string, std::string> parse_query(const std::string& query) {
    std::map<std::string, std::string> out;
    std::istringstream iss(query);
    std::string pair;
    while (std::getline(iss, pair, '&')) {
        if (pair.empty()) continue;
        size_t eq = pair.find('=');
        std::string key = eq == std::string::npos ? pair : pair.substr(0, eq);
        std::string val = eq == std::string::npos ? "" : pair.substr(eq + 1);
        out[percent_decode(key)] = percent_decode(val);
    }
    return out;
}

// Reads until "\r\n\r\n" (end of headers), discarding the header block -
// this server never needs a request body (GET only).
bool read_request_line_and_headers(int fd, std::string& request_line) {
    std::string buf;
    char chunk[512];
    while (buf.find("\r\n\r\n") == std::string::npos) {
        ssize_t n = ::recv(fd, chunk, sizeof(chunk), 0);
        if (n <= 0) return false;
        buf.append(chunk, static_cast<size_t>(n));
        if (buf.size() > 16384) return false;  // guard against a runaway header block
    }
    size_t line_end = buf.find("\r\n");
    if (line_end == std::string::npos) return false;
    request_line = buf.substr(0, line_end);
    return true;
}

}  // namespace

void HttpConnection::send(int status, const std::string& content_type, const std::string& body) {
    std::ostringstream head;
    head << "HTTP/1.0 " << status << (status == 200 ? " OK" : " ERROR") << "\r\n"
         << "Content-Type: " << content_type << "\r\n"
         << "Content-Length: " << body.size() << "\r\n"
         << "Connection: close\r\n\r\n";
    std::string h = head.str();
    ::send(fd_, h.data(), h.size(), MSG_NOSIGNAL);
    ::send(fd_, body.data(), body.size(), MSG_NOSIGNAL);
}

void HttpConnection::send_404() { send(404, "text/plain", "not found"); }

bool HttpConnection::send_stream_headers(const std::string& content_type) {
    std::ostringstream head;
    head << "HTTP/1.0 200 OK\r\n" << "Content-Type: " << content_type << "\r\n\r\n";
    std::string h = head.str();
    return ::send(fd_, h.data(), h.size(), MSG_NOSIGNAL) == static_cast<ssize_t>(h.size());
}

bool HttpConnection::write_raw(const std::string& data) {
    size_t sent = 0;
    while (sent < data.size()) {
        ssize_t n = ::send(fd_, data.data() + sent, data.size() - sent, MSG_NOSIGNAL);
        if (n <= 0) return false;
        sent += static_cast<size_t>(n);
    }
    return true;
}

double HttpRequest::query_double(const std::string& key, double default_value) const {
    auto it = query.find(key);
    if (it == query.end()) return default_value;
    try {
        return std::stod(it->second);
    } catch (...) {
        return default_value;
    }
}

HttpServer::HttpServer(int port) {
    listen_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    if (listen_fd_ < 0) throw std::runtime_error("socket() failed");

    int one = 1;
    setsockopt(listen_fd_, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(static_cast<uint16_t>(port));

    if (bind(listen_fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
        ::close(listen_fd_);
        throw std::runtime_error("bind() failed on port " + std::to_string(port));
    }
    if (listen(listen_fd_, 32) != 0) {
        ::close(listen_fd_);
        throw std::runtime_error("listen() failed");
    }
}

HttpServer::~HttpServer() {
    if (listen_fd_ >= 0) ::close(listen_fd_);
}

void HttpServer::add_route(const std::string& path, RouteFn fn) { routes_[path] = std::move(fn); }

void HttpServer::handle_client(int fd) {
    std::string request_line;
    if (read_request_line_and_headers(fd, request_line)) {
        // "GET /path?query HTTP/1.1"
        std::istringstream iss(request_line);
        std::string method, target, version;
        iss >> method >> target >> version;

        if (method == "GET") {
            std::string path = target, query;
            size_t qpos = target.find('?');
            if (qpos != std::string::npos) {
                path = target.substr(0, qpos);
                query = target.substr(qpos + 1);
            }

            HttpRequest req;
            req.path = path;
            req.query = parse_query(query);
            HttpConnection conn(fd);

            auto it = routes_.find(path);
            if (it != routes_.end()) {
                try {
                    it->second(req, conn);
                } catch (const std::exception& e) {
                    std::cerr << "[http] handler for " << path << " threw: " << e.what() << std::endl;
                }
            } else {
                conn.send_404();
            }
        }
    }
    ::close(fd);
}

void HttpServer::serve_forever() {
    while (!stopping_.load()) {
        sockaddr_in addr{};
        socklen_t len = sizeof(addr);
        int client = ::accept(listen_fd_, reinterpret_cast<sockaddr*>(&addr), &len);
        if (client < 0) {
            if (stopping_.load()) break;
            continue;
        }
        std::thread(&HttpServer::handle_client, this, client).detach();
    }
}

void HttpServer::stop() {
    stopping_.store(true);
    ::shutdown(listen_fd_, SHUT_RDWR);
}
