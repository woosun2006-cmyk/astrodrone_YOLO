#pragma once

// Minimal GET-only threaded HTTP server, standing in for yolo_live.py's
// http.server.BaseHTTPRequestHandler + socketserver.ThreadingTCPServer.
// One detached thread per connection (daemon_threads=True equivalent);
// route handlers get raw socket access so a handler like /stream can hold
// the connection open and push a multipart MJPEG feed indefinitely.

#include <atomic>
#include <functional>
#include <map>
#include <string>

class HttpConnection {
public:
    explicit HttpConnection(int fd) : fd_(fd) {}

    // Buffered response: one shot, sets Content-Length, closes out the reply.
    void send(int status, const std::string& content_type, const std::string& body);

    // Starts a multipart/x-mixed-replace (or any non-Content-Length) reply;
    // caller then streams with write_raw() until it returns false.
    bool send_stream_headers(const std::string& content_type);
    bool write_raw(const std::string& data);
    void send_404();

private:
    int fd_;
};

struct HttpRequest {
    std::string path;
    std::map<std::string, std::string> query;

    // Returns query[key] as a double, or default_value if absent/unparsable.
    double query_double(const std::string& key, double default_value) const;
};

using RouteFn = std::function<void(const HttpRequest&, HttpConnection&)>;

class HttpServer {
public:
    explicit HttpServer(int port);
    ~HttpServer();

    HttpServer(const HttpServer&) = delete;
    HttpServer& operator=(const HttpServer&) = delete;

    void add_route(const std::string& path, RouteFn fn);

    // Blocks accepting connections until stop() is called (from a signal
    // handler or another thread).
    void serve_forever();
    void stop();

private:
    void handle_client(int fd);

    int listen_fd_ = -1;
    std::map<std::string, RouteFn> routes_;
    std::atomic<bool> stopping_{false};
};
