#pragma once

#include <map>
#include <string>

// Minimal indentation-based YAML reader for this project's settings files.
// Supports nested string-keyed maps with scalar leaf values only (no lists,
// no quoting rules) -- the subset actually used by setting/MAVLink.yaml.
class YamlValue {
public:
    const YamlValue& operator[](const std::string& key) const;
    std::string as_string() const;
    long as_long() const;
    double as_double() const;

    long get_long_or(const std::string& key, long default_value) const;
    double get_double_or(const std::string& key, double default_value) const;

private:
    bool is_scalar_ = false;
    std::string scalar_;
    std::map<std::string, YamlValue> children_;

    friend YamlValue parse_yaml_file(const std::string& path);
};

YamlValue parse_yaml_file(const std::string& path);

// Resolves to <repo_root>/setting/MAVLink.yaml and parses it, mirroring
// drone_lib.py's load_mavlink_settings().
YamlValue load_mavlink_settings();

// Resolves to <repo_root>/setting/safety.yaml and parses it (altitude
// limits, etc.) -- same executable-relative lookup as load_mavlink_settings().
YamlValue load_safety_settings();

// Resolves to <repo_root>/setting/port.yaml and parses it (HTTP server
// ports, MAVLink proxy ports) -- same executable-relative lookup as
// load_mavlink_settings().
YamlValue load_port_settings();

// Resolves to <repo_root>/setting/rate.yaml and parses it (frame-rate /
// polling-rate values shared with YOLO_MODEL/yolo_live.py) -- same
// executable-relative lookup as load_mavlink_settings().
YamlValue load_rate_settings();

// Replaces the ":<port>" suffix of a "udp:host:port" / "tcp:host:port"
// address with a different port, keeping the scheme and host as-is.
std::string with_port(const std::string& address, long port);
