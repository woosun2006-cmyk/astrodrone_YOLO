#include "yaml_settings.hpp"

#include <fstream>
#include <limits.h>
#include <stdexcept>
#include <unistd.h>
#include <vector>

const YamlValue& YamlValue::operator[](const std::string& key) const {
    auto it = children_.find(key);
    if (it == children_.end()) {
        throw std::out_of_range("YAML key not found: " + key);
    }
    return it->second;
}

std::string YamlValue::as_string() const {
    if (!is_scalar_) {
        throw std::runtime_error("YAML value is a map, not a scalar");
    }
    return scalar_;
}

long YamlValue::as_long() const { return std::stol(as_string()); }

double YamlValue::as_double() const { return std::stod(as_string()); }

long YamlValue::get_long_or(const std::string& key, long default_value) const {
    auto it = children_.find(key);
    if (it == children_.end()) return default_value;
    return it->second.as_long();
}

double YamlValue::get_double_or(const std::string& key, double default_value) const {
    auto it = children_.find(key);
    if (it == children_.end()) return default_value;
    return it->second.as_double();
}

namespace {

std::string rstrip(const std::string& s) {
    size_t end = s.find_last_not_of(" \t\r\n");
    return end == std::string::npos ? std::string() : s.substr(0, end + 1);
}

std::string strip(const std::string& s) {
    size_t start = s.find_first_not_of(" \t\r\n");
    if (start == std::string::npos) return std::string();
    size_t end = s.find_last_not_of(" \t\r\n");
    return s.substr(start, end - start + 1);
}

// Drops a trailing "  # comment" while leaving '#' inside values alone is not
// needed here: every comment in MAVLink.yaml starts a line of its own.
bool is_comment_or_blank(const std::string& line) {
    std::string trimmed = strip(line);
    return trimmed.empty() || trimmed[0] == '#';
}

}  // namespace

YamlValue parse_yaml_file(const std::string& path) {
    std::ifstream file(path);
    if (!file) {
        throw std::runtime_error("cannot open YAML file: " + path);
    }

    YamlValue root;
    root.is_scalar_ = false;

    std::vector<std::pair<int, YamlValue*>> stack;
    stack.push_back({-1, &root});

    std::string raw_line;
    while (std::getline(file, raw_line)) {
        std::string line = rstrip(raw_line);
        if (is_comment_or_blank(line)) continue;

        size_t indent = line.find_first_not_of(' ');
        size_t colon = line.find(':', indent);
        if (colon == std::string::npos) continue;  // not a "key: value" line

        std::string key = strip(line.substr(indent, colon - indent));
        std::string value = strip(line.substr(colon + 1));

        while (stack.size() > 1 && stack.back().first >= static_cast<int>(indent)) {
            stack.pop_back();
        }

        YamlValue* parent = stack.back().second;
        YamlValue child;
        if (value.empty()) {
            child.is_scalar_ = false;
        } else {
            child.is_scalar_ = true;
            child.scalar_ = value;
        }

        auto result = parent->children_.emplace(key, std::move(child));
        YamlValue* inserted = &result.first->second;
        if (!result.second) {
            *inserted = child;  // key repeated: last write wins
        }
        stack.push_back({static_cast<int>(indent), inserted});
    }

    return root;
}

namespace {

// Resolves <repo_root>/setting/<filename>, where repo_root is the parent of
// this executable's directory (control/), mirroring drone_lib.py.
YamlValue load_setting_file(const std::string& filename) {
    char exe_path[PATH_MAX];
    ssize_t len = readlink("/proc/self/exe", exe_path, sizeof(exe_path) - 1);
    std::string exe_dir;
    if (len != -1) {
        exe_path[len] = '\0';
        std::string full(exe_path);
        exe_dir = full.substr(0, full.find_last_of('/'));
    } else {
        exe_dir = ".";
    }

    std::string candidates[] = {
        exe_dir + "/../../setting/" + filename,  // build/<exe> layout
        exe_dir + "/../setting/" + filename,      // control/<exe> layout
    };
    for (const auto& candidate : candidates) {
        std::ifstream probe(candidate);
        if (probe.good()) {
            return parse_yaml_file(candidate);
        }
    }
    throw std::runtime_error(filename + " not found relative to executable");
}

}  // namespace

YamlValue load_mavlink_settings() { return load_setting_file("MAVLink.yaml"); }

YamlValue load_safety_settings() { return load_setting_file("safety.yaml"); }
