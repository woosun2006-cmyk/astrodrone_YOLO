#include "target_json.hpp"

#include <cctype>
#include <cmath>
#include <string_view>

namespace target_json {
namespace {

bool is_json_space(char ch) {
    return ch == ' ' || ch == '\t' || ch == '\r' || ch == '\n';
}

void skip_space(std::string_view text, size_t& pos) {
    while (pos < text.size() && is_json_space(text[pos])) ++pos;
}

bool scan_string(std::string_view text, size_t& pos) {
    if (pos >= text.size() || text[pos] != '"') return false;
    ++pos;
    while (pos < text.size()) {
        if (text[pos] == '\\') {
            ++pos;
            if (pos >= text.size()) return false;
            ++pos;
        } else if (text[pos] == '"') {
            ++pos;
            return true;
        } else {
            ++pos;
        }
    }
    return false;
}

bool scan_composite(std::string_view text, size_t& pos) {
    if (pos >= text.size() || (text[pos] != '{' && text[pos] != '[')) return false;
    int depth = 0;
    while (pos < text.size()) {
        if (text[pos] == '"') {
            if (!scan_string(text, pos)) return false;
            continue;
        }
        if (text[pos] == '{' || text[pos] == '[') {
            ++depth;
        } else if (text[pos] == '}' || text[pos] == ']') {
            --depth;
            ++pos;
            if (depth == 0) return true;
            if (depth < 0) return false;
            continue;
        }
        ++pos;
    }
    return false;
}

bool scan_value(std::string_view text, size_t& pos) {
    if (pos >= text.size()) return false;
    if (text[pos] == '"') return scan_string(text, pos);
    if (text[pos] == '{' || text[pos] == '[') return scan_composite(text, pos);

    size_t begin = pos;
    while (pos < text.size() && text[pos] != ',' && text[pos] != '}') ++pos;
    size_t end = pos;
    while (end > begin && is_json_space(text[end - 1])) --end;
    return end > begin;
}

std::optional<std::string_view> find_top_level_value(const std::string& body,
                                                      const std::string& key) {
    std::string_view text(body);
    size_t pos = 0;
    skip_space(text, pos);
    if (pos >= text.size() || text[pos] != '{') return std::nullopt;
    ++pos;

    while (true) {
        skip_space(text, pos);
        if (pos >= text.size()) return std::nullopt;
        if (text[pos] == '}') return std::nullopt;
        if (text[pos] != '"') return std::nullopt;

        size_t key_start = ++pos;
        bool escaped_key = false;
        while (pos < text.size() && text[pos] != '"') {
            if (text[pos] == '\\') {
                escaped_key = true;
                ++pos;
                if (pos >= text.size()) return std::nullopt;
            }
            ++pos;
        }
        if (pos >= text.size()) return std::nullopt;
        size_t key_end = pos++;

        skip_space(text, pos);
        if (pos >= text.size() || text[pos] != ':') return std::nullopt;
        ++pos;
        skip_space(text, pos);

        size_t value_start = pos;
        if (!scan_value(text, pos)) return std::nullopt;
        size_t value_end = pos;
        while (value_end > value_start && is_json_space(text[value_end - 1])) --value_end;

        bool key_matches = !escaped_key &&
                           text.substr(key_start, key_end - key_start) == std::string_view(key);
        if (key_matches) return text.substr(value_start, value_end - value_start);

        skip_space(text, pos);
        if (pos >= text.size()) return std::nullopt;
        if (text[pos] == ',') {
            ++pos;
            continue;
        }
        if (text[pos] == '}') return std::nullopt;
        return std::nullopt;
    }
}

bool is_json_number(std::string_view token) {
    size_t pos = 0;
    if (pos < token.size() && token[pos] == '-') ++pos;
    if (pos >= token.size()) return false;

    if (token[pos] == '0') {
        ++pos;
    } else if (token[pos] >= '1' && token[pos] <= '9') {
        while (pos < token.size() && std::isdigit(static_cast<unsigned char>(token[pos]))) ++pos;
    } else {
        return false;
    }

    if (pos < token.size() && token[pos] == '.') {
        ++pos;
        size_t fraction_start = pos;
        while (pos < token.size() && std::isdigit(static_cast<unsigned char>(token[pos]))) ++pos;
        if (pos == fraction_start) return false;
    }

    if (pos < token.size() && (token[pos] == 'e' || token[pos] == 'E')) {
        ++pos;
        if (pos < token.size() && (token[pos] == '+' || token[pos] == '-')) ++pos;
        size_t exponent_start = pos;
        while (pos < token.size() && std::isdigit(static_cast<unsigned char>(token[pos]))) ++pos;
        if (pos == exponent_start) return false;
    }
    return pos == token.size();
}

std::optional<std::string> decode_json_string(std::string_view token) {
    if (token.size() < 2 || token.front() != '"' || token.back() != '"') return std::nullopt;
    std::string value;
    value.reserve(token.size() - 2);
    for (size_t i = 1; i + 1 < token.size(); ++i) {
        if (token[i] != '\\') {
            value.push_back(token[i]);
            continue;
        }
        if (++i + 1 >= token.size()) return std::nullopt;
        switch (token[i]) {
            case '"': value.push_back('"'); break;
            case '\\': value.push_back('\\'); break;
            case '/': value.push_back('/'); break;
            case 'b': value.push_back('\b'); break;
            case 'f': value.push_back('\f'); break;
            case 'n': value.push_back('\n'); break;
            case 'r': value.push_back('\r'); break;
            case 't': value.push_back('\t'); break;
            default: return std::nullopt;
        }
    }
    return value;
}

}  // namespace

std::optional<bool> json_bool(const std::string& body, const std::string& key) {
    auto token = find_top_level_value(body, key);
    if (!token) return std::nullopt;
    if (*token == "true") return true;
    if (*token == "false") return false;
    return std::nullopt;
}

std::optional<double> json_number(const std::string& body, const std::string& key) {
    auto token = find_top_level_value(body, key);
    if (!token || !is_json_number(*token)) return std::nullopt;
    try {
        double value = std::stod(std::string(*token));
        if (!std::isfinite(value)) return std::nullopt;
        return value;
    } catch (...) {
        return std::nullopt;
    }
}

std::optional<std::string> json_string(const std::string& body, const std::string& key) {
    auto token = find_top_level_value(body, key);
    if (!token) return std::nullopt;
    return decode_json_string(*token);
}

}  // namespace target_json
