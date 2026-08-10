#include "target_json.hpp"

#include <cmath>
#include <functional>
#include <iostream>
#include <string>
#include <vector>

namespace {

bool expect(bool condition, const char* expression, int line) {
    if (condition) return true;
    std::cerr << "line " << line << ": check failed: " << expression << std::endl;
    return false;
}

#define CHECK(expression) \
    do { \
        if (!expect(static_cast<bool>(expression), #expression, __LINE__)) return false; \
    } while (false)

bool compact_boolean() {
    auto value = target_json::json_bool(R"({"found":true})", "found");
    CHECK(value.has_value());
    CHECK(*value);
    return true;
}

bool whitespace_around_boolean() {
    auto after_colon = target_json::json_bool(R"({"found": true})", "found");
    auto around_colon = target_json::json_bool(R"({"found" : true})", "found");
    auto multiline = target_json::json_bool("{\n  \"found\"\t:\r\n true\n}", "found");
    CHECK(after_colon && *after_colon);
    CHECK(around_colon && *around_colon);
    CHECK(multiline && *multiline);
    return true;
}

bool key_order_and_false() {
    std::string body = R"({"confirmed":true,"x_px":-3.5,"found":false})";
    auto found = target_json::json_bool(body, "found");
    auto confirmed = target_json::json_bool(body, "confirmed");
    CHECK(found.has_value());
    CHECK(!*found);
    CHECK(confirmed && *confirmed);
    return true;
}

bool missing_null_and_invalid_boolean() {
    CHECK(!target_json::json_bool(R"({"found":true})", "missing").has_value());
    CHECK(!target_json::json_bool(R"({"found":null})", "found").has_value());
    CHECK(!target_json::json_bool(R"({"found":"true"})", "found").has_value());
    CHECK(!target_json::json_bool(R"({"found":truth})", "found").has_value());
    return true;
}

bool number_whitespace_negative_and_decimal() {
    auto spaced = target_json::json_number(R"({"age_ms" :  12.5 })", "age_ms");
    auto negative = target_json::json_number(R"({"x_px": -42.75})", "x_px");
    auto exponent = target_json::json_number(R"({"value":1.25e2})", "value");
    CHECK(spaced && std::abs(*spaced - 12.5) < 1e-9);
    CHECK(negative && std::abs(*negative + 42.75) < 1e-9);
    CHECK(exponent && std::abs(*exponent - 125.0) < 1e-9);
    return true;
}

bool null_and_invalid_number() {
    CHECK(!target_json::json_number(R"({"age_ms":null})", "age_ms").has_value());
    CHECK(!target_json::json_number(R"({"age_ms":"12.5"})", "age_ms").has_value());
    CHECK(!target_json::json_number(R"({"age_ms":12ms})", "age_ms").has_value());
    CHECK(!target_json::json_number(R"({"age_ms":+12})", "age_ms").has_value());
    return true;
}

bool actual_target_sample() {
    std::string body = R"({"name": "target", "detections": 1, "multi": false, "found": true, "conf": 0.912, "x_px": -12.5, "y_px": 4.0, "confirmed": true, "t": 123.4, "age_ms": 12.3, "width": 640, "height": 480})";
    auto found = target_json::json_bool(body, "found");
    auto confirmed = target_json::json_bool(body, "confirmed");
    auto x_px = target_json::json_number(body, "x_px");
    auto y_px = target_json::json_number(body, "y_px");
    auto age_ms = target_json::json_number(body, "age_ms");
    CHECK(found && *found);
    CHECK(confirmed && *confirmed);
    CHECK(x_px && std::abs(*x_px + 12.5) < 1e-9);
    CHECK(y_px && std::abs(*y_px - 4.0) < 1e-9);
    CHECK(age_ms && std::abs(*age_ms - 12.3) < 1e-9);

    std::string no_detection = R"({"found": false, "detections": 0, "multi": false, "confirmed": false, "t": 0.0, "age_ms": null, "width": 640, "height": 480})";
    auto age_null = target_json::json_number(no_detection, "age_ms");
    CHECK(!age_null.has_value());
    return true;
}

bool skips_nested_values() {
    std::string body = R"({"metadata":{"found":false},"items":[1,{"confirmed":false}],"found" : true,"confirmed":true})";
    auto found = target_json::json_bool(body, "found");
    auto confirmed = target_json::json_bool(body, "confirmed");
    CHECK(found && *found);
    CHECK(confirmed && *confirmed);
    return true;
}

}  // namespace

int main() {
    const std::vector<std::pair<const char*, std::function<bool()>>> tests = {
        {"compact_boolean", compact_boolean},
        {"whitespace_around_boolean", whitespace_around_boolean},
        {"key_order_and_false", key_order_and_false},
        {"missing_null_and_invalid_boolean", missing_null_and_invalid_boolean},
        {"number_whitespace_negative_and_decimal", number_whitespace_negative_and_decimal},
        {"null_and_invalid_number", null_and_invalid_number},
        {"actual_target_sample", actual_target_sample},
        {"skips_nested_values", skips_nested_values},
    };

    int failed = 0;
    for (const auto& test : tests) {
        if (test.second()) {
            std::cout << "PASS " << test.first << std::endl;
        } else {
            std::cerr << "FAIL " << test.first << std::endl;
            ++failed;
        }
    }
    return failed == 0 ? 0 : 1;
}
