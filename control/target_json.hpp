#pragma once

#include <optional>
#include <string>

namespace target_json {

// Reads primitive fields from the top-level object returned by
// YOLO_MODEL/yolo_live.py's /target endpoint. This is intentionally not a
// general-purpose JSON parser: nested values are skipped safely, but only
// top-level boolean and number fields are exposed. Missing keys, null and
// malformed values return std::nullopt.
std::optional<bool> json_bool(const std::string& body, const std::string& key);
std::optional<double> json_number(const std::string& body, const std::string& key);

}  // namespace target_json
