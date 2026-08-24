#include "../app/preflight_readiness.hpp"

#include <chrono>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <string>
#include <unistd.h>

namespace {

bool check(bool value, const char* expression, int line) {
    if (value) return true;
    std::cerr << "line " << line << ": check failed: " << expression << '\n';
    return false;
}

#define CHECK(expression) \
    do { \
        if (!check(static_cast<bool>(expression), #expression, __LINE__)) return 1; \
    } while (false)

std::int64_t monotonic_ns(std::chrono::steady_clock::time_point value) {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(value.time_since_epoch()).count();
}

}  // namespace

int main() {
    const auto now = std::chrono::steady_clock::now();
    const std::string marker = "/tmp/astrodrone-preflight-readiness-" +
                               std::to_string(static_cast<long long>(::getpid()));
    std::remove(marker.c_str());

    const auto missing = app::read_camera_frame_readiness(marker, now);
    CHECK(!missing.valid);
    CHECK(app::evaluate_vision_readiness(true, true, true, true, missing).reason == "실제 frame 미수신");
    CHECK(app::evaluate_vision_readiness(true, true, false, true, missing).reason ==
          "YOLO readiness marker 없음");
    CHECK(app::evaluate_vision_readiness(true, true, true, false, missing).reason ==
          "카메라 topic 미수신");

    {
        std::ofstream output(marker);
        output << "frame_sequence=7\n"
               << "frame_timestamp_ns=123\n"
               << "monotonic_timestamp_ns=" << monotonic_ns(now) << "\n"
               << "width=640\nheight=480\nsource=gazebo:/camera\n";
    }
    const auto fresh = app::read_camera_frame_readiness(marker, now);
    CHECK(fresh.valid);
    CHECK(fresh.fresh);
    CHECK(fresh.frame_sequence == 7);
    CHECK(app::evaluate_vision_readiness(true, true, true, true, fresh).ready);

    const auto stale = app::read_camera_frame_readiness(
        marker, now + std::chrono::seconds(2));
    CHECK(!stale.fresh);
    CHECK(app::evaluate_vision_readiness(true, true, true, true, stale).reason == "frame stale");
    CHECK(app::evaluate_vision_readiness(false, true, true, true, fresh).reason == "YOLO 프로세스 미실행");
    CHECK(app::evaluate_vision_readiness(true, false, true, true, fresh).reason == "HTTP readiness 실패");

    std::remove(marker.c_str());
    return 0;
}
