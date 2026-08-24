#include "target_link.hpp"

#include <chrono>
#include <cstring>
#include <iostream>
#include <thread>

namespace {

bool same_target_fields(const TargetRangeMsg& a, const TargetRangeMsg& b) {
    return a.seq == b.seq &&
           a.bbox_x_px == b.bbox_x_px &&
           a.bbox_y_px == b.bbox_y_px &&
           a.bbox_width_px == b.bbox_width_px &&
           a.bbox_height_px == b.bbox_height_px &&
           a.target_confidence == b.target_confidence &&
           std::strncmp(a.class_name, b.class_name, sizeof(a.class_name)) == 0;
}

bool receive_until(TargetRangeReceiver& receiver, TargetRangeMsg& out) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(500);
    while (std::chrono::steady_clock::now() < deadline) {
        if (receiver.poll(out)) return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    return false;
}

}  // namespace

int main() {
    constexpr int flight_port = 25120;
    constexpr int gcs_port = 25122;
    TargetRangeReceiver flight_receiver(flight_port);
    TargetRangeReceiver gcs_receiver(gcs_port);
    TargetRangeSender flight_sender(flight_port);
    TargetRangeSender gcs_sender(gcs_port);

    TargetRangeMsg input{};
    input.seq = 42;
    input.bbox_x_px = 12.5F;
    input.bbox_y_px = -8.0F;
    input.bbox_width_px = 50.0F;
    input.bbox_height_px = 60.0F;
    input.target_confidence = 0.83F;
    std::strncpy(input.class_name, "astro-drone", sizeof(input.class_name) - 1);

    flight_sender.send(input);
    gcs_sender.send(input);

    TargetRangeMsg flight_copy{};
    TargetRangeMsg gcs_copy{};
    if (!receive_until(flight_receiver, flight_copy) ||
        !receive_until(gcs_receiver, gcs_copy) ||
        !same_target_fields(flight_copy, gcs_copy) ||
        !same_target_fields(input, flight_copy)) {
        std::cerr << "TargetRangeMsg fan-out mismatch" << std::endl;
        return 1;
    }
    std::cout << "PASS target range fan-out 15020/15022 equivalent payloads" << std::endl;
    return 0;
}
