#pragma once

#include <opencv2/core.hpp>

#include <cstdint>
#include <condition_variable>
#include <deque>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

// Best-effort debug-video publisher. It owns a bounded latest-frame queue and
// never runs in the YOLO/control thread's critical path. A missing receiver,
// dropped frame, or stopped publisher must not affect inference or /target.
class GcsVideoPublisher {
public:
    using EncodedFrameCallback =
        std::function<void(const std::vector<std::uint8_t>&, std::uint64_t, std::int64_t)>;

    explicit GcsVideoPublisher(EncodedFrameCallback callback = {});
    ~GcsVideoPublisher();

    GcsVideoPublisher(const GcsVideoPublisher&) = delete;
    GcsVideoPublisher& operator=(const GcsVideoPublisher&) = delete;

    void submit(const cv::Mat& bgr, std::uint64_t frame_sequence,
                std::int64_t timestamp_ns);

private:
    struct Frame {
        cv::Mat bgr;
        std::uint64_t sequence = 0;
        std::int64_t timestamp_ns = -1;
    };

    void run();

    std::string host_;
    std::uint16_t port_ = 15560;
    int width_ = 600;
    int quality_ = 50;
    double fps_ = 6.0;
    int fd_ = -1;
    EncodedFrameCallback encoded_frame_callback_;
    std::mutex mutex_;
    std::condition_variable condition_;
    std::deque<Frame> queue_;
    bool stopping_ = false;
    std::thread worker_;
};
