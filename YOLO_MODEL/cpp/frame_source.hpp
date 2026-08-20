#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include <opencv2/core/mat.hpp>

struct FrameMetadata {
    std::string source;
    std::string pixel_format;
    std::uint32_t width = 0;
    std::uint32_t height = 0;
    std::uint32_t stride = 0;
    std::int64_t source_timestamp_ns = -1;
};

class FrameSource {
public:
    virtual ~FrameSource() = default;
    virtual bool read(cv::Mat& bgr, FrameMetadata& metadata, std::string& error) = 0;
};

// Common Jetson camera adapter. Its pipeline text is intentionally kept
// identical to the pre-adapter yolo_live grabber.
std::unique_ptr<FrameSource> make_jetson_camera_frame_source();

// Compatibility name for existing launchers and tests.
std::unique_ptr<FrameSource> make_imx219_frame_source();

// Requires an explicit gz.msgs.Image topic. There is deliberately no topic
// discovery or first-camera fallback.
std::unique_ptr<FrameSource> make_gazebo_frame_source(const std::string& topic);
