#include "../../YOLO_MODEL/cpp/frame_source.hpp"

#include <gz/msgs/image.pb.h>
#include <gz/transport/Node.hh>

#include <chrono>
#include <iostream>
#include <string>
#include <thread>

namespace {
bool publish_and_read(const std::string& topic, const gz::msgs::Image& message,
                      bool expected_ok, const std::string& expected_error = {}) {
    gz::transport::Node node;
    auto publisher = node.Advertise<gz::msgs::Image>(topic);
    auto source = make_gazebo_frame_source(topic);
    std::thread sender([&] {
        for (int i = 0; i < 20; ++i) {
            publisher.Publish(message);
            std::this_thread::sleep_for(std::chrono::milliseconds(25));
        }
    });
    cv::Mat bgr;
    FrameMetadata metadata;
    std::string error;
    const bool ok = source->read(bgr, metadata, error);
    sender.join();
    if (ok != expected_ok) {
        std::cerr << topic << ": unexpected result: " << ok << " error=" << error << '\n';
        return false;
    }
    if (!expected_ok) return error.find(expected_error) != std::string::npos;
    return metadata.width == 640 && metadata.height == 480 && metadata.stride == 1920 &&
           metadata.pixel_format == "RGB_INT8" && metadata.source_timestamp_ns == 2000000003LL &&
           bgr.rows == 480 && bgr.cols == 640 && bgr.type() == CV_8UC3 &&
           bgr.at<cv::Vec3b>(0, 0) == cv::Vec3b(30, 20, 10);
}

gz::msgs::Image valid_image() {
    gz::msgs::Image image;
    image.set_width(640);
    image.set_height(480);
    image.set_step(1920);
    image.set_pixel_format_type(gz::msgs::RGB_INT8);
    std::string data(640 * 480 * 3, '\0');
    for (std::size_t i = 0; i < data.size(); i += 3) {
        data[i] = 10; data[i + 1] = 20; data[i + 2] = 30;
    }
    image.set_data(std::move(data));
    image.mutable_header()->mutable_stamp()->set_sec(2);
    image.mutable_header()->mutable_stamp()->set_nsec(3);
    return image;
}
}  // namespace

int main() {
    try {
        bool relative_rejected = false;
        try { (void)make_gazebo_frame_source("relative/topic"); }
        catch (const std::invalid_argument&) { relative_rejected = true; }
        if (!relative_rejected) return 1;

        auto valid = valid_image();
        if (!publish_and_read("/astrodrone/frame_source/valid", valid, true)) return 2;
        auto size = valid;
        size.set_width(320);
        if (!publish_and_read("/astrodrone/frame_source/size", size, false, "640x480")) return 3;
        auto format = valid;
        format.set_pixel_format_type(gz::msgs::BGR_INT8);
        if (!publish_and_read("/astrodrone/frame_source/format", format, false, "RGB_INT8")) return 4;
        auto stride = valid;
        stride.set_step(2048);
        if (!publish_and_read("/astrodrone/frame_source/stride", stride, false, "width*3")) return 5;
        std::cout << "PASS explicit_topic RGB_INT8 640x480 stride timestamp RGB_to_BGR invalid_contract_rejection\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 6;
    }
}

