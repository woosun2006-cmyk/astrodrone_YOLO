#include "frame_source.hpp"

#include <chrono>
#include <condition_variable>
#include <cstring>
#include <functional>
#include <mutex>
#include <stdexcept>

#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#ifdef ASTRODRONE_HAVE_GZ_FRAME_SOURCE
#include <gz/msgs/image.pb.h>
#include <gz/transport/Node.hh>
#endif

namespace {
constexpr int kWidth = 640;
constexpr int kHeight = 480;
constexpr int kFps = 30;
constexpr auto kGazeboFrameTimeout = std::chrono::milliseconds(500);

class JetsonCameraFrameSource final : public FrameSource {
public:
    JetsonCameraFrameSource() {
        pipeline_ = "nvarguscamerasrc sensor-id=0 wbmode=3 ! "
                    "video/x-raw(memory:NVMM),width=640,height=480,format=NV12,framerate=30/1 ! "
                    "nvvidconv flip-method=0 ! "
                    "video/x-raw,width=640,height=480,format=BGRx ! "
                    "videoconvert ! video/x-raw,format=BGR ! appsink drop=1";
    }

    bool read(cv::Mat& bgr, FrameMetadata& metadata, std::string& error) override {
        if (!capture_.isOpened() && !capture_.open(pipeline_, cv::CAP_GSTREAMER)) {
            error = "IMX219 GStreamer pipeline could not be opened";
            return false;
        }
        if (!capture_.read(bgr)) {
            capture_.release();
            error = "IMX219 GStreamer frame read failed";
            return false;
        }
        if (bgr.cols != kWidth || bgr.rows != kHeight || bgr.type() != CV_8UC3) {
            error = "IMX219 returned an unexpected frame layout";
            return false;
        }
        metadata = {"imx219", "BGR_INT8", static_cast<std::uint32_t>(bgr.cols),
                    static_cast<std::uint32_t>(bgr.rows),
                    static_cast<std::uint32_t>(bgr.step), -1};
        return true;
    }

private:
    std::string pipeline_;
    cv::VideoCapture capture_;
};

#ifdef ASTRODRONE_HAVE_GZ_FRAME_SOURCE
class GazeboFrameSource final : public FrameSource {
public:
    explicit GazeboFrameSource(std::string topic) : topic_(std::move(topic)) {
        if (topic_.empty() || topic_.front() != '/') {
            throw std::invalid_argument("Gazebo FrameSource requires an absolute explicit topic");
        }
        const std::function<void(const gz::msgs::Image&)> callback = [this](const gz::msgs::Image& message) {
            std::lock_guard<std::mutex> lock(mutex_);
            latest_ = message;
            ++generation_;
            changed_.notify_all();
        };
        const bool ok = node_.Subscribe(topic_, callback);
        if (!ok) throw std::runtime_error("could not subscribe to Gazebo topic: " + topic_);
    }

    bool read(cv::Mat& bgr, FrameMetadata& metadata, std::string& error) override {
        gz::msgs::Image image;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            const auto previous = consumed_;
            if (!changed_.wait_for(lock, kGazeboFrameTimeout,
                                   [&] { return generation_ != previous; })) {
                error = "Gazebo frame timeout on explicit topic: " + topic_;
                return false;
            }
            image = latest_;
            consumed_ = generation_;
        }
        if (image.width() != kWidth || image.height() != kHeight) {
            error = "Gazebo image must be exactly 640x480";
            return false;
        }
        if (image.pixel_format_type() != gz::msgs::RGB_INT8) {
            error = "Gazebo image must be RGB_INT8, got " +
                    gz::msgs::PixelFormatType_Name(image.pixel_format_type());
            return false;
        }
        const std::uint32_t packed_stride = image.width() * 3U;
        if (image.step() != packed_stride) {
            error = "Gazebo image stride must equal width*3";
            return false;
        }
        const std::size_t expected = static_cast<std::size_t>(image.step()) * image.height();
        if (image.data().size() != expected) {
            error = "Gazebo image payload size does not match stride*height";
            return false;
        }
        cv::Mat rgb(kHeight, kWidth, CV_8UC3,
                    const_cast<char*>(image.data().data()), image.step());
        cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);
        std::int64_t timestamp_ns = -1;
        if (image.has_header() && image.header().has_stamp()) {
            timestamp_ns = image.header().stamp().sec() * 1000000000LL +
                           image.header().stamp().nsec();
        }
        metadata = {"gazebo:" + topic_, "RGB_INT8", image.width(), image.height(),
                    image.step(), timestamp_ns};
        return true;
    }

private:
    std::string topic_;
    gz::transport::Node node_;
    std::mutex mutex_;
    std::condition_variable changed_;
    gz::msgs::Image latest_;
    std::uint64_t generation_ = 0;
    std::uint64_t consumed_ = 0;
};
#endif
}  // namespace

std::unique_ptr<FrameSource> make_jetson_camera_frame_source() {
    return std::make_unique<JetsonCameraFrameSource>();
}

std::unique_ptr<FrameSource> make_imx219_frame_source() {
    return make_jetson_camera_frame_source();
}

std::unique_ptr<FrameSource> make_gazebo_frame_source(const std::string& topic) {
#ifdef ASTRODRONE_HAVE_GZ_FRAME_SOURCE
    return std::make_unique<GazeboFrameSource>(topic);
#else
    (void)topic;
    throw std::runtime_error("this yolo_live build has no Gazebo transport support");
#endif
}
