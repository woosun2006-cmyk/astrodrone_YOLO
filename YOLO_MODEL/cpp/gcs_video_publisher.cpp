#include "gcs_video_publisher.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <stdexcept>
#include <utility>
#include <vector>

namespace {

constexpr std::uint32_t kMagic = 0x31475641;  // AGV1
constexpr std::uint8_t kVersion = 1;
constexpr std::uint8_t kData = 0;
constexpr std::uint8_t kParity = 1;
constexpr std::size_t kChunkPayload = 1100;

#pragma pack(push, 1)
struct PacketHeader {
    std::uint32_t magic;
    std::uint8_t version;
    std::uint8_t type;
    std::uint16_t reserved;
    std::uint64_t frame_sequence;
    std::int64_t timestamp_ns;
    std::uint16_t chunk_index;
    std::uint16_t chunk_count;
    std::uint16_t payload_len;
    std::uint16_t chunk_payload_size;
    std::uint32_t crc32;
};
#pragma pack(pop)

static_assert(sizeof(PacketHeader) == 36, "video packet header changed");

std::uint32_t crc32(const std::uint8_t* data, std::size_t length) {
    std::uint32_t crc = 0xFFFFFFFFU;
    for (std::size_t i = 0; i < length; ++i) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc >> 1U) ^ (0xEDB88320U & -(crc & 1U));
        }
    }
    return ~crc;
}

long env_long(const char* name, long fallback) {
    const char* value = std::getenv(name);
    if (value == nullptr || *value == '\0') return fallback;
    try { return std::stol(value); } catch (...) { return fallback; }
}

double env_double(const char* name, double fallback) {
    const char* value = std::getenv(name);
    if (value == nullptr || *value == '\0') return fallback;
    try { return std::stod(value); } catch (...) { return fallback; }
}

}  // namespace

GcsVideoPublisher::GcsVideoPublisher(EncodedFrameCallback callback)
    : encoded_frame_callback_(std::move(callback)) {
    const char* host = std::getenv("GCS_VIDEO_HOST");
    host_ = host && *host ? host : "127.0.0.1";
    port_ = static_cast<std::uint16_t>(std::clamp(env_long("GCS_VIDEO_PORT", 15560), 1L, 65535L));
    width_ = static_cast<int>(std::clamp(env_long("GCS_VIDEO_WIDTH", 600), 160L, 1920L));
    quality_ = static_cast<int>(std::clamp(env_long("GCS_VIDEO_JPEG_QUALITY", 50), 10L, 95L));
    fps_ = std::max(1.0, std::min(12.0, env_double("GCS_VIDEO_FPS", 6.0)));

    fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    // Video is best-effort. A socket failure must not terminate YOLO or the
    // target/control pipeline; the worker can still serve an optional local
    // preview callback.
    worker_ = std::thread(&GcsVideoPublisher::run, this);
}

GcsVideoPublisher::~GcsVideoPublisher() {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        stopping_ = true;
    }
    condition_.notify_all();
    if (worker_.joinable()) worker_.join();
    if (fd_ >= 0) ::close(fd_);
}

void GcsVideoPublisher::submit(const cv::Mat& bgr, std::uint64_t frame_sequence,
                               std::int64_t timestamp_ns) {
    if (bgr.empty()) return;
    Frame frame;
    try {
        frame = Frame{bgr.clone(), frame_sequence, timestamp_ns};
    } catch (...) {
        return;
    }
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (queue_.size() >= 2) queue_.pop_front();
        queue_.push_back(std::move(frame));
    }
    condition_.notify_one();
}

void GcsVideoPublisher::run() {
    sockaddr_in destination{};
    destination.sin_family = AF_INET;
    destination.sin_port = htons(port_);
    const bool destination_valid =
        inet_pton(AF_INET, host_.c_str(), &destination.sin_addr) == 1;

    const auto minimum_period = std::chrono::duration<double>(1.0 / fps_);
    auto last_sent = std::chrono::steady_clock::now() - minimum_period;
    while (true) {
        Frame frame;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            condition_.wait(lock, [&] { return stopping_ || !queue_.empty(); });
            if (stopping_ && queue_.empty()) return;
            frame = std::move(queue_.back());
            queue_.clear();
        }

        const auto now = std::chrono::steady_clock::now();
        if (now - last_sent < minimum_period) {
            std::this_thread::sleep_for(minimum_period - (now - last_sent));
        }
        last_sent = std::chrono::steady_clock::now();

        cv::Mat resized;
        const double scale = static_cast<double>(width_) / frame.bgr.cols;
        cv::resize(frame.bgr, resized, cv::Size(width_,
                    static_cast<int>(std::round(frame.bgr.rows * scale))));
        std::vector<std::uint8_t> jpeg;
        if (!cv::imencode(".jpg", resized, jpeg,
                          {cv::IMWRITE_JPEG_QUALITY, quality_})) continue;

        if (encoded_frame_callback_) {
            try {
                encoded_frame_callback_(jpeg, frame.sequence, frame.timestamp_ns);
            } catch (...) {
                // A debug/preview consumer is never allowed to kill this
                // worker or affect the inference thread.
            }
        }

        if (fd_ < 0 || !destination_valid) continue;

        const std::uint16_t chunk_count = static_cast<std::uint16_t>(
            (jpeg.size() + kChunkPayload - 1) / kChunkPayload);
        if (chunk_count == 0) continue;
        std::vector<std::uint8_t> parity(kChunkPayload, 0);
        for (std::uint16_t index = 0; index < chunk_count; ++index) {
            const std::size_t offset = static_cast<std::size_t>(index) * kChunkPayload;
            const std::size_t length = std::min(kChunkPayload, jpeg.size() - offset);
            std::vector<std::uint8_t> payload(kChunkPayload, 0);
            std::copy_n(jpeg.data() + offset, length, payload.data());
            for (std::size_t i = 0; i < kChunkPayload; ++i) parity[i] ^= payload[i];

            PacketHeader header{kMagic, kVersion, kData, 0, frame.sequence,
                                frame.timestamp_ns, index, chunk_count,
                                static_cast<std::uint16_t>(length),
                                static_cast<std::uint16_t>(kChunkPayload),
                                crc32(payload.data(), kChunkPayload)};
            std::vector<std::uint8_t> packet(sizeof(header) + payload.size());
            std::memcpy(packet.data(), &header, sizeof(header));
            std::memcpy(packet.data() + sizeof(header), payload.data(), payload.size());
            ::sendto(fd_, packet.data(), packet.size(), 0,
                     reinterpret_cast<sockaddr*>(&destination), sizeof(destination));
        }

        PacketHeader parity_header{kMagic, kVersion, kParity, 0, frame.sequence,
                                   frame.timestamp_ns, 0xFFFF, chunk_count,
                                   static_cast<std::uint16_t>(kChunkPayload),
                                   static_cast<std::uint16_t>(kChunkPayload),
                                   crc32(parity.data(), parity.size())};
        std::vector<std::uint8_t> packet(sizeof(parity_header) + parity.size());
        std::memcpy(packet.data(), &parity_header, sizeof(parity_header));
        std::memcpy(packet.data() + sizeof(parity_header), parity.data(), parity.size());
        ::sendto(fd_, packet.data(), packet.size(), 0,
                 reinterpret_cast<sockaddr*>(&destination), sizeof(destination));
    }
}
