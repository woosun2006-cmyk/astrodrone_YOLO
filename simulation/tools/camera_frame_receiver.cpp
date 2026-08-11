#include <gz/msgs/image.pb.h>
#include <gz/transport/Node.hh>

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <ctime>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>

namespace
{
using Clock = std::chrono::steady_clock;

struct State
{
  std::mutex mutex;
  std::condition_variable changed;
  std::uint64_t frameCount{0};
  gz::msgs::Image firstFrame;
  Clock::time_point subscribeTime;
  Clock::time_point firstArrival;
  Clock::time_point lastArrival;
  std::chrono::system_clock::time_point firstWallTime;
  bool received{false};
};

std::string Iso8601(const std::chrono::system_clock::time_point &_time)
{
  const auto time = std::chrono::system_clock::to_time_t(_time);
  std::tm tm{};
  localtime_r(&time, &tm);
  const auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(
      _time.time_since_epoch()) % 1000;
  std::ostringstream output;
  output << std::put_time(&tm, "%Y-%m-%dT%H:%M:%S") << '.'
         << std::setw(3) << std::setfill('0') << millis.count();
  return output.str();
}

bool BytesPerPixel(const gz::msgs::PixelFormatType _format,
                   std::size_t &_bytes)
{
  switch (_format)
  {
    case gz::msgs::L_INT8:
      _bytes = 1;
      return true;
    case gz::msgs::RGB_INT8:
    case gz::msgs::BGR_INT8:
      _bytes = 3;
      return true;
    case gz::msgs::RGBA_INT8:
    case gz::msgs::BGRA_INT8:
      _bytes = 4;
      return true;
    default:
      return false;
  }
}

bool SavePpm(const gz::msgs::Image &_image, const std::string &_path,
             double &_mean, unsigned int &_minimum, unsigned int &_maximum,
             std::string &_error)
{
  std::size_t bytesPerPixel = 0;
  if (!BytesPerPixel(_image.pixel_format_type(), bytesPerPixel))
  {
    _error = "unsupported pixel format: " +
        gz::msgs::PixelFormatType_Name(_image.pixel_format_type());
    return false;
  }
  if (_image.width() == 0 || _image.height() == 0)
  {
    _error = "zero image dimension";
    return false;
  }

  const std::size_t packedStep =
      static_cast<std::size_t>(_image.width()) * bytesPerPixel;
  const std::size_t step = _image.step() == 0 ? packedStep : _image.step();
  if (step < packedStep || _image.data().size() < step * _image.height())
  {
    _error = "image payload is smaller than dimensions and row step require";
    return false;
  }

  std::ofstream output(_path, std::ios::binary | std::ios::trunc);
  if (!output)
  {
    _error = "could not open output file";
    return false;
  }
  output << "P6\n" << _image.width() << ' ' << _image.height() << "\n255\n";

  std::uint64_t sum = 0;
  std::uint64_t samples = 0;
  _minimum = std::numeric_limits<unsigned int>::max();
  _maximum = 0;
  const auto *data = reinterpret_cast<const unsigned char *>(_image.data().data());
  for (std::uint32_t y = 0; y < _image.height(); ++y)
  {
    const auto *row = data + static_cast<std::size_t>(y) * step;
    for (std::uint32_t x = 0; x < _image.width(); ++x)
    {
      const auto *pixel = row + static_cast<std::size_t>(x) * bytesPerPixel;
      unsigned char rgb[3]{};
      switch (_image.pixel_format_type())
      {
        case gz::msgs::L_INT8:
          rgb[0] = rgb[1] = rgb[2] = pixel[0];
          break;
        case gz::msgs::RGB_INT8:
        case gz::msgs::RGBA_INT8:
          rgb[0] = pixel[0]; rgb[1] = pixel[1]; rgb[2] = pixel[2];
          break;
        case gz::msgs::BGR_INT8:
        case gz::msgs::BGRA_INT8:
          rgb[0] = pixel[2]; rgb[1] = pixel[1]; rgb[2] = pixel[0];
          break;
        default:
          _error = "internal pixel format conversion error";
          return false;
      }
      output.write(reinterpret_cast<const char *>(rgb), 3);
      for (const auto value : rgb)
      {
        sum += value;
        ++samples;
        _minimum = std::min(_minimum, static_cast<unsigned int>(value));
        _maximum = std::max(_maximum, static_cast<unsigned int>(value));
      }
    }
  }
  output.close();
  if (!output)
  {
    _error = "failed while writing PPM data";
    return false;
  }
  _mean = samples == 0 ? 0.0 : static_cast<double>(sum) / samples;
  return true;
}
}  // namespace

int main(int argc, char **argv)
{
  if (argc != 5)
  {
    std::cerr << "usage: " << argv[0]
              << " TOPIC OUTPUT.ppm FIRST_FRAME_TIMEOUT_SEC SAMPLE_SEC\n";
    return 2;
  }

  const std::string topic = argv[1];
  const std::string outputPath = argv[2];
  double timeoutSeconds = 0;
  double sampleSeconds = 0;
  try
  {
    timeoutSeconds = std::stod(argv[3]);
    sampleSeconds = std::stod(argv[4]);
  }
  catch (const std::exception &)
  {
    std::cerr << "timeout and sample duration must be numeric\n";
    return 2;
  }
  if (timeoutSeconds <= 0 || sampleSeconds < 0)
  {
    std::cerr << "timeout must be positive and sample duration non-negative\n";
    return 2;
  }

  State state;
  state.subscribeTime = Clock::now();
  gz::transport::Node node;
  const std::function<void(const gz::msgs::Image &)> callback =
      [&state](const gz::msgs::Image &_message)
      {
        const auto now = Clock::now();
        std::lock_guard<std::mutex> lock(state.mutex);
        ++state.frameCount;
        state.lastArrival = now;
        if (!state.received)
        {
          state.firstFrame = _message;
          state.firstArrival = now;
          state.firstWallTime = std::chrono::system_clock::now();
          state.received = true;
        }
        state.changed.notify_all();
      };
  const bool subscribed = node.Subscribe(topic, callback);
  if (!subscribed)
  {
    std::cerr << "could not subscribe to topic: " << topic << '\n';
    return 3;
  }

  {
    std::unique_lock<std::mutex> lock(state.mutex);
    if (!state.changed.wait_for(lock,
        std::chrono::duration<double>(timeoutSeconds),
        [&state] { return state.received; }))
    {
      std::cerr << "frame timeout after " << timeoutSeconds << " seconds\n";
      return 4;
    }
  }
  std::this_thread::sleep_for(std::chrono::duration<double>(sampleSeconds));

  gz::msgs::Image image;
  std::uint64_t frameCount = 0;
  Clock::time_point subscribeTime;
  Clock::time_point firstArrival;
  Clock::time_point lastArrival;
  std::chrono::system_clock::time_point firstWallTime;
  {
    std::lock_guard<std::mutex> lock(state.mutex);
    image = state.firstFrame;
    frameCount = state.frameCount;
    subscribeTime = state.subscribeTime;
    firstArrival = state.firstArrival;
    lastArrival = state.lastArrival;
    firstWallTime = state.firstWallTime;
  }

  double pixelMean = 0;
  unsigned int pixelMinimum = 0;
  unsigned int pixelMaximum = 0;
  std::string error;
  if (!SavePpm(image, outputPath, pixelMean, pixelMinimum, pixelMaximum, error))
  {
    std::cerr << error << '\n';
    return 5;
  }

  const double firstLatencyMs =
      std::chrono::duration<double, std::milli>(firstArrival - subscribeTime).count();
  const double receiveSpan =
      std::chrono::duration<double>(lastArrival - firstArrival).count();
  const double fps = frameCount > 1 && receiveSpan > 0
      ? static_cast<double>(frameCount - 1) / receiveSpan : 0.0;
  std::ostringstream messageStamp;
  if (image.has_header() && image.header().has_stamp())
  {
    messageStamp << image.header().stamp().sec() << '.' << std::setw(9)
                 << std::setfill('0') << image.header().stamp().nsec();
  }
  else
  {
    messageStamp << "absent";
  }

  std::cout << "topic=" << topic << '\n'
            << "message_type=" << image.GetTypeName() << '\n'
            << "width=" << image.width() << '\n'
            << "height=" << image.height() << '\n'
            << "pixel_format="
            << gz::msgs::PixelFormatType_Name(image.pixel_format_type()) << '\n'
            << "row_step=" << image.step() << '\n'
            << "payload_bytes=" << image.data().size() << '\n'
            << "message_timestamp=" << messageStamp.str() << '\n'
            << "first_frame_wall_time=" << Iso8601(firstWallTime) << '\n'
            << "first_frame_latency_ms=" << std::fixed << std::setprecision(3)
            << firstLatencyMs << '\n'
            << "frame_count=" << frameCount << '\n'
            << "receive_span_sec=" << receiveSpan << '\n'
            << "fps=" << fps << '\n'
            << "pixel_min=" << pixelMinimum << '\n'
            << "pixel_max=" << pixelMaximum << '\n'
            << "pixel_mean=" << pixelMean << '\n'
            << "all_black=" << (pixelMaximum == 0 ? "true" : "false") << '\n'
            << "sample_image=" << outputPath << '\n';
  return 0;
}
