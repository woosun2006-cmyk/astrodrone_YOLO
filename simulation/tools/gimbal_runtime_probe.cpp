#include <gz/msgs/double.pb.h>
#include <gz/msgs/pose_v.pb.h>
#include <gz/transport/Node.hh>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <mutex>
#include <string>
#include <thread>

namespace
{
using Clock = std::chrono::steady_clock;
constexpr double kPi = 3.14159265358979323846;

struct Vec3
{
  double x{0};
  double y{0};
  double z{0};
};

struct Quaternion
{
  double w{1};
  double x{0};
  double y{0};
  double z{0};
};

struct Pose
{
  Vec3 position;
  Quaternion rotation;
};

struct Snapshot
{
  double yaw{0};
  double roll{0};
  double pitch{0};
  Pose cameraLinkWorld;
  Quaternion cameraWorldRotation;
  Vec3 opticalWorld;
  Vec3 opticalBody;
};

struct CommandStats
{
  std::uint64_t count{0};
  double firstLatency{-1};
  double firstValue{0};
  double lastValue{0};
  double minimum{std::numeric_limits<double>::infinity()};
  double maximum{-std::numeric_limits<double>::infinity()};
};

Quaternion Normalize(const Quaternion &_q)
{
  const double length = std::sqrt(_q.w * _q.w + _q.x * _q.x +
      _q.y * _q.y + _q.z * _q.z);
  if (length == 0)
    return {};
  return {_q.w / length, _q.x / length, _q.y / length, _q.z / length};
}

Quaternion Multiply(const Quaternion &_a, const Quaternion &_b)
{
  return Normalize({
      _a.w * _b.w - _a.x * _b.x - _a.y * _b.y - _a.z * _b.z,
      _a.w * _b.x + _a.x * _b.w + _a.y * _b.z - _a.z * _b.y,
      _a.w * _b.y - _a.x * _b.z + _a.y * _b.w + _a.z * _b.x,
      _a.w * _b.z + _a.x * _b.y - _a.y * _b.x + _a.z * _b.w});
}

Quaternion Inverse(const Quaternion &_q)
{
  const auto normalized = Normalize(_q);
  return {normalized.w, -normalized.x, -normalized.y, -normalized.z};
}

Vec3 Rotate(const Quaternion &_q, const Vec3 &_v)
{
  const auto q = Normalize(_q);
  const Vec3 u{q.x, q.y, q.z};
  const double dot = u.x * _v.x + u.y * _v.y + u.z * _v.z;
  const Vec3 cross{u.y * _v.z - u.z * _v.y,
                   u.z * _v.x - u.x * _v.z,
                   u.x * _v.y - u.y * _v.x};
  return {
      2 * dot * u.x + (q.w * q.w - (u.x*u.x + u.y*u.y + u.z*u.z)) * _v.x + 2 * q.w * cross.x,
      2 * dot * u.y + (q.w * q.w - (u.x*u.x + u.y*u.y + u.z*u.z)) * _v.y + 2 * q.w * cross.y,
      2 * dot * u.z + (q.w * q.w - (u.x*u.x + u.y*u.y + u.z*u.z)) * _v.z + 2 * q.w * cross.z};
}

Vec3 Add(const Vec3 &_a, const Vec3 &_b)
{
  return {_a.x + _b.x, _a.y + _b.y, _a.z + _b.z};
}

Quaternion FromRpy(const double _roll, const double _pitch, const double _yaw)
{
  const double cr = std::cos(_roll / 2), sr = std::sin(_roll / 2);
  const double cp = std::cos(_pitch / 2), sp = std::sin(_pitch / 2);
  const double cy = std::cos(_yaw / 2), sy = std::sin(_yaw / 2);
  return Normalize({cr*cp*cy + sr*sp*sy,
                    sr*cp*cy - cr*sp*sy,
                    cr*sp*cy + sr*cp*sy,
                    cr*cp*sy - sr*sp*cy});
}

double NormalizeAngle(double _angle)
{
  while (_angle > kPi) _angle -= 2 * kPi;
  while (_angle < -kPi) _angle += 2 * kPi;
  return _angle;
}

double AxisAngle(const Quaternion &_q, const char _axis)
{
  auto q = Normalize(_q);
  if (q.w < 0)
    q = {-q.w, -q.x, -q.y, -q.z};
  const double component = _axis == 'x' ? q.x : (_axis == 'y' ? q.y : q.z);
  return NormalizeAngle(2 * std::atan2(component, q.w));
}

Pose FromMessage(const gz::msgs::Pose &_pose)
{
  return {{_pose.position().x(), _pose.position().y(), _pose.position().z()},
          Normalize({_pose.orientation().w(), _pose.orientation().x(),
                     _pose.orientation().y(), _pose.orientation().z()})};
}

bool MakeSnapshot(const std::map<std::string, Pose> &_poses, Snapshot &_out)
{
  for (const auto *name : {"iris_with_gimbal", "gimbal", "yaw_link",
                           "roll_link", "pitch_link"})
  {
    if (_poses.find(name) == _poses.end())
      return false;
  }

  const auto &body = _poses.at("iris_with_gimbal");
  const auto &gimbal = _poses.at("gimbal");
  const auto &yaw = _poses.at("yaw_link");
  const auto &roll = _poses.at("roll_link");
  const auto &pitch = _poses.at("pitch_link");

  _out.yaw = AxisAngle(yaw.rotation, 'y');
  _out.roll = AxisAngle(Multiply(Inverse(yaw.rotation), roll.rotation), 'z');
  _out.pitch = AxisAngle(Multiply(Inverse(roll.rotation), pitch.rotation), 'x');

  const auto pitchWorldRotation = Multiply(body.rotation,
      Multiply(gimbal.rotation, pitch.rotation));
  _out.cameraLinkWorld.rotation = pitchWorldRotation;
  _out.cameraLinkWorld.position = Add(body.position, Rotate(body.rotation,
      Add(gimbal.position, Rotate(gimbal.rotation, pitch.position))));

  // Sensor pose from the current gimbal_small_3d SDF. Gazebo camera optical
  // axis is +X in the sensor frame.
  const auto sensorRotation = FromRpy(-1.57, -1.57, 0);
  _out.cameraWorldRotation = Multiply(pitchWorldRotation, sensorRotation);
  _out.opticalWorld = Rotate(_out.cameraWorldRotation, {1, 0, 0});
  _out.opticalBody = Rotate(Inverse(body.rotation), _out.opticalWorld);
  return true;
}

void PrintVector(const std::string &_name, const Vec3 &_value)
{
  std::cout << _name << '=' << _value.x << ',' << _value.y << ',' << _value.z << '\n';
}

void PrintQuaternion(const std::string &_name, const Quaternion &_value)
{
  std::cout << _name << '=' << _value.x << ',' << _value.y << ','
            << _value.z << ',' << _value.w << '\n';
}

void PrintSnapshot(const std::string &_prefix, const Snapshot &_snapshot)
{
  std::cout << _prefix << "_yaw_rad=" << _snapshot.yaw << '\n'
            << _prefix << "_roll_rad=" << _snapshot.roll << '\n'
            << _prefix << "_pitch_rad=" << _snapshot.pitch << '\n'
            << _prefix << "_yaw_deg=" << _snapshot.yaw * 180 / kPi << '\n'
            << _prefix << "_roll_deg=" << _snapshot.roll * 180 / kPi << '\n'
            << _prefix << "_pitch_deg=" << _snapshot.pitch * 180 / kPi << '\n';
  PrintVector(_prefix + "_camera_link_world_position", _snapshot.cameraLinkWorld.position);
  PrintQuaternion(_prefix + "_camera_link_world_quaternion", _snapshot.cameraLinkWorld.rotation);
  PrintQuaternion(_prefix + "_camera_sensor_world_quaternion", _snapshot.cameraWorldRotation);
  PrintVector(_prefix + "_optical_axis_world", _snapshot.opticalWorld);
  PrintVector(_prefix + "_optical_axis_body", _snapshot.opticalBody);
}
}  // namespace

int main(int argc, char **argv)
{
  if (argc != 3)
  {
    std::cerr << "usage: " << argv[0] << " DYNAMIC_POSE_TOPIC DURATION_SEC\n";
    return 2;
  }
  double duration = 0;
  try { duration = std::stod(argv[2]); }
  catch (const std::exception &) { return 2; }
  if (duration <= 0) return 2;

  std::mutex mutex;
  std::condition_variable changed;
  std::map<std::string, Pose> poses;
  Snapshot initial;
  Snapshot latest;
  bool haveInitial = false;
  bool haveLatest = false;
  std::map<std::string, CommandStats> commands;
  const auto start = Clock::now();

  gz::transport::Node node;
  const std::function<void(const gz::msgs::Pose_V &)> poseCallback =
      [&](const gz::msgs::Pose_V &_message)
      {
        std::lock_guard<std::mutex> lock(mutex);
        for (const auto &pose : _message.pose())
          poses[pose.name()] = FromMessage(pose);
        Snapshot next;
        if (MakeSnapshot(poses, next))
        {
          if (!haveInitial)
          {
            initial = next;
            haveInitial = true;
          }
          latest = next;
          haveLatest = true;
          changed.notify_all();
        }
      };
  if (!node.Subscribe(argv[1], poseCallback))
  {
    std::cerr << "could not subscribe to pose topic\n";
    return 3;
  }

  for (const auto &axis : {std::string("roll"), std::string("pitch"), std::string("yaw")})
  {
    commands[axis] = {};
    const std::string topic = "/gimbal/cmd_" + axis;
    const std::function<void(const gz::msgs::Double &)> callback =
        [&, axis](const gz::msgs::Double &_message)
        {
          std::lock_guard<std::mutex> lock(mutex);
          auto &stats = commands[axis];
          if (stats.count == 0)
          {
            stats.firstLatency = std::chrono::duration<double>(Clock::now() - start).count();
            stats.firstValue = _message.data();
          }
          ++stats.count;
          stats.lastValue = _message.data();
          stats.minimum = std::min(stats.minimum, _message.data());
          stats.maximum = std::max(stats.maximum, _message.data());
        };
    if (!node.Subscribe(topic, callback))
    {
      std::cerr << "could not subscribe to command topic " << topic << '\n';
      return 3;
    }
  }

  {
    std::unique_lock<std::mutex> lock(mutex);
    if (!changed.wait_for(lock, std::chrono::seconds(10), [&] { return haveLatest; }))
    {
      std::cerr << "runtime pose timeout\n";
      return 4;
    }
  }
  std::this_thread::sleep_for(std::chrono::duration<double>(duration));

  std::lock_guard<std::mutex> lock(mutex);
  std::cout << std::fixed << std::setprecision(9);
  PrintSnapshot("initial", initial);
  PrintSnapshot("final", latest);
  for (const auto &entry : commands)
  {
    const auto &axis = entry.first;
    const auto &stats = entry.second;
    std::cout << axis << "_command_count=" << stats.count << '\n';
    if (stats.count > 0)
    {
      std::cout << axis << "_first_command_latency_sec=" << stats.firstLatency << '\n'
                << axis << "_first_command_rad=" << stats.firstValue << '\n'
                << axis << "_last_command_rad=" << stats.lastValue << '\n'
                << axis << "_min_command_rad=" << stats.minimum << '\n'
                << axis << "_max_command_rad=" << stats.maximum << '\n';
    }
  }
  return 0;
}
