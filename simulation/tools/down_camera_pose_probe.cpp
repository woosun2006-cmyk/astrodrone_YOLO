#include <gz/msgs/pose_v.pb.h>
#include <gz/transport/Node.hh>

#include <chrono>
#include <cmath>
#include <condition_variable>
#include <functional>
#include <iomanip>
#include <iostream>
#include <map>
#include <mutex>
#include <string>
#include <thread>

namespace
{
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
  Pose linkWorld;
  Quaternion sensorWorld;
  Vec3 opticalBody;
  Vec3 opticalWorld;
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
  const auto q = Normalize(_q);
  return {q.w, -q.x, -q.y, -q.z};
}

Vec3 Rotate(const Quaternion &_q, const Vec3 &_v)
{
  const auto q = Normalize(_q);
  const Vec3 u{q.x, q.y, q.z};
  const double dot = u.x * _v.x + u.y * _v.y + u.z * _v.z;
  const Vec3 cross{u.y * _v.z - u.z * _v.y,
                   u.z * _v.x - u.x * _v.z,
                   u.x * _v.y - u.y * _v.x};
  const double uu = u.x*u.x + u.y*u.y + u.z*u.z;
  return {2*dot*u.x + (q.w*q.w-uu)*_v.x + 2*q.w*cross.x,
          2*dot*u.y + (q.w*q.w-uu)*_v.y + 2*q.w*cross.y,
          2*dot*u.z + (q.w*q.w-uu)*_v.z + 2*q.w*cross.z};
}

Vec3 Add(const Vec3 &_a, const Vec3 &_b)
{
  return {_a.x + _b.x, _a.y + _b.y, _a.z + _b.z};
}

Quaternion FromRpy(const double _roll, const double _pitch, const double _yaw)
{
  const double cr = std::cos(_roll/2), sr = std::sin(_roll/2);
  const double cp = std::cos(_pitch/2), sp = std::sin(_pitch/2);
  const double cy = std::cos(_yaw/2), sy = std::sin(_yaw/2);
  return Normalize({cr*cp*cy + sr*sp*sy,
                    sr*cp*cy - cr*sp*sy,
                    cr*sp*cy + sr*cp*sy,
                    cr*cp*sy - sr*sp*cy});
}

Pose FromMessage(const gz::msgs::Pose &_pose)
{
  return {{_pose.position().x(), _pose.position().y(), _pose.position().z()},
          Normalize({_pose.orientation().w(), _pose.orientation().x(),
                     _pose.orientation().y(), _pose.orientation().z()})};
}

bool MakeSnapshot(const std::map<std::string, Pose> &_poses,
                  const std::string &_outerName,
                  const std::string &_bodyName,
                  const std::string &_linkName,
                  const Quaternion &_sensorRotation,
                  Snapshot &_snapshot)
{
  if (_poses.count(_outerName) == 0 || _poses.count(_bodyName) == 0 ||
      _poses.count(_linkName) == 0)
    return false;

  const auto &outer = _poses.at(_outerName);
  const auto &body = _poses.at(_bodyName);
  const auto &link = _poses.at(_linkName);
  const auto bodyWorldRotation = Multiply(outer.rotation, body.rotation);
  _snapshot.linkWorld.rotation = Multiply(bodyWorldRotation, link.rotation);
  _snapshot.linkWorld.position = Add(outer.position, Rotate(outer.rotation,
      Add(body.position, Rotate(body.rotation, link.position))));
  _snapshot.sensorWorld = Multiply(_snapshot.linkWorld.rotation, _sensorRotation);
  _snapshot.opticalWorld = Rotate(_snapshot.sensorWorld, {1, 0, 0});
  _snapshot.opticalBody = Rotate(Inverse(bodyWorldRotation), _snapshot.opticalWorld);
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
  PrintVector(_prefix + "_camera_link_world_position", _snapshot.linkWorld.position);
  PrintQuaternion(_prefix + "_camera_link_world_quaternion", _snapshot.linkWorld.rotation);
  PrintQuaternion(_prefix + "_camera_sensor_world_quaternion", _snapshot.sensorWorld);
  PrintVector(_prefix + "_optical_axis_body", _snapshot.opticalBody);
  PrintVector(_prefix + "_optical_axis_world", _snapshot.opticalWorld);
}
}  // namespace

int main(int argc, char **argv)
{
  if (argc != 9)
  {
    std::cerr << "usage: " << argv[0]
              << " POSE_TOPIC OUTER_MODEL BODY_MODEL CAMERA_LINK"
              << " SENSOR_ROLL SENSOR_PITCH SENSOR_YAW DURATION_SEC\n";
    return 2;
  }

  double roll = 0, pitch = 0, yaw = 0, duration = 0;
  try
  {
    roll = std::stod(argv[5]);
    pitch = std::stod(argv[6]);
    yaw = std::stod(argv[7]);
    duration = std::stod(argv[8]);
  }
  catch (const std::exception &)
  {
    return 2;
  }
  if (duration < 0)
    return 2;

  const std::string outerName = argv[2];
  const std::string bodyName = argv[3];
  const std::string linkName = argv[4];
  const auto sensorRotation = FromRpy(roll, pitch, yaw);
  std::mutex mutex;
  std::condition_variable changed;
  std::map<std::string, Pose> poses;
  Snapshot initial;
  Snapshot latest;
  bool haveInitial = false;
  bool haveLatest = false;

  gz::transport::Node node;
  const std::function<void(const gz::msgs::Pose_V &)> callback =
      [&](const gz::msgs::Pose_V &_message)
      {
        std::lock_guard<std::mutex> lock(mutex);
        for (const auto &pose : _message.pose())
          poses[pose.name()] = FromMessage(pose);
        Snapshot next;
        if (MakeSnapshot(poses, outerName, bodyName, linkName, sensorRotation, next))
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
  if (!node.Subscribe(argv[1], callback))
  {
    std::cerr << "could not subscribe to pose topic\n";
    return 3;
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
  return 0;
}
