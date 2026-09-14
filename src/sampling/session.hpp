#pragma once
#include <tp_gmm/sampling_math.hpp>
#include <tp_gmm/srv/prepare_sampling.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <chrono>
#include <deque>
#include <map>
#include <mutex>

namespace tp_gmm
{
using Clock = std::chrono::steady_clock;
inline double seconds(Clock::time_point start) { return std::chrono::duration<double>(Clock::now() - start).count(); }
using Metrics = std::map<std::string, double>;
struct SamplingSession
{
  srv::PrepareSampling::Request config;
  moveit_msgs::msg::Constraints constraints;
  std::string id;
  std::vector<sampling::Gaussian> gaussians;
  std::vector<double> weights, cutoffs;
  std::mutex mutex;
  Metrics metrics;
  // Four independently bounded buffers: raw Cartesian, valid FK, rejected FK, uniform valid.
  std::deque<geometry_msgs::msg::Point> points[4];
  Clock::time_point created = Clock::now();
};
inline geometry_msgs::msg::Point point(const Eigen::Vector3d& x)
{
  geometry_msgs::msg::Point p; p.x = x.x(); p.y = x.y(); p.z = x.z(); return p;
}
inline Eigen::Quaterniond quaternion(const geometry_msgs::msg::Quaternion& q) { return {q.w, q.x, q.y, q.z}; }

}  // namespace tp_gmm
