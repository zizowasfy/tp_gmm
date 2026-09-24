#pragma once
#include <tp_gmm/path_proposal.hpp>
#include <tp_gmm/gmm_constraint_sampler.hpp>
#include <tp_gmm/sampling_math.hpp>
#include <moveit/constraint_samplers/default_constraint_samplers.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <random_numbers/random_numbers.h>
#include <chrono>
#include <algorithm>
#include <limits>
#include <deque>
#include <iomanip>
#include <sstream>


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
  std::shared_ptr<const sampling::ReferencePath> reference;
  // Independently bounded buffers: raw GMM, valid FK, rejected FK, uniform valid, raw GMR.
  std::deque<geometry_msgs::msg::Point> points[5];
  planning_scene::PlanningSceneConstPtr scene;
  Clock::time_point created = Clock::now();
};
inline geometry_msgs::msg::Point point(const Eigen::Vector3d& x)
{
  geometry_msgs::msg::Point p; p.x = x.x(); p.y = x.y(); p.z = x.z(); return p;
}
inline Eigen::Quaterniond quaternion(const geometry_msgs::msg::Quaternion& q) { return {q.w, q.x, q.y, q.z}; }
inline void moments(Metrics& m, const std::string& prefix, const Eigen::Vector3d& x)
{
  m[prefix + "count"]++;
  for (int i = 0; i < 3; ++i)
  {
    m[prefix + "sum_" + std::to_string(i)] += x[i];
    for (int j = 0; j < 3; ++j)
      m[prefix + "outer_" + std::to_string(i) + std::to_string(j)] += x[i] * x[j];
  }
}
}
