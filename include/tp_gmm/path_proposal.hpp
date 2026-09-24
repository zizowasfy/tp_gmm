#pragma once
#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

namespace tp_gmm::sampling
{
// Immutable, ROS-independent arc-length geometry. Time and pose orientations are
// deliberately not interpreted: this is a geometric reference for a fixed tool pose.
class ReferencePath
{
public:
  explicit ReferencePath(const std::vector<Eigen::Vector3d>& input)
  {
    if (input.size() < 2 || input.size() > 4096)
      throw std::invalid_argument("Reference path requires 2..4096 positions");
    for (const auto& x : input)
    {
      if (!x.allFinite()) throw std::invalid_argument("Non-finite reference position");
      if (points_.empty()) { points_.push_back(x); cumulative_.push_back(0.); continue; }
      const double length = (x - points_.back()).norm();
      if (length <= 1e-9) continue;
      points_.push_back(x); cumulative_.push_back(cumulative_.back() + length);
    }
    if (points_.size() < 2 || !std::isfinite(cumulative_.back()))
      throw std::invalid_argument("Reference path has zero or non-finite length");
  }
  double length() const { return cumulative_.back(); }
  const std::vector<Eigen::Vector3d>& points() const { return points_; }
  Eigen::Vector3d at(double fraction) const
  {
    if (!std::isfinite(fraction) || fraction < 0 || fraction > 1)
      throw std::invalid_argument("Path fraction must be in [0,1]");
    if (fraction == 1) return points_.back();
    const double arc = fraction * length();
    const size_t hi = std::min(points_.size()-1, size_t(std::upper_bound(cumulative_.begin(), cumulative_.end(), arc) - cumulative_.begin()));
    const double u = (arc - cumulative_[hi-1]) / (cumulative_[hi] - cumulative_[hi-1]);
    return points_[hi-1] + u * (points_[hi] - points_[hi-1]);
  }
  double distance(const Eigen::Vector3d& x) const
  {
    double best = std::numeric_limits<double>::infinity();
    for (size_t i = 1; i < points_.size(); ++i)
    {
      const Eigen::Vector3d d = points_[i] - points_[i-1];
      const double t = std::clamp((x-points_[i-1]).dot(d)/d.squaredNorm(), 0., 1.);
      best = std::min(best, (x-points_[i-1]-t*d).squaredNorm());
    }
    return std::sqrt(best);
  }
private:
  std::vector<Eigen::Vector3d> points_;
  std::vector<double> cumulative_;
};
}  // namespace tp_gmm::sampling
