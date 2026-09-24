#include "sampler.hpp"

namespace tp_gmm
{
bool GMMConstraintSampler::cartesianSample(moveit::core::RobotState& state, size_t k)
{
  local_["cartesian_attempts"]++;
  const auto start = Clock::now();
  Eigen::Vector3d z;
  if (!sampling::truncatedNormal(rng_, normal_, session_->cutoffs[k], z)) return false;
  Eigen::Vector3d x = session_->gaussians[k].mean + session_->gaussians[k].factor * z;
  local_["draw_s"] += seconds(start);
  moments(local_, "component_" + std::to_string(k) + "_proposal_", x);
  local_["proposal_mahalanobis_squared_sum"] += z.squaredNorm();
  record(0, x);
  state.setToRandomPositions(jmg_, *seed_rng_);
  return ik(state, x, false);
}
}  // namespace tp_gmm
