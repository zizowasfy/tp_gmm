#include "sampler.hpp"

namespace tp_gmm
{
bool GMMConstraintSampler::gmrSample(moveit::core::RobotState& state)
{
  local_["gmr_attempts"]++;
  local_["cartesian_attempts"]++;
  const auto start = Clock::now();
  const double fraction = uniform_(rng_);
  Eigen::Vector3d z = Eigen::Vector3d::Zero();
  if (session_->config.gmr_stddev > 0 &&
      !sampling::truncatedNormal(rng_, normal_, session_->config.gmr_cutoff, z)) return false;
  const Eigen::Vector3d x = session_->reference->at(fraction) + session_->config.gmr_stddev * z;
  local_["draw_s"] += seconds(start);
  local_["gmr_draws"]++;
  local_["gmr_fraction_sum"] += fraction;
  local_["gmr_offset_squared_sum_m2"] += session_->config.gmr_stddev * session_->config.gmr_stddev * z.squaredNorm();
  local_["gmr_bin_" + std::to_string(std::min(9, int(fraction*10))) + "_draws"]++;
  record(4, x);
  // Same randomized IK seed policy as ordinary GMM Cartesian proposals.
  state.setToRandomPositions(jmg_, *seed_rng_);
  return ik(state, x, false);
}
}  // namespace tp_gmm
