#include "sampler.hpp"

namespace tp_gmm
{
bool GMMConstraintSampler::jointSample(moveit::core::RobotState& state, size_t k)
{
  local_["projected_attempts"]++;
  const auto start = Clock::now();
  const Anchor& anchor = anchors_[k][std::uniform_int_distribution<size_t>(0, anchors_[k].size() - 1)(rng_)];
  Eigen::Vector3d z;
  if (!sampling::truncatedNormal(rng_, normal_, session_->cutoffs[k], z)) return false;
  const Eigen::Vector3d linear_target = session_->gaussians[k].mean + session_->gaussians[k].factor * z;
  moments(local_, "component_" + std::to_string(k) + "_projected_linear_", linear_target);
  record(0, linear_target);
  Eigen::VectorXd u(anchor.null_basis.cols());
  for (int i = 0; i < u.size(); ++i) u[i] = normal_(rng_);
  const Eigen::VectorXd delta = anchor.task_factor * z + session_->config.nullspace_stddev * anchor.null_basis * u;
  local_["draw_s"] += seconds(start);
  if (delta.norm() > session_->config.max_joint_delta) { local_["trust_region_rejections"]++; return false; }
  Eigen::VectorXd q = Eigen::Map<const Eigen::VectorXd>(anchor.q.data(), anchor.q.size()) + delta;
  state.setJointGroupPositions(jmg_, q);
  if (!state.satisfiesBounds(jmg_)) { local_["bounds_rejections"]++; return false; }
  const auto fk_start = Clock::now();
  const Eigen::Vector3d x = fk(state);
  const Eigen::Vector3d linear = session_->gaussians[k].mean + anchor.jacobian * delta;
  const double residual = (x - linear).norm();
  local_["fk_mapping_s"] += seconds(fk_start);
  local_["linearization_residual_sum_m"] += residual;
  local_["linearization_checks"]++;
  moments(local_, "component_" + std::to_string(k) + "_projected_fk_", x);
  if (residual > session_->config.linearization_tolerance ||
      sampling::mahalanobisSquared(session_->gaussians[k], x) > session_->cutoffs[k] * session_->cutoffs[k])
  { local_["projection_support_rejections"]++; record(2, x); return false; }
  return true;
}

void GMMConstraintSampler::buildAnchors()
{
  moveit::core::RobotState seed(scene_->getCurrentState());
  for (size_t k = 0; k < anchors_.size(); ++k)
  {
    for (unsigned attempt = 0; attempt < session_->config.anchor_attempts && anchors_[k].size() < session_->config.branches; ++attempt)
    {
      if (attempt) seed.setToRandomPositions(jmg_, *seed_rng_);
      if (!ik(seed, session_->gaussians[k].mean, true)) continue;
      std::vector<double> q; seed.copyJointGroupPositions(jmg_, q);
      bool duplicate = false;
      for (const auto& a : anchors_[k])
        if ((Eigen::Map<const Eigen::VectorXd>(q.data(), q.size()) - Eigen::Map<const Eigen::VectorXd>(a.q.data(), a.q.size())).norm() < 0.1) duplicate = true;
      if (duplicate) continue;
      const auto start = Clock::now();
      Eigen::MatrixXd full;
      if (!seed.getJacobian(jmg_, link_, Eigen::Vector3d::Zero(), full) || full.rows() != 6 || full.cols() != int(jmg_->getVariableCount())) continue;
      // MoveIt returns the Jacobian in the group-root frame. Rotate it to the model frame.
      const auto* root = jmg_->getJointModels().front()->getParentLinkModel();
      Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
      if (root) rotation = seed.getGlobalLinkTransform(root).linear();
      Eigen::MatrixXd jacobian = model_from_gmm_.linear().transpose() * rotation * full.topRows(3);
      try
      {
        const auto projection = sampling::project(jacobian);
        anchors_[k].push_back({q, projection.inverse * session_->gaussians[k].factor, projection.null_basis, jacobian});
        local_["anchors"]++;
      }
      catch (const std::invalid_argument&) { local_["singular_anchors"]++; }
      local_["jacobian_setup_s"] += seconds(start);
    }
    if (anchors_[k].empty()) local_["unanchored_components"]++;
  }
}
}  // namespace tp_gmm
