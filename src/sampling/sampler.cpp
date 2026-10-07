#include "sampler.hpp"

namespace tp_gmm
{
GMMConstraintSampler::GMMConstraintSampler(const planning_scene::PlanningSceneConstPtr& scene, const std::string& group,
                    std::shared_ptr<SamplingSession> session)
  : ConstraintSampler(scene, group), session_(std::move(session)), rng_(session_->config.seed),
    seed_rng_(std::make_unique<random_numbers::RandomNumberGenerator>(session_->config.seed)), mixture_(session_->weights.begin(), session_->weights.end()),
    constraint_set_(scene->getRobotModel())
{
  std::lock_guard<std::mutex> lock(session_->mutex);
  const auto instance = static_cast<unsigned>(session_->metrics["allocated_instances"]++);
  rng_.seed(session_->config.seed + 104729u * instance);
  seed_rng_ = std::make_unique<random_numbers::RandomNumberGenerator>(session_->config.seed + 104729u * instance);
}
const std::string& GMMConstraintSampler::getName() const
{ static const std::string name = "GMMConstraintSampler"; return name; }

bool GMMConstraintSampler::configure(const moveit_msgs::msg::Constraints& constraints)
{
  is_valid_ = false;
  const auto start = Clock::now();
  if (!jmg_ || constraints != session_->constraints || jmg_->getName() != session_->config.group_name)
    return false;
  link_ = scene_->getRobotModel()->getLinkModel(session_->config.link_name);
  if (!link_ || !jmg_->isChain() || !jmg_->getUpdatedLinkModelsSet().count(link_)) return false;
  // Keep Gaussians in their fixed message frame; transform IK/FK and the Jacobian consistently.
  const auto& frame = session_->config.model.header.frame_id;
  if (!scene_->knowsFrameTransform(frame)) throw std::runtime_error("Unknown GMM frame: " + frame);
  if (const auto* frame_link = scene_->getRobotModel()->getLinkModel(frame))
  {
    for (const auto* parent = frame_link; parent; parent = parent->getParentLinkModel())
      if (parent->getParentJointModel()->getType() != moveit::core::JointModel::FIXED)
        throw std::runtime_error("GMM frame must be fixed relative to the model frame");
  }
  else if (!scene_->getTransforms().isFixedFrame(frame))
    throw std::runtime_error("GMM frame must be a known fixed frame");
  model_from_gmm_ = scene_->getFrameTransform(frame);
  if (!constraint_set_.add(constraints, scene_->getTransforms())) return false;
  fallback_ = std::make_shared<constraint_samplers::IKConstraintSampler>(scene_, jmg_->getName());
  if (!fallback_->configure(constraints)) return false;
  anchors_.resize(session_->gaussians.size());
  if (session_->config.mode == "joint_projected") buildAnchors();
  local_["setup_s"] += seconds(start);
  local_["instances"] = 1;
  is_valid_ = true;
  {
    std::lock_guard<std::mutex> lock(session_->mutex);
    // Allocation is against the request's planning-scene snapshot, not a global/latest scene.
    session_->scene = scene_;
  }
  flush();
  return true;
}

bool GMMConstraintSampler::sample(moveit::core::RobotState& state, const moveit::core::RobotState& reference,
            unsigned int max_attempts)
{
  const auto start = Clock::now();
  local_["sampler_calls"]++;
  // Copy before touching output: OMPL may pass the same object as state and reference.
  const moveit::core::RobotState reference_copy(reference);
  bool success = false;
  for (unsigned attempt = 0; is_valid_ && attempt < max_attempts; ++attempt)
  {
    moveit::core::RobotState candidate(reference_copy);
    local_["attempts"]++;
    if (session_->config.mode == "uniform" || uniform_(rng_) < session_->config.uniform_fraction)
    {
      local_["uniform_attempts"]++;
      const auto begin = Clock::now();
      bool ok = fallback_->sample(candidate, reference_copy, 1);
      local_["uniform_sampler_s"] += seconds(begin);
      if (ok && valid(candidate)) { state = candidate; local_["uniform_valid"]++; record(3, fk(candidate)); success = true; }
    }
    else if (session_->config.proposal == "gmr_path" ||
             (session_->config.proposal == "hybrid" && uniform_(rng_) < session_->config.gmr_fraction))
    {
      if (gmrSample(candidate))
      {
        const Eigen::Vector3d x = fk(candidate);
        if (valid(candidate))
        { state = candidate; success = true; record(1, x); local_["gmr_valid"]++; local_["cartesian_valid"]++; }
        else record(2, x);
      }
    }
    else
    {
      local_["gmm_attempts"]++;
      const size_t k = mixture_(rng_);
      local_["component_" + std::to_string(k) + "_selected"]++;
      bool projected = session_->config.mode == "joint_projected" &&
                       uniform_(rng_) >= session_->config.cartesian_fraction && !anchors_[k].empty();
      if (session_->config.mode == "joint_projected" && anchors_[k].empty())
      {
        if (session_->config.cartesian_fraction == 0.)
        { local_["missing_anchor_rejections"]++; continue; }
        local_["missing_anchor_fallbacks"]++;
      }
      bool ok = projected ? jointSample(candidate, k) : cartesianSample(candidate, k);
      if (ok)
      {
        const Eigen::Vector3d x = fk(candidate);
        if (valid(candidate))
        {
          state = candidate; success = true; record(1, x);
          local_[projected ? "projected_valid" : "cartesian_valid"]++;
          moments(local_, "component_" + std::to_string(k) + "_valid_", x);
          local_["valid_mahalanobis_squared_sum"] += sampling::mahalanobisSquared(session_->gaussians[k], x);
        }
        else record(2, x);
      }
    }
    if (success) { local_["valid_samples"]++; break; }
  }
  if (!success) local_["failed_calls"]++;
  local_["sampling_s"] += seconds(start);
  flush();
  return success;
}

Eigen::Vector3d GMMConstraintSampler::fk(moveit::core::RobotState& state) const
{ state.update(); return model_from_gmm_.inverse() * state.getGlobalLinkTransform(link_).translation(); }

bool GMMConstraintSampler::valid(moveit::core::RobotState& state)
{
  const auto start = Clock::now();
  state.update();
  bool ok = true;
  if (!state.satisfiesBounds(jmg_)) { local_["bounds_rejections"]++; ok = false; }
  else if (!constraint_set_.decide(state).satisfied) { local_["constraint_rejections"]++; ok = false; }
  else if (!scene_->isStateFeasible(state, false)) { local_["feasibility_rejections"]++; ok = false; }
  else
  {
    local_["collision_checks"]++;
    if (scene_->isStateColliding(state, jmg_->getName(), false)) { local_["collision_rejections"]++; ok = false; }
  }
  if (ok && group_state_validity_callback_)
  {
    std::vector<double> q; state.copyJointGroupPositions(jmg_, q);
    if (!group_state_validity_callback_(&state, jmg_, q.data())) { local_["callback_rejections"]++; ok = false; }
  }
  local_["validity_s"] += seconds(start);
  return ok;
}

bool GMMConstraintSampler::ik(moveit::core::RobotState& state, const Eigen::Vector3d& x, bool anchor)
{
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.linear() = quaternion(session_->config.orientation).toRotationMatrix();
  pose.translation() = x;
  const auto start = Clock::now();
  local_[anchor ? "anchor_ik_calls" : "online_ik_calls"]++;
  bool ok = state.setFromIK(jmg_, model_from_gmm_ * pose, link_->getName(), session_->config.ik_timeout);
  local_[anchor ? "anchor_ik_s" : "online_ik_s"] += seconds(start);
  if (ok && state.satisfiesBounds(jmg_) && (fk(state) - x).norm() <= 1e-4)
  { local_[anchor ? "anchor_ik_success" : "online_ik_success"]++; return true; }
  return false;
}

void GMMConstraintSampler::record(int stage, const Eigen::Vector3d& x)
{
  if (!session_->config.visualize) return;
  std::lock_guard<std::mutex> lock(session_->mutex);
  auto& buffer = session_->points[stage];
  if (buffer.size() == 1000) buffer.pop_front();
  buffer.push_back(point(x));
}

void GMMConstraintSampler::flush()
{
  std::lock_guard<std::mutex> lock(session_->mutex);
  for (const auto& [key, value] : local_) session_->metrics[key] += value;
  local_.clear();
}
}  // namespace tp_gmm
