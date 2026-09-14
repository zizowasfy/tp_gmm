#pragma once
#include <tp_gmm/gmm_constraint_sampler.hpp>
#include <tp_gmm/sampling_math.hpp>
#include <moveit/constraint_samplers/default_constraint_samplers.hpp>
#include <random_numbers/random_numbers.h>
#include "session.hpp"

namespace tp_gmm
{
class GMMConstraintSampler final : public constraint_samplers::ConstraintSampler
{
  struct Anchor { std::vector<double> q; Eigen::MatrixXd task_factor, null_basis, jacobian; };
public:
  GMMConstraintSampler(const planning_scene::PlanningSceneConstPtr& scene, const std::string& group,
                       std::shared_ptr<SamplingSession> session);
  const std::string& getName() const override;
  bool configure(const moveit_msgs::msg::Constraints& constraints) override;
  bool sample(moveit::core::RobotState& state, const moveit::core::RobotState& reference, unsigned int max_attempts) override;
private:
  Eigen::Vector3d fk(moveit::core::RobotState& state) const;
  bool valid(moveit::core::RobotState& state);
  bool ik(moveit::core::RobotState& state, const Eigen::Vector3d& x, bool anchor);
  bool cartesianSample(moveit::core::RobotState& state, size_t k);
  bool jointSample(moveit::core::RobotState& state, size_t k);
  void buildAnchors();
  void record(int stage, const Eigen::Vector3d& x);
  void flush();
  std::shared_ptr<SamplingSession> session_;
  std::mt19937 rng_;
  std::unique_ptr<random_numbers::RandomNumberGenerator> seed_rng_;
  std::normal_distribution<double> normal_{0, 1};
  std::uniform_real_distribution<double> uniform_{0, 1};
  std::discrete_distribution<size_t> mixture_;
  kinematic_constraints::KinematicConstraintSet constraint_set_;
  const moveit::core::LinkModel* link_{nullptr};
  Eigen::Isometry3d model_from_gmm_ = Eigen::Isometry3d::Identity();
  constraint_samplers::ConstraintSamplerPtr fallback_;
  std::vector<std::vector<Anchor>> anchors_;
  Metrics local_;
};
}  // namespace tp_gmm
