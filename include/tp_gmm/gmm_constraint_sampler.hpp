#pragma once

#include <moveit/constraint_samplers/constraint_sampler.hpp>
#include <moveit/constraint_samplers/constraint_sampler_allocator.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <tp_gmm/msg/gaussian_mixture.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/string.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <rclcpp/rclcpp.hpp>

#include <Eigen/Dense>
#include <vector>
#include <string>
#include <memory>
#include <random>
#include <mutex>
#include <atomic>
#include <thread>

namespace tp_gmm
{

enum class SamplingMode
{
  CARTESIAN_IK = 0,     // Approach 1: Cartesian GMM + warm-started IK
  JOINT_PROJECTED = 1,  // Approach 2: Direct Joint-Space GMM (zero runtime IK)
  UNIFORM_BOX = 2       // Approach 3: Baseline Uniform Bounding Box corridor
};

struct GMMComponent
{
  Eigen::Vector3d mean;              // 3D Cartesian position
  Eigen::Matrix3d cov_cartesian;     // 3x3 covariance in task space
  Eigen::Matrix3d cholesky_L_x;      // Cholesky factor L of cov_cartesian (Sigma = L * L^T)
  Eigen::Quaterniond orientation;    // Nominal orientation

  std::vector<double> nominal_joint_positions; // Nominal joint configuration (q_bar)

  // Approach 2: Joint-space projected covariance and Cholesky factor
  Eigen::MatrixXd cov_joint;         // n x n joint covariance
  Eigen::MatrixXd cholesky_L_q;      // n x n Cholesky factor
};

struct BoundingBoxData
{
  Eigen::Vector3d center;
  Eigen::Quaterniond orientation;
  Eigen::Vector3d half_extents;
  double volume{0.0};
};

class GMMConstraintSampler : public constraint_samplers::ConstraintSampler
{
public:
  GMMConstraintSampler(const planning_scene::PlanningSceneConstPtr& scene,
                       const std::string& group_name,
                       tp_gmm::msg::GaussianMixture::ConstSharedPtr gmm_msg,
                       SamplingMode mode = SamplingMode::CARTESIAN_IK,
                       rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr viz_pub = nullptr,
                       rclcpp::Publisher<std_msgs::msg::String>::SharedPtr stats_pub = nullptr);

  ~GMMConstraintSampler() override;

  bool configure(const moveit_msgs::msg::Constraints& constr) override;

  bool sample(moveit::core::RobotState& state,
              const moveit::core::RobotState& reference_state,
              unsigned int max_attempts) override;

  const std::string& getName() const override
  {
    return sampler_name_;
  }

  SamplingMode getSamplingMode() const
  {
    return mode_;
  }

  void setSamplingMode(SamplingMode mode)
  {
    mode_ = mode;
  }

  size_t getSamplesDrawn() const { return samples_drawn_; }
  size_t getSamplesAccepted() const { return samples_accepted_; }

  void publishSamplingStats(bool is_final = false);

private:
  bool parseGMMMessage(const tp_gmm::msg::GaussianMixture& msg);
  void precomputeNominalJointStatesAndJacobians();
  void publishVisualSample(const Eigen::Vector3d& point, bool accepted);

  std::string sampler_name_{"GMMConstraintSampler"};
  SamplingMode mode_{SamplingMode::CARTESIAN_IK};
  tp_gmm::msg::GaussianMixture::ConstSharedPtr gmm_msg_;

  std::vector<GMMComponent> components_;
  std::vector<double> weights_;
  std::discrete_distribution<int> component_dist_;

  // Approach 3: Bounding boxes for uniform corridor sampling
  std::vector<BoundingBoxData> uniform_boxes_;
  std::vector<double> box_weights_;
  std::discrete_distribution<int> box_dist_;

  const moveit::core::LinkModel* ee_link_{nullptr};
  Eigen::Quaterniond default_orientation_{0.0, 1.0, 0.0, 0.0}; // Downward orientation (w=0, x=1, y=0, z=0)
  double ik_timeout_{0.005}; // 5ms warm-started IK timeout

  std::mt19937 rng_;
  std::normal_distribution<double> normal_dist_{0.0, 1.0};

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr viz_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr stats_pub_;
  mutable std::mutex viz_mutex_;
  visualization_msgs::msg::Marker sample_marker_;

  size_t samples_drawn_{0};
  size_t samples_accepted_{0};
};

class GMMConstraintSamplerAllocator : public constraint_samplers::ConstraintSamplerAllocator
{
public:
  GMMConstraintSamplerAllocator();
  ~GMMConstraintSamplerAllocator() override;

  constraint_samplers::ConstraintSamplerPtr alloc(
      const planning_scene::PlanningSceneConstPtr& scene,
      const std::string& group_name,
      const moveit_msgs::msg::Constraints& constr) override;

  bool canService(
      const planning_scene::PlanningSceneConstPtr& scene,
      const std::string& group_name,
      const moveit_msgs::msg::Constraints& constr) const override;

  void setLatestGMM(tp_gmm::msg::GaussianMixture::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(gmm_mutex_);
    latest_gmm_ = msg;
  }

  tp_gmm::msg::GaussianMixture::SharedPtr getLatestGMM() const
  {
    std::lock_guard<std::mutex> lock(gmm_mutex_);
    return latest_gmm_;
  }

private:
  void gmmCallback(const tp_gmm::msg::GaussianMixture::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<tp_gmm::msg::GaussianMixture>::SharedPtr gmm_sub_;
  rclcpp::Subscription<tp_gmm::msg::GaussianMixture>::SharedPtr deformed_gmm_sub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr viz_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr stats_pub_;

  mutable std::mutex gmm_mutex_;
  tp_gmm::msg::GaussianMixture::SharedPtr latest_gmm_;

  std::thread spin_thread_;
  std::atomic<bool> running_{true};
};

} // namespace tp_gmm
