#pragma once
#include <moveit/constraint_samplers/constraint_sampler_allocator.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tp_gmm/srv/prepare_sampling.hpp>
#include <tp_gmm/srv/sampling_report.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <map>
#include <mutex>
#include <thread>

namespace tp_gmm
{
struct SamplingSession;
class GMMConstraintSamplerAllocator : public constraint_samplers::ConstraintSamplerAllocator
{
public:
  GMMConstraintSamplerAllocator();
  ~GMMConstraintSamplerAllocator() override;
  constraint_samplers::ConstraintSamplerPtr alloc(const planning_scene::PlanningSceneConstPtr& scene,
      const std::string& group, const moveit_msgs::msg::Constraints& constraints) override;
  bool canService(const planning_scene::PlanningSceneConstPtr&, const std::string&,
      const moveit_msgs::msg::Constraints& constraints) const override;
private:
  void prepare(const srv::PrepareSampling::Request& req, srv::PrepareSampling::Response& res);
  void report(const srv::SamplingReport::Request& req, srv::SamplingReport::Response& res);
  void publishMarkers();
  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread worker_;
  rclcpp::Service<srv::PrepareSampling>::SharedPtr prepare_service_;
  rclcpp::Service<srv::SamplingReport>::SharedPtr report_service_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr paths_;
  visualization_msgs::msg::MarkerArray comparison_paths_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::mutex mutex_;
  std::map<std::string, std::shared_ptr<SamplingSession>> sessions_;
  uint64_t sequence_{0};
  std::string epoch_;
};
}  // namespace tp_gmm
