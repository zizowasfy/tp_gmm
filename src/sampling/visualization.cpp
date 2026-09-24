#include "sampler.hpp"

namespace tp_gmm
{
void GMMConstraintSamplerAllocator::publishMarkers()
{
  visualization_msgs::msg::MarkerArray array;
  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto& [id, session] : sessions_)
  {
    if (!session->config.visualize) continue;
    std::lock_guard<std::mutex> session_lock(session->mutex);
    auto base = [&] {
      visualization_msgs::msg::Marker m; m.header.frame_id = session->config.model.header.frame_id;
      m.header.stamp = node_->now(); m.pose.orientation.w = 1; m.lifetime = rclcpp::Duration::from_seconds(30); return m;
    };
    const char* names[] = {"cartesian_proposals", "valid_fk", "rejected_fk", "uniform_valid", "gmr_proposals"};
    const float colors[5][3] = {{0, 0.8f, 1}, {0.1f, 1, 0.1f}, {1, 0.1f, 0.1f}, {1, 0.8f, 0}, {1, 0.2f, 0.8f}};
    for (int s = 0; s < 5; ++s)
    {
      auto m = base(); m.ns = id + "/" + names[s]; m.type = m.POINTS;
      m.scale.x = m.scale.y = 0.006;
      m.color.r = colors[s][0]; m.color.g = colors[s][1]; m.color.b = colors[s][2]; m.color.a = 0.85;
      m.points.assign(session->points[s].begin(), session->points[s].end()); array.markers.push_back(m);
    }
    if (session->reference)
    {
      auto line = base(); line.ns = id + "/gmr_reference"; line.type = line.LINE_STRIP;
      line.scale.x = .004; line.color.r = 1; line.color.b = .8; line.color.a = 1;
      // Bound visualization size independently from the supplied path resolution.
      const size_t count = std::min(size_t(128), session->reference->points().size());
      for (const auto& x : session->reference->points()) line.points.push_back(point(x));
      array.markers.push_back(line);
      if (session->config.proposal != "gmm" && session->config.gmr_stddev > 0)
      {
        auto tube = base(); tube.ns = id + "/gmr_neighborhood"; tube.type = tube.SPHERE_LIST;
        tube.scale.x = tube.scale.y = tube.scale.z = 2 * session->config.gmr_stddev * session->config.gmr_cutoff;
        tube.color = line.color; tube.color.a = .08; 
        for (size_t i = 0; i < count; ++i)
          tube.points.push_back(point(session->reference->at(double(i)/(count-1))));
        array.markers.push_back(tube);
      }
    }
    for (size_t k = 0; k < session->gaussians.size(); ++k)
    {
      if (session->cutoffs[k] == 0) continue;
      const auto& g = session->gaussians[k]; auto m = base(); m.ns = id + "/support"; m.id = k; m.type = m.SPHERE;
      m.pose.position = point(g.mean); const Eigen::Quaterniond q(g.axes);
      m.pose.orientation.w = q.w(); m.pose.orientation.x = q.x(); m.pose.orientation.y = q.y(); m.pose.orientation.z = q.z();
      const Eigen::Vector3d scale = 2 * session->cutoffs[k] * g.eigenvalues.cwiseSqrt();
      m.scale.x = scale.x(); m.scale.y = scale.y(); m.scale.z = scale.z();
      m.color.r = 0.65; m.color.g = 0.4; m.color.b = 1; m.color.a = 0.12; array.markers.push_back(m);
    }
  }
  if (!array.markers.empty()) markers_->publish(array);
}

}  // namespace tp_gmm
