#include <rclcpp/rclcpp.hpp>

#include <moveit_msgs/msg/bounding_volume.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <sstream>
#include <cmath>

namespace tp_gmm
{

class GMMMoveIt : public rclcpp::Node
{
private:
    rclcpp::Publisher<moveit_msgs::msg::BoundingVolume>::SharedPtr gmm_constraint_pub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr gmmViz_pub;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr gmm_markerarray_sub;

public:
    GMMMoveIt(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("GMM_Moveit_Node", options)
    {   
        gmm_markerarray_sub = this->create_subscription<visualization_msgs::msg::MarkerArray>(
            "gmm_rviz_converter_output", 1, std::bind(&GMMMoveIt::onGMM, this, std::placeholders::_1));

        gmm_constraint_pub = this->create_publisher<moveit_msgs::msg::BoundingVolume>("gmm_moveit", 1);
        gmmViz_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("solid_primitives_viz", 1);
    }

    void onGMM(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
    {
      moveit_msgs::msg::BoundingVolume bv;
      visualization_msgs::msg::MarkerArray viz_markerarray;

      for (const auto & marker : msg->markers)
      {
        // Only process markers with ADD action; skip DELETE markers
        if (marker.action != visualization_msgs::msg::Marker::ADD)
        {
          continue;
        }

        // Skip degenerate markers with non-positive dimensions
        if (marker.scale.x <= 0.0 || marker.scale.y <= 0.0 || marker.scale.z <= 0.0)
        {
          continue;
        }

        shape_msgs::msg::SolidPrimitive sp;
        sp.type = shape_msgs::msg::SolidPrimitive::BOX;
        sp.dimensions = {marker.scale.x, marker.scale.y, marker.scale.z};
        bv.primitives.push_back(sp);

        geometry_msgs::msg::Pose sp_pose = marker.pose;
        bv.primitive_poses.push_back(sp_pose);

        // Visualizing the SolidPrimitive
        visualization_msgs::msg::Marker viz_marker;
        viz_marker.header.frame_id = marker.header.frame_id;
        viz_marker.header.stamp = this->now();
        viz_marker.id = marker.id;
        viz_marker.type = visualization_msgs::msg::Marker::CUBE;
        viz_marker.action = visualization_msgs::msg::Marker::ADD;
        viz_marker.pose = sp_pose;
        viz_marker.scale = marker.scale;
        viz_marker.color = marker.color;
        viz_marker.color.a = 0.4;

        viz_markerarray.markers.push_back(viz_marker);
      }

      RCLCPP_INFO(this->get_logger(), "Published bounding volume with %zu primitives", bv.primitives.size());
      gmm_constraint_pub->publish(bv);
      gmmViz_pub->publish(viz_markerarray);
    }
};

} // namespace tp_gmm

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(tp_gmm::GMMMoveIt)
