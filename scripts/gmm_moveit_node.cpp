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
      shape_msgs::msg::SolidPrimitive sp;
      visualization_msgs::msg::Marker viz_marker;
      visualization_msgs::msg::MarkerArray viz_markerarray;

      // counting the number of gaussians in MarkerArray
      int nbGMM = 0;
      for (size_t id = 0; id < msg->markers.size(); id++)
      {
        if ( std::sqrt(std::pow(msg->markers[id].pose.position.x, 2) +
                       std::pow(msg->markers[id].pose.position.y, 2) +
                       std::pow(msg->markers[id].pose.position.z, 2)) == 0.0 )
        { break; }
        nbGMM++;
      }
      RCLCPP_INFO(this->get_logger(), "nbGMM = %d", nbGMM);

      for (int g = 0; g < nbGMM; g++)
      {
        sp.type = shape_msgs::msg::SolidPrimitive::BOX;
        sp.dimensions = {msg->markers[g].scale.x, msg->markers[g].scale.y, msg->markers[g].scale.z};
        bv.primitives.push_back(sp);

        geometry_msgs::msg::Pose sp_pose;
        sp_pose.position.x = msg->markers[g].pose.position.x;
        sp_pose.position.y = msg->markers[g].pose.position.y;
        sp_pose.position.z = msg->markers[g].pose.position.z;
        sp_pose.orientation.x = msg->markers[g].pose.orientation.x;
        sp_pose.orientation.y = msg->markers[g].pose.orientation.y;
        sp_pose.orientation.z = msg->markers[g].pose.orientation.z;
        sp_pose.orientation.w = msg->markers[g].pose.orientation.w;
        bv.primitive_poses.push_back(sp_pose);

        // Visualizing the SolidPrimitive
        viz_marker.header.frame_id = msg->markers[g].header.frame_id;
        viz_marker.id = msg->markers[g].id;
        viz_marker.type = visualization_msgs::msg::Marker::CUBE;
        viz_marker.action = visualization_msgs::msg::Marker::ADD;
        viz_marker.pose = sp_pose;

        geometry_msgs::msg::Vector3 v3;
        v3.x = msg->markers[g].scale.x;
        v3.y = msg->markers[g].scale.y;
        v3.z = msg->markers[g].scale.z;
        viz_marker.scale = v3;
        viz_marker.color = msg->markers[g].color;
        viz_marker.color.a = 0.4;

        viz_markerarray.markers.push_back(viz_marker);
      }
      
      gmm_constraint_pub->publish(bv);
      gmmViz_pub->publish(viz_markerarray);
    }
};

} // namespace tp_gmm

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(tp_gmm::GMMMoveIt)
