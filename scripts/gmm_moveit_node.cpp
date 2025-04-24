#include "ros/ros.h"

#include <moveit/macros/class_forward.h>
#include <moveit_msgs/Constraints.h>
#include <visualization_msgs/MarkerArray.h>

#include <sstream>

// This node converts the GMM received from the topic, and convert it to position_contraint for move_group_interface

class gmmMoveit
{
private:
    /* data */
    ros::Publisher gmm_constraint_pub;
    ros::Publisher gmmViz_pub;
    ros::Subscriber gmm_markerarray_sub;
public:
    gmmMoveit(ros::NodeHandle nh)
    {   
        gmm_markerarray_sub = nh.subscribe("/gmm_rviz_converter_output", 1, &gmmMoveit::onGMM, this);

        gmm_constraint_pub = nh.advertise<moveit_msgs::BoundingVolume>("/gmm_moveit", 1);

        gmmViz_pub = nh.advertise<visualization_msgs::MarkerArray>("/solid_primitives_viz", 1);
    }

    void onGMM(visualization_msgs::MarkerArrayPtr msg)
    {
      // moveit_msgs::PositionConstraint pc;
      moveit_msgs::BoundingVolume bv;
      shape_msgs::SolidPrimitive sp;
      visualization_msgs::Marker viz_marker;
      visualization_msgs::MarkerArray viz_markerarray;        

      // counting the number of gaussians in MarkerArray, because MarkerArray by default contains number of elements more than the actual saved GMM
      int nbGMM = 0;
      for (uint id = 0; id < msg->markers.size(); id++)
      {
        if ( sqrt(pow(msg->markers[id].pose.position.x, 2) + pow(msg->markers[id].pose.position.y, 2) + pow(msg->markers[id].pose.position.z, 2)) == 0.0 )
        { break; }
        nbGMM++;
      }
      std::cout << "nbGMM = " << nbGMM << std::endl;
      for (uint g = 0; g < nbGMM; g++)
      {
        sp.type = shape_msgs::SolidPrimitive::BOX;
        sp.dimensions = {msg->markers[g].scale.x, msg->markers[g].scale.y, msg->markers[g].scale.z};
        bv.primitives.push_back(sp);

        geometry_msgs::Pose sp_pose;
        sp_pose.position.x = msg->markers[g].pose.position.x;
        sp_pose.position.y = msg->markers[g].pose.position.y;
        sp_pose.position.z = msg->markers[g].pose.position.z;
        sp_pose.orientation.x = msg->markers[g].pose.orientation.x;
        sp_pose.orientation.y = msg->markers[g].pose.orientation.y;
        sp_pose.orientation.z = msg->markers[g].pose.orientation.z;
        sp_pose.orientation.w = msg->markers[g].pose.orientation.w;
        bv.primitive_poses.push_back(sp_pose);

        // Visualizing the SolidPrimitive that is used to define an approximation of GMM volume (the new search space)
        viz_marker.header.frame_id = msg->markers[g].header.frame_id; //"base_link";
        viz_marker.id = msg->markers[g].id;
        viz_marker.type = visualization_msgs::Marker::CUBE;
        viz_marker.action = visualization_msgs::Marker::ADD;
        viz_marker.pose = sp_pose;
        geometry_msgs::Vector3 v3;
        // v3.x = 1.0; v3.y = 1.0; v3.z = 1.0;
        v3.x = msg->markers[g].scale.x; v3.y = msg->markers[g].scale.y; v3.z = msg->markers[g].scale.z;
        viz_marker.scale = v3;
        viz_marker.color = msg->markers[g].color; viz_marker.color.a = 0.4;
        // gmmViz_pub.publish(viz_marker);
        viz_markerarray.markers.push_back(viz_marker);
      }
      
      gmm_constraint_pub.publish(bv);
      gmmViz_pub.publish(viz_markerarray);
    //   pcm.constraint_region = bv;

    }

};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "GMM_Moveit_Node");
    ros::NodeHandle nh;

    gmmMoveit gmmMovit(nh);
    ros::spin();

    return 0;
}