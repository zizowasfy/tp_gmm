#!/usr/bin/env python3
import rospy
import sys
from geometry_msgs.msg import Pose, PoseStamped, PoseArray
from tp_gmm.srv import StartTPGMM, StartTPGMMRequest, ReproduceTPGMM, ReproduceTPGMMRequest

def create_pose(p, o):
    """Helper function to create a Pose object."""
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = p
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = o
    return pose

def tpgmm_client():
    """
    A client to test the StartTPGMM and ReproduceTPGMM ROS services.
    """
    rospy.init_node('tpgmm_client')

    # The lfd.launch file starts the services within a namespace.
    # We'll target the 'ur10_1' namespace for this test.
    # Change this if you want to test 'ur10_2'.
    namespace = "ur10_1"
    start_service_name = f'/{namespace}/StartTPGMM_service'
    reproduce_service_name = f'/{namespace}/ReproduceTPGMM_service'

    print(f"Waiting for service '{start_service_name}'...")
    rospy.wait_for_service(start_service_name)
    print(f"Waiting for service '{reproduce_service_name}'...")
    rospy.wait_for_service(reproduce_service_name)

    try:
        # 1. --- Call the StartTPGMM service to train the model ---
        start_tpgmm = rospy.ServiceProxy(start_service_name, StartTPGMM)
        
        print("Requesting to start and train the TP-GMM...")
        start_req = StartTPGMMRequest()
        start_req.task_name = "pick"  # As seen in your lfd.launch
        start_req.train = False          # Explicitly request training

        start_resp = start_tpgmm(start_req)
        
        if start_resp.started:
            print("TP-GMM training service call successful!")
        else:
            print("TP-GMM training service call failed!")
            sys.exit(1)

        # 2. --- Call the ReproduceTPGMM service to get a new trajectory ---
        reproduce_tpgmm = rospy.ServiceProxy(reproduce_service_name, ReproduceTPGMM)

        print("\nRequesting to reproduce a trajectory from the TP-GMM...")
        reproduce_req = ReproduceTPGMMRequest()
        reproduce_req.task_name = "pick"
        reproduce_req.frame_id = "ur10_1_base_link" # Example frame_id

        # Create a sample start pose
        start_pose_stamped = PoseStamped()
        start_pose_stamped.header.frame_id = reproduce_req.frame_id
        start_pose_stamped.header.stamp = rospy.Time.now()
        start_pose_stamped.pose = create_pose(
            p=[0.5, -0.5, 0.4], 
            o=[0.0, 0.0, 0.0, 1.0]
        )
        reproduce_req.start_pose = start_pose_stamped

        # Create a sample goal pose
        goal_pose_stamped = PoseStamped()
        goal_pose_stamped.header.frame_id = reproduce_req.frame_id
        goal_pose_stamped.header.stamp = rospy.Time.now()
        goal_pose_stamped.pose = create_pose(
            p=[0.5, 0.5, 0.4], 
            o=[0.0, 0.0, 0.0, 1.0]
        )
        reproduce_req.goal_pose = goal_pose_stamped

        # Create a sample obstacle pose (optional, can be empty)
        obstacle_poses = PoseArray()
        obstacle_poses.header.frame_id = reproduce_req.frame_id
        obstacle_poses.header.stamp = rospy.Time.now()
        obstacle_poses.poses.append(
            create_pose(p=[0.5, 0.0, 0.5], o=[0.0, 0.0, 0.0, 1.0])
        )
        # reproduce_req.obstacles_poses = obstacle_poses # Uncomment if your model uses obstacles

        reproduce_resp = reproduce_tpgmm(reproduce_req)
        print("TP-GMM reproduction service call successful. Check RViz for the published GMM and trajectory.")

    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")

if __name__ == "__main__":
    tpgmm_client()
