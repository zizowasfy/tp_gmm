#!/usr/bin/env python3

"""
Policy Validation Script for Kinova Gen 3 (ROS 2)
Validates the standalone policy outputs against recorded IsaacLab RL environment experiments.
"""

import sys
import os
import time
import math
import pickle
import numpy as np
from pathlib import Path
from copy import deepcopy

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from scipy.spatial.transform import Rotation as R

from geometry_msgs.msg import PoseStamped, Pose, PoseArray, Quaternion
from moveit_msgs.msg import BoundingVolume, Constraints, PositionConstraint, DisplayTrajectory, OrientationConstraint
from moveit_msgs.action import MoveGroup
from moveit_msgs.srv import GetPositionFK
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header
from tp_gmm.srv import ReproduceTPGMM, DeformTPGMM

# Import the standalone policy
from ament_index_python.packages import get_package_share_directory
pkg_share = get_package_share_directory('tp_gmm')
sys.path.append(os.path.join(pkg_share, 'scripts'))
sys.path.append(os.path.join(pkg_share, 'include'))
from standalone_policy import TPGMMDeformationPolicy
import torch

class PolicyValidator(Node):
    def __init__(self, namespace="", frame_id="base_link", group_name="manipulator"):
        super().__init__('validate_policy_node')
        
        self.namespace = namespace
        self.frame_id = frame_id
        self.group_name = group_name
        self.ee_link = f"{self.namespace[1:]}_end_effector_link" if namespace.startswith("/") else "end_effector_link"
        
        # Load recorded experiments
        self.recorded_data_path = "/home/zizo/the_folder/Reach_direct/recorded_experiments.pkl"
        if not os.path.exists(self.recorded_data_path):
            self.get_logger().error(f"Recorded data path not found: {self.recorded_data_path}")
            sys.exit(1)
        with open(self.recorded_data_path, 'rb') as f:
            self.recorded_episodes = pickle.load(f)
        self.get_logger().info(f"Loaded {len(self.recorded_episodes)} episodes for validation.")

        # Load local policy
        # self.policy_ckpt_path = '/home/zizo/the_folder/Reach_direct/logs/skrl/cartpole_direct/2026-06-04_16-49-13_ppo_torch_envs=32/checkpoints/best_agent.pt'
        self.policy_ckpt_path = '/home/zizo/the_folder/Reach_direct/logs/skrl/cartpole_direct/2026-08-05_21-41-30_ppo_torch/checkpoints/best_agent.pt'
        if os.path.exists(self.policy_ckpt_path):
            self.policy = TPGMMDeformationPolicy.load_from_skrl_checkpoint(self.policy_ckpt_path)
            self.policy.eval()
            self.get_logger().info(f"Loaded standalone policy from {self.policy_ckpt_path}")
        else:
            self.get_logger().error(f"Policy checkpoint not found: {self.policy_ckpt_path}")
            sys.exit(1)

        self.get_logger().info("Initializing MoveGroup Action Client...")
        self.move_action_client = ActionClient(self, MoveGroup, f'{self.namespace}/move_action')
        if not self.move_action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("MoveGroup action server not available!")
            sys.exit(1)
            
        self.fk_client = self.create_client(GetPositionFK, f'{self.namespace}/compute_fk')
        self.reproduce_tpgmm_client = self.create_client(ReproduceTPGMM, f"{self.namespace}/ReproduceTPGMM_service")
        self.deform_tpgmm_client = self.create_client(DeformTPGMM, f"{self.namespace}/DeformTPGMM_service")
        
        self.start_pose_pub = self.create_publisher(PoseStamped, f"{self.namespace}/start_pose", 1)
        self.goal_pose_pub = self.create_publisher(PoseStamped, f"{self.namespace}/goal_pose", 1)
        self.start_pose_gmm_pub = self.create_publisher(PoseStamped, f"{self.namespace}/start_pose_gmm", 1)
        self.goal_pose_gmm_pub = self.create_publisher(PoseStamped, f"{self.namespace}/goal_pose_gmm", 1)
        self.posearray_pub = self.create_publisher(PoseArray, f"{self.namespace}/planned_trajectory/posearray", 1)
        self.obstacle_pose_pub = self.create_publisher(PoseStamped, f"{self.namespace}/obstacle_pose", 1)
        
        self.create_subscription(BoundingVolume, f"{self.namespace}/gmm_moveit", self.gmm_constraint_cb, 10)
        self.create_subscription(BoundingVolume, f"{self.namespace}/deformed_gmm_moveit", self.deformed_gmm_constraint_cb, 10)
        self.create_subscription(DisplayTrajectory, f"{self.namespace}/move_group/display_planned_path", self.display_path_cb, 10)
        
        self.start_pose = None
        self.goal_pose = None
        self.plan_result = None
        self.gmm_bounding_volume = None
        self.deformed_gmm_bounding_volume = None
        
        self.get_logger().info("Policy Validator Node Initialized Successfully!")

    def adjust_orientation(self, pose, ref_robot):
        # This orientation adjustment aligns the frames of the operated robot's ee with the ur10e's ee frames (which the demonstrations are recorded in)
        kinova_orientation = R.from_quat([pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w])
        if ref_robot == "ur10":
            adjustment_rotation = R.from_euler('ZYX', [-90, -90, 0], degrees=True)
        elif ref_robot == "franka_panda":
            adjustment_rotation = R.from_euler('ZYX', [90, 0, 0], degrees=True)

        adjusted_rotation = kinova_orientation * adjustment_rotation
        adjusted_pose = deepcopy(pose)
        adjusted_pose.pose.orientation.x = adjusted_rotation.as_quat()[0]
        adjusted_pose.pose.orientation.y = adjusted_rotation.as_quat()[1]
        adjusted_pose.pose.orientation.z = adjusted_rotation.as_quat()[2]
        adjusted_pose.pose.orientation.w = adjusted_rotation.as_quat()[3]
        
        return adjusted_pose

    def reverse_adjust_orientation(self, pose, ref_robot):
        adjusted_orientation = R.from_quat([pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w])
        if ref_robot == "ur10":
            adjustment_rotation = R.from_euler('ZYX', [-90, -90, 0], degrees=True)
        elif ref_robot == "franka_panda":
            adjustment_rotation = R.from_euler('ZYX', [90, 0, 0], degrees=True)

        inverse_adjustment = adjustment_rotation.inv()
        original_rotation = adjusted_orientation * inverse_adjustment
        original_pose = deepcopy(pose)
        original_pose.pose.orientation.x = original_rotation.as_quat()[0]
        original_pose.pose.orientation.y = original_rotation.as_quat()[1]
        original_pose.pose.orientation.z = original_rotation.as_quat()[2]
        original_pose.pose.orientation.w = original_rotation.as_quat()[3]
        
        return original_pose

    def construct_goal_constraints(self, target_pose):
        goal_constraint = Constraints()
        
        pos_constraint = PositionConstraint()
        pos_constraint.header = target_pose.header
        pos_constraint.link_name = self.ee_link
        
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [0.01]
        
        region_pose = Pose()
        region_pose.position = target_pose.pose.position
        region_pose.orientation.w = 1.0
        
        bv = BoundingVolume()
        bv.primitives.append(primitive)
        bv.primitive_poses.append(region_pose)
        
        pos_constraint.constraint_region = bv
        pos_constraint.weight = 1.0
        
        ori_constraint = OrientationConstraint()
        ori_constraint.header = target_pose.header
        ori_constraint.link_name = self.ee_link
        ori_constraint.orientation = target_pose.pose.orientation
        ori_constraint.absolute_x_axis_tolerance = 0.05
        ori_constraint.absolute_y_axis_tolerance = 0.05
        ori_constraint.absolute_z_axis_tolerance = 0.05
        ori_constraint.weight = 1.0
        
        goal_constraint.position_constraints.append(pos_constraint)
        goal_constraint.orientation_constraints.append(ori_constraint)
        return goal_constraint

    def plan_and_execute(self, target_pose, apply_constraints=False, use_deformed=False):
        self.get_logger().info("Sending goal to MoveGroup Action Server...")
        
        req = MoveGroup.Goal()
        req.request.group_name = self.group_name
        req.request.num_planning_attempts = 3
        req.request.allowed_planning_time = 5.0
        req.request.max_velocity_scaling_factor = 1.0
        req.request.max_acceleration_scaling_factor = 1.0
        
        goal_constraint = self.construct_goal_constraints(target_pose)
        req.request.goal_constraints.append(goal_constraint)
        
        bv_to_use = self.deformed_gmm_bounding_volume if use_deformed else self.gmm_bounding_volume
        if apply_constraints and bv_to_use:
            self.get_logger().info("Applying GMM Path Constraints...")
            path_constraint = Constraints()
            path_constraint.name = "position_constraint"
            p_const = PositionConstraint()
            p_const.header.frame_id = self.frame_id
            p_const.link_name = self.ee_link
            p_const.constraint_region = bv_to_use
            p_const.weight = 1.0
            path_constraint.position_constraints.append(p_const)
            req.request.path_constraints = path_constraint

        req.planning_options.plan_only = False
        
        future = self.move_action_client.send_goal_async(req)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error("MoveGroup Goal rejected")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        res = result_future.result().result

        if res.error_code.val == res.error_code.SUCCESS:
            self.get_logger().info("Execution succeeded.")
            self.plan_result = res.planned_trajectory
            return True
        else:
            self.get_logger().error(f"MoveGroup failed with error code: {res.error_code.val}")
            return False

    def gmm_constraint_cb(self, msg):
        self.gmm_bounding_volume = msg

    def deformed_gmm_constraint_cb(self, msg):
        self.deformed_gmm_bounding_volume = msg

    def display_path_cb(self, msg):
        pass

    def call_tpgmm_service(self, task, start_pose, target_pose):
        self.get_logger().info("Calling ReproduceTPGMM service...")
        if not self.reproduce_tpgmm_client.wait_for_service(timeout_sec=2.0):
            return False

        req = ReproduceTPGMM.Request()
        req.task_name = task
        req.frame_id = self.frame_id
        req.start_pose = self.adjust_orientation(start_pose, ref_robot='ur10')
        req.goal_pose = self.adjust_orientation(target_pose, ref_robot='ur10')

        self.start_pose_gmm_pub.publish(req.start_pose)
        self.goal_pose_gmm_pub.publish(req.goal_pose)

        future = self.reproduce_tpgmm_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info("Successfully reproduced TPGMM.")
            for _ in range(20):
                rclpy.spin_once(self, timeout_sec=0.1)
                if self.gmm_bounding_volume is not None:
                    break
            return True
        return False

    def call_deform_tpgmm_service(self, task, start_pose, target_pose, obstacle_pose, obstacle_radius, desired_clearance):
        self.get_logger().info("Calling DeformTPGMM service...")
        if not self.deform_tpgmm_client.wait_for_service(timeout_sec=2.0):
            return False

        req = DeformTPGMM.Request()
        req.task_name = task
        req.frame_id = self.frame_id
        req.tpgmm_start_pose = self.adjust_orientation(start_pose, ref_robot='ur10')
        req.tpgmm_goal_pose = self.adjust_orientation(target_pose, ref_robot='ur10')
        req.deformed_tpgmm_start_pose = self.adjust_orientation(start_pose, ref_robot='ur10')
        req.deformed_tpgmm_goal_pose = self.adjust_orientation(target_pose, ref_robot='ur10')
        req.obstacle_pose = obstacle_pose
        req.obstacle_radius = obstacle_radius
        req.desired_clearance = desired_clearance

        self.obstacle_pose_pub.publish(obstacle_pose)

        future = self.deform_tpgmm_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info("Successfully deformed TPGMM.")
            for _ in range(20):
                rclpy.spin_once(self, timeout_sec=0.1)
                if self.deformed_gmm_bounding_volume is not None:
                    break
            return True
        return False

    def validate(self, task_name):
        self.get_logger().info("Starting Policy Validation against Recorded Episodes...")
        for ep_idx, ep_data in enumerate(self.recorded_episodes):
            self.get_logger().info(f"\n==========================================")
            self.get_logger().info(f"--- Episode {ep_idx} ---")
            self.get_logger().info(f"==========================================")

            self.gmm_bounding_volume = None
            self.deformed_gmm_bounding_volume = None

            # Get poses from recorded experiment (stored in UR10/Panda convention)
            rec_start = ep_data["start_pose"]
            rec_goal = ep_data["goal_pose"]
            rec_obs_pos = ep_data["obstacle_pos_local"]
            rec_obs_quat = ep_data["obstacle_quat_local"]

            # Construct ROS PoseStamped messages (quaternion expected in xyzw in ROS, but we map w to index 3 and xyz to 4,5,6)
            start_pose_ur10 = PoseStamped()
            start_pose_ur10.header.frame_id = self.frame_id
            start_pose_ur10.pose.position.x = float(rec_start[0])
            start_pose_ur10.pose.position.y = float(rec_start[1])
            start_pose_ur10.pose.position.z = float(rec_start[2])
            start_pose_ur10.pose.orientation.x = float(rec_start[4])
            start_pose_ur10.pose.orientation.y = float(rec_start[5])
            start_pose_ur10.pose.orientation.z = float(rec_start[6])
            start_pose_ur10.pose.orientation.w = float(rec_start[3])

            goal_pose_ur10 = PoseStamped()
            goal_pose_ur10.header.frame_id = self.frame_id
            goal_pose_ur10.pose.position.x = float(rec_goal[0])
            goal_pose_ur10.pose.position.y = float(rec_goal[1])
            goal_pose_ur10.pose.position.z = float(rec_goal[2])
            goal_pose_ur10.pose.orientation.x = float(rec_goal[4])
            goal_pose_ur10.pose.orientation.y = float(rec_goal[5])
            goal_pose_ur10.pose.orientation.z = float(rec_goal[6])
            goal_pose_ur10.pose.orientation.w = float(rec_goal[3])

            # Convert to Kinova frame for MoveIt planning/execution
            start_pose_kinova = self.reverse_adjust_orientation(start_pose_ur10, ref_robot='ur10')
            goal_pose_kinova = self.reverse_adjust_orientation(goal_pose_ur10, ref_robot='ur10')

            # Obstacle PoseStamped
            obstacle_pose = PoseStamped()
            obstacle_pose.header.frame_id = self.frame_id
            obstacle_pose.pose.position.x = float(rec_obs_pos[0])
            obstacle_pose.pose.position.y = float(rec_obs_pos[1])
            obstacle_pose.pose.position.z = float(rec_obs_pos[2])
            obstacle_pose.pose.orientation.x = float(rec_obs_quat[1])
            obstacle_pose.pose.orientation.y = float(rec_obs_quat[2])
            obstacle_pose.pose.orientation.z = float(rec_obs_quat[3])
            obstacle_pose.pose.orientation.w = float(rec_obs_quat[0])

            self.start_pose = start_pose_kinova
            self.goal_pose = goal_pose_kinova

            # Publish start, goal and obstacle
            self.start_pose_pub.publish(self.start_pose)
            self.goal_pose_pub.publish(self.goal_pose)
            self.obstacle_pose_pub.publish(obstacle_pose)

            # Compare recorded steps against local policy
            steps = ep_data["steps"]
            if len(steps) > 0:
                first_step = steps[0]
                rec_obs = first_step["observation"]
                rec_action = first_step["action"]
                rec_def_mu = first_step["deformed_mu"]
                rec_orig_mu = first_step["original_mu"]

                obs_tensor = torch.tensor(rec_obs, dtype=torch.float32).unsqueeze(0)
                with torch.no_grad():
                    pred_action = self.policy(obs_tensor).squeeze(0).cpu().numpy()

                action_diff_norm = np.linalg.norm(pred_action - rec_action)
                action_diff_max = np.max(np.abs(pred_action - rec_action))

                action_scale = 0.1
                pred_action_deltas = pred_action.reshape(-1, 3) * action_scale
                pred_def_mu = rec_orig_mu + pred_action_deltas

                mu_diff_norm = np.linalg.norm(pred_def_mu - rec_def_mu)
                mu_diff_max = np.max(np.abs(pred_def_mu - rec_def_mu))

                self.get_logger().info(f"Local Policy Validation Results:")
                self.get_logger().info(f"  Action Diff (L2 norm): {action_diff_norm:.6f}")
                self.get_logger().info(f"  Action Diff (Max abs):  {action_diff_max:.6f}")
                self.get_logger().info(f"  Deformed Mu Diff (L2):  {mu_diff_norm:.6f}")
                self.get_logger().info(f"  Deformed Mu Diff (Max): {mu_diff_max:.6f}")

            ans = input(f"Proceed to execute planning for Episode {ep_idx}? [y/n]: ")
            if ans.lower() != 'y':
                continue

            if self.plan_and_execute(self.start_pose):
                time.sleep(1.0)
                
                # Call TPGMM reproduction service
                if self.call_tpgmm_service(task_name, self.start_pose, self.goal_pose):
                    # ans2 = input("Execute the original constrained plan? [y/n]: ")
                    ans2 = 'n'
                    if ans2.lower() == 'y':
                        self.plan_and_execute(self.goal_pose, apply_constraints=True, use_deformed=False)

                ## debugging
                obstacle_clearance = 0.0
                while obstacle_clearance < 1.0:
                    # Call DeformTPGMM service using recorded obstacle position and radius
                    obstacle_clearance = float(input("Enter desired clearance scale factor [0, 1]: "))
                    if self.call_deform_tpgmm_service(task_name, self.start_pose, self.goal_pose, obstacle_pose, obstacle_radius=0.05, desired_clearance=obstacle_clearance):
                        # ans3 = input("Execute the DEFORMED constrained plan? [y/n]: ")
                        ans3 = 'n'
                        if ans3.lower() == 'y':
                            self.plan_and_execute(self.goal_pose, apply_constraints=True, use_deformed=True)
                ##\ debugging
            time.sleep(0.5)

def main(args=None):
    rclpy.init(args=args)
    namespace = ""
    frame_id = "base_link"
    group_name = "manipulator" 
    task_name = "pick"
    
    node = PolicyValidator(namespace, frame_id, group_name)
    try:
        node.validate(task_name)
        node.get_logger().info("Execution complete.")
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
