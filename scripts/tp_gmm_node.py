#!/usr/bin/env python3
# ROS stuff
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose, PointStamped, PoseArray
from sensor_msgs.msg import JointState
from moveit_msgs.action import MoveGroup
from trajectory_msgs.msg import JointTrajectoryPoint

from tp_gmm.msg import GaussianMixture
from tp_gmm.srv import StartTPGMM, ReproduceTPGMM, DeformTPGMM

import pickle

## System and directories stuff
import sys
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import os

pkg_share = get_package_share_directory('tp_gmm')
sys.path.append(os.path.join(pkg_share, 'include'))
sys.path.append(os.path.join(pkg_share, 'scripts'))

from dir_paths import get_paths, get_task_dir, ensure_task_dir, get_demonstrations_dir
paths = get_paths()
ws_dir = paths['external_root']
data_dir = paths['data_dir']
scripts_dir = paths['scripts_dir']
tasks_dir = paths['tasks_dir']
# print(f"scripts_dir: {scripts_dir}")
from demons_to_samples import process_demonstrations
from standalone_policy import TPGMMDeformationPolicy

# tpgmm-related stuff
import time
import numpy as np
from math import sqrt
from scipy.spatial.transform import Rotation as R
from sClass import s
from pClass import p
from rClass import r
from modelClass import model
from matplotlib import pyplot as plt
from TPGMM_GMR import TPGMM_GMR
from copy import deepcopy,copy
import torch

class TPGMM(Node):
    def __init__(self):
        super().__init__('tp_gmm_node')
        self.get_logger().info(" --> Node tp_gmm_node is initialized")
        self.TPGMM_model = None
        self.demons_info = None

        self.srv_start = self.create_service(StartTPGMM, "StartTPGMM_service", self.startTPGMM)
        self.srv_reproduce = self.create_service(ReproduceTPGMM, "ReproduceTPGMM_service", self.tpGMMGMR)
        self.srv_deform = self.create_service(DeformTPGMM, "DeformTPGMM_service", self.deform_tpgmm_callback)

        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.policy = None
        self.declare_parameter('policy_ckpt_path', '/home/zizo/the_folder/Reach_direct/logs/skrl/cartpole_direct/2026-09-01_01-05-26_ppo_torch/checkpoints/best_agent.pt')
        self.declare_parameter('action_scale', 0.15)

        ckpt_path = self.get_parameter('policy_ckpt_path').get_parameter_value().string_value
        self.action_scale = self.get_parameter('action_scale').get_parameter_value().double_value

        if os.path.exists(ckpt_path):
            try:
                self.policy = TPGMMDeformationPolicy.load_from_skrl_checkpoint(ckpt_path)
                self.policy.to(self.device)
                self.get_logger().info(f"Successfully loaded policy from {ckpt_path}")
            except Exception as e:
                self.get_logger().error(f"Failed to load policy: {e}")
        else:
            self.get_logger().warn(f"Policy checkpoint not found at {ckpt_path}. Deform service will just return original GMM.")

        self.regress_traj_pub = self.create_publisher(PoseArray, 'gmm/regressed_trajectory', 1)
        self.deformed_regress_traj_pub = self.create_publisher(PoseArray, 'gmm/deformed_regressed_trajectory', 1)
        self.move_group_q_viz_pub = self.create_publisher(MoveGroup.Result, '/move_group/_action/status', 1) # Just a placeholder since old type is removed
        self.tpgmm_pub = self.create_publisher(GaussianMixture, '/gmm/mix', 1)
        self.tpgmm_viz_pub = self.create_publisher(GaussianMixture, '/gmm/cartesian_space', 1)
        self.tpgmm_deformed_viz_pub = self.create_publisher(GaussianMixture, '/gmm/deformed_cartesian_space', 1)
        self.learned_traj_pub = self.create_publisher(PoseArray, '/gmm/learned_trajectory', 1)
        self.solveFK_pub = self.create_publisher(JointState, '/joint_samples', 1)
        self.tpgmm_time_pub = self.create_publisher(Float32, '/tpgmm/planning_time', 1)
        self.Data_posearray_pub = self.create_publisher(PoseArray, '/gmm/traj_input', 1)

    def startTPGMM(self, request, response):
        ## Receiving service Request
        self.frame1_pose = Pose()
        self.frame2_pose = Pose()
        _task_name = request.task_name #'pick'
        _train = True #req.train
        task_dir = ensure_task_dir(_task_name)

        ## Fetching Samples and paramters
        self.nbVar = 4      # Dim !!
        self.nbFrames = 2
        self.nbStates = 5  # nb of Gaussians

        if _train:
            self.get_logger().info(f"Processing demonstrations for task '{_task_name}'...")
            self.demons_info, slist = process_demonstrations(_task_name, self.nbFrames, self.nbStates, self.nbVar)
            self.get_logger().info(f"demons_info: {self.demons_info}")
        else:
            with open(task_dir / 'demons_info.pkl', 'rb') as fp:
                self.demons_info = pickle.load(fp)
                self.get_logger().info(f"demons_info: {self.demons_info}")
            slist = None

        ## Initialization of parameters and properties
        self.nbSamples = self.demons_info['nbDemons']  # nb of demonstrations
        self.nbData = self.demons_info['ref_nbpoints']#-1
        self.down_sample_factor = self.demons_info['down_sample_factor']

        if _train and slist is not None:
            self.tpGMM(_task_name, slist)
        response.started = True
        return response

    ## Preparing the samples and fit
    def tpGMM(self, _task_name, slist):
        self.slist = slist
        task_dir = ensure_task_dir(_task_name)

        # Creating instance of TPGMM_GMR
        TPGMMGMR = TPGMM_GMR(self.nbStates, self.nbFrames, self.nbVar)

        # Learning the model
        self.get_logger().info(f"Learning the TPGMM model for '{_task_name}'...")
        TPGMMGMR.fit(self.slist)

        # Saving the model in .pkl as backup and in memory
        self.TPGMM_model = TPGMMGMR
        with open(task_dir / 'TPGMM_model.pkl', 'wb') as fp:
            pickle.dump(self.TPGMM_model, fp)
        self.get_logger().info(f"TPGMM model for '{_task_name}' trained and saved successfully.")

    def tpGMMGMR(self, request, response):

        _task_name = request.task_name #'pick'
        task_dir = ensure_task_dir(_task_name)

        if self.TPGMM_model is not None:
            TPGMM_model = self.TPGMM_model
            self.get_logger().info(f"Using in-memory TPGMM model for '{_task_name}'.")
        else:
            with open(task_dir / 'TPGMM_model.pkl', 'rb') as fp:
                TPGMM_model = pickle.load(fp)
            self.get_logger().info(f"Loaded TPGMM model for '{_task_name}' from disk.")

        if self.demons_info is not None:
            task_demons_info = self.demons_info
        else:
            with open(task_dir / 'demons_info.pkl', 'rb') as fp:
                task_demons_info = pickle.load(fp)

        # Sorting the Task Parameters into Frames format
        frames_array = [request.start_pose.pose, request.goal_pose.pose]
        frames_array.extend(PoseArray().poses)

        # Reproduction with generated parameters in Cartesian Space
        newP = deepcopy(TPGMM_model.s[task_demons_info['demons_nums'].index(task_demons_info['ref'])].p)

        self.get_logger().info(f"newP.shape = {newP.shape}")

        for f in range(task_demons_info['nbFrames']):
            if f > 1:
                newb1 = np.array([[0], [frames_array[f].position.x], [frames_array[f].position.y], [frames_array[f].position.z+0.1]], dtype=object)
            else:
                newb1 = np.array([[0], [frames_array[f].position.x], [frames_array[f].position.y], [frames_array[f].position.z]], dtype=object)

            rA1 = R.from_quat([frames_array[f].orientation.x, frames_array[f].orientation.y, frames_array[f].orientation.z, frames_array[f].orientation.w])
            newA1 = np.vstack(( np.array([1,0,0,0]), np.hstack(( np.zeros((3,1)), rA1.as_matrix() )) ))

            newP[f,0].b = newb1
            newP[f,0].A = newA1
            newP[f,0].invA = np.linalg.inv(newA1)

        newPP = np.tile(newP[:,0][:, np.newaxis], newP.shape[1])

        reproduce_time = time.time()
        start_point = np.array([[0], [frames_array[0].position.x], [frames_array[0].position.y], [frames_array[0].position.z]], dtype=object)
        rnew = TPGMM_model.reproduce(newPP, start_point[1:,:])
        self.get_logger().info(f"... reproduce_time: {time.time() - reproduce_time}")

        ## Saving and Publishing GMM in Cartesian Space
        gmm = TPGMM_model.convertToGM(rnew, task_demons_info['down_sample_factor'], request.frame_id)
        self.tpgmm_viz_pub.publish(gmm)
        self.get_logger().info("GMM is Published!")

        regressed_trajectory = PoseArray() ; regressed_trajectory.header.frame_id = request.frame_id #'base_link'
        regressed_point = Pose()
        for i, point in enumerate(rnew.Data.T):
            regressed_point.position.x = point[1]
            regressed_point.position.y = point[2]
            regressed_point.position.z = point[3]
            regressed_trajectory.poses.append(deepcopy(regressed_point))

        self.get_logger().info(f"No. of points in Regressed Trajectory: {i}")
        self.regress_traj_pub.publish(regressed_trajectory)
        self.get_logger().info("Regressed Trajectory is Published!")

        return response

    def deform_tpgmm_callback(self, request, response):
        
        _task_name = request.task_name #'pick'
        task_dir = ensure_task_dir(_task_name)

        if self.TPGMM_model is not None:
            TPGMM_model = self.TPGMM_model
            self.get_logger().info(f"Using in-memory TPGMM model for '{_task_name}'.")
        else:
            with open(task_dir / 'TPGMM_model.pkl', 'rb') as fp:
                TPGMM_model = pickle.load(fp)
            self.get_logger().info(f"Loaded TPGMM model for '{_task_name}' from disk.")

        if self.demons_info is not None:
            task_demons_info = self.demons_info
        else:
            with open(task_dir / 'demons_info.pkl', 'rb') as fp:
                task_demons_info = pickle.load(fp)

        # Sorting the Task Parameters into Frames format
        # NOTE: if the RL policy trained on different start and goal poses, we should use the original tpgmm start and goal poses, i.e., request.tpgmm_start_pose and request.tpgmm_goal_pose
        # here it doesn't matter since the RL policy trained on the same start and goal poses, i.e. w.r.t ur10
        frames_array = [request.tpgmm_start_pose.pose, request.tpgmm_goal_pose.pose]
        frames_array.extend(PoseArray().poses)

        # Reproduction with generated parameters in Cartesian Space
        newP = deepcopy(TPGMM_model.s[task_demons_info['demons_nums'].index(task_demons_info['ref'])].p)

        self.get_logger().info(f"newP.shape = {newP.shape}")

        for f in range(task_demons_info['nbFrames']):
            if f > 1:
                newb1 = np.array([[0], [frames_array[f].position.x], [frames_array[f].position.y], [frames_array[f].position.z]], dtype=object)
            else:
                newb1 = np.array([[0], [frames_array[f].position.x], [frames_array[f].position.y], [frames_array[f].position.z]], dtype=object)

            rA1 = R.from_quat([frames_array[f].orientation.x, frames_array[f].orientation.y, frames_array[f].orientation.z, frames_array[f].orientation.w])
            newA1 = np.vstack(( np.array([1,0,0,0]), np.hstack(( np.zeros((3,1)), rA1.as_matrix() )) ))

            newP[f,0].b = newb1
            newP[f,0].A = newA1
            newP[f,0].invA = np.linalg.inv(newA1)

        newPP = np.tile(newP[:,0][:, np.newaxis], newP.shape[1])

        reproduce_time = time.time()
        start_point = np.array([[0], [frames_array[0].position.x], [frames_array[0].position.y], [frames_array[0].position.z]], dtype=object)
        rnew = TPGMM_model.reproduce(newPP, start_point[1:,:])
        self.get_logger().info(f"... reproduce_time: {time.time() - reproduce_time}")

        ## Saving and Publishing the Original GMM in Cartesian Space
        original_gmm = TPGMM_model.convertToGM(rnew, task_demons_info['down_sample_factor'], request.frame_id)
        self.tpgmm_viz_pub.publish(original_gmm)
        self.get_logger().info("Original GMM is Published!")
        response.original_gmm = original_gmm

        # Extract origianl means and covariances
        original_mu = torch.zeros((1, TPGMM_model.model.nbStates, 3), device=self.device, dtype=torch.float32)
        original_sigma = torch.zeros((1, TPGMM_model.model.nbStates, 3, 3), device=self.device, dtype=torch.float32)

        for k in range(TPGMM_model.model.nbStates):
            original_mu[0, k, 0] = rnew.Mu[1, k, -1]
            original_mu[0, k, 1] = rnew.Mu[2, k, -1]
            original_mu[0, k, 2] = rnew.Mu[3, k, -1]
            
            # Covariance is 4x4, extract the 3x3 spatial part
            cov_3x3 = rnew.Sigma[1:4, 1:4, k, -1]
            original_sigma[0, k, :, :] = torch.tensor(cov_3x3, device=self.device, dtype=torch.float32)

        if self.policy is not None:
            cov_diags = torch.diagonal(original_sigma, dim1=-2, dim2=-1)
            # gmm_features = torch.cat([original_mu.reshape(1, -1), cov_diags.reshape(1, -1)], dim=-1)
            gmm_features = original_mu.reshape(1, -1)
            
            sp = request.deformed_tpgmm_start_pose.pose
            sp_tensor = torch.tensor([[sp.position.x, sp.position.y, sp.position.z, sp.orientation.w, sp.orientation.x, sp.orientation.y, sp.orientation.z]], device=self.device, dtype=torch.float32)
            
            gp = request.deformed_tpgmm_goal_pose.pose
            gp_tensor = torch.tensor([[gp.position.x, gp.position.y, gp.position.z, gp.orientation.w, gp.orientation.x, gp.orientation.y, gp.orientation.z]], device=self.device, dtype=torch.float32)
            
            op = request.obstacle_pose.pose
            op_tensor = torch.tensor([[op.position.x, op.position.y, op.position.z]], device=self.device, dtype=torch.float32)
            
            orad = torch.tensor([[request.obstacle_radius]], device=self.device, dtype=torch.float32)
            
            des_clearance = torch.tensor([[request.desired_clearance]], device=self.device, dtype=torch.float32)
            
            obs = torch.cat((gmm_features, sp_tensor, gp_tensor, op_tensor, orad, des_clearance), dim=-1)

            with torch.no_grad():
                action = self.policy(obs)
            
            action_scale = self.action_scale # Has to be the same as set during training of the RL policy
            action_deltas = action.view(1, TPGMM_model.model.nbStates, 3) * action_scale
            deformed_mu = original_mu + action_deltas

            # Apply deformation to rnew across all time steps
            for k in range(TPGMM_model.model.nbStates):
                rnew.Mu[1, k, :] = deformed_mu[0, k, 0].item()
                rnew.Mu[2, k, :] = deformed_mu[0, k, 1].item()
                rnew.Mu[3, k, :] = deformed_mu[0, k, 2].item()
            
            # Recompute trajectory via GMR using deformed rnew.Mu
            rnew = TPGMM_model.recompute_trajectory(rnew, start_point[1:, :])

            self.get_logger().info("GMM has been deformed and trajectory recomputed successfully.")
        else:
            self.get_logger().warn("Policy is not loaded, returning original GMM")

        ## Saving and Publishing GMM in Cartesian Space
        gmm = TPGMM_model.convertToGM(rnew, task_demons_info['down_sample_factor'], request.frame_id)
        self.tpgmm_deformed_viz_pub.publish(gmm)
        self.get_logger().info("Deformed GMM is Published!")

        regressed_trajectory = PoseArray() ; regressed_trajectory.header.frame_id = request.frame_id
        regressed_point = Pose()
        for i, point in enumerate(rnew.Data.T):
            regressed_point.position.x = point[1]
            regressed_point.position.y = point[2]
            regressed_point.position.z = point[3]
            regressed_trajectory.poses.append(deepcopy(regressed_point))

        self.deformed_regress_traj_pub.publish(regressed_trajectory)
        self.get_logger().info("Deformed Regressed Trajectory is Published!")

        response.deformed_gmm = gmm
        return response

def main(args=None):
    rclpy.init(args=args)
    tpgmm = TPGMM()
    rclpy.spin(tpgmm)
    tpgmm.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
