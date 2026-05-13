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
from tp_gmm.srv import StartTPGMM, ReproduceTPGMM

import pickle

## System and directories stuff
import sys
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import os

pkg_share = get_package_share_directory('tp_gmm')
sys.path.append(os.path.join(pkg_share, 'include'))
sys.path.append(os.path.join(pkg_share, 'scripts'))
data_dir = os.path.join(pkg_share, 'data/') + '/'
scripts_dir = os.path.join(pkg_share, 'scripts/') + '/'
tasks_dir = os.path.join(pkg_share, 'tasks/') + '/'

from demons_to_samples import process_demonstrations

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

class TPGMM(Node):
    def __init__(self):
        super().__init__('tp_gmm_node')
        self.get_logger().info(" --> Node tp_gmm_node is initialized")
        self.TPGMM_model = None
        self.demons_info = None

        self.srv_start = self.create_service(StartTPGMM, "StartTPGMM_service", self.startTPGMM)
        self.srv_reproduce = self.create_service(ReproduceTPGMM, "ReproduceTPGMM_service", self.tpGMMGMR)

        self.regress_traj_pub = self.create_publisher(PoseArray, 'gmm/regressed_trajectory', 1)
        self.move_group_q_viz_pub = self.create_publisher(MoveGroup.Result, '/move_group/_action/status', 1) # Just a placeholder since old type is removed
        self.tpgmm_pub = self.create_publisher(GaussianMixture, '/gmm/mix', 1)
        self.tpgmm_viz_pub = self.create_publisher(GaussianMixture, '/gmm/cartesian_space', 1)
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

        ## Fetching Samples and paramters
        self.nbVar = 4      # Dim !!
        self.nbFrames = 2
        self.nbStates = 5  # nb of Gaussians

        if _train:
            self.get_logger().info("Processing demonstrations...")
            self.demons_info, slist = process_demonstrations(_task_name, self.nbFrames, self.nbStates, self.nbVar)
            self.get_logger().info(f"demons_info: {self.demons_info}")
        else:
            with open(tasks_dir + f'{_task_name}/demons_info.pkl', 'rb') as fp:
                self.demons_info = pickle.load(fp)
                self.get_logger().info(f"demons_info: {self.demons_info}")
            # If not training, we assume the model is already trained and available (or will be loaded in reproduction)
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

        # Creating instance of TPGMM_GMR
        TPGMMGMR = TPGMM_GMR(self.nbStates, self.nbFrames, self.nbVar)

        # Learning the model
        self.get_logger().info("Learning the TPGMM model...")
        TPGMMGMR.fit(self.slist)

        # Saving the model in .pkl as backup and in memory
        self.TPGMM_model = TPGMMGMR
        with open(tasks_dir + f'{_task_name}/TPGMM_model.pkl', 'wb') as fp:
            pickle.dump(self.TPGMM_model, fp)
        self.get_logger().info("TPGMM model trained and saved successfully.")

    def tpGMMGMR(self, request, response):

        _task_name = request.task_name #'pick'

        if self.TPGMM_model is not None:
            TPGMM_model = self.TPGMM_model
            self.get_logger().info("Using in-memory TPGMM model.")
        else:
            with open(tasks_dir + f'{_task_name}/TPGMM_model.pkl', 'rb') as fp:
                TPGMM_model = pickle.load(fp)
            self.get_logger().info("Loaded TPGMM model from disk.")

        if self.demons_info is not None:
            task_demons_info = self.demons_info
        else:
            with open(tasks_dir + f'{_task_name}/demons_info.pkl', 'rb') as fp:
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

def main(args=None):
    rclpy.init(args=args)
    tpgmm = TPGMM()
    rclpy.spin(tpgmm)
    tpgmm.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
