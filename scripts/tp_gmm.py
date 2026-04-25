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
from nbclient import NotebookClient
import nbformat
import papermill as pm

## System and directories stuff
import sys
from pathlib import Path

TPGMM_DIR = Path(__file__).resolve().parent.parent
sys.path.append(str(TPGMM_DIR / 'include'))
data_dir = str(TPGMM_DIR / 'data/') + '/'
scripts_dir = str(TPGMM_DIR / 'scripts') + '/'
tasks_dir = str(TPGMM_DIR / 'tasks') + '/'

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
        if _train:
            self.demonsToSamples(_task_name)
            time.sleep(0.5)

        with open(tasks_dir + f'{_task_name}/demons_info.pkl', 'rb') as fp:
            self.demons_info = pickle.load(fp)
            self.get_logger().info(f"demons_info: {self.demons_info}")

        ## Initialization of parameters and properties
        self.nbSamples = self.demons_info['nbDemons']  # nb of demonstrations
        self.nbVar = 4      # Dim !!
        self.nbFrames = 2
        self.nbStates = 5  # nb of Gaussians
        self.nbData = self.demons_info['ref_nbpoints']#-1
        self.down_sample_factor = self.demons_info['down_sample_factor']

        self.tpGMM(_task_name)
        response.started = True
        return response

    ## Running the demons_to_samples.ipynb
    def demonsToSamples(self, _task_name):
        params = {"task_name": _task_name}
        self.get_logger().info("Running demons_to_samples_.ipynb ...")
        pm.execute_notebook(input_path=scripts_dir + "demons_to_samples_ur10_demons.ipynb",
                            output_path=scripts_dir + "demons_to_samples_ur10_demons_output.ipynb",
                            parameters=params)
        self.get_logger().info("demons_to_samples finished running!")

    ## Preparing the samples and fit
    def tpGMM(self, _task_name):
        demons_nums = self.demons_info['demons_nums']
        self.slist = []
        for i in range(self.nbSamples):
            pmat = np.empty(shape=(self.nbFrames, self.nbData), dtype=object)
            tempData = np.loadtxt(data_dir + f'{_task_name}/' + demons_nums[i] + '_sample' + '_Data.txt', delimiter=',')
            self.get_logger().info(f"{tempData.shape}")
            for j in range(self.nbFrames):
                tempA = np.loadtxt(data_dir + f'{_task_name}/' + demons_nums[i] + '_sample' + '_frame' + str(j + 1) + '_A.txt', delimiter=',')
                tempB = np.loadtxt(data_dir + f'{_task_name}/' + demons_nums[i] + '_sample' + '_frame' + str(j + 1) + '_b.txt', delimiter=',')

                for k in range(self.nbData):
                    pmat[j, k] = p(tempA[:, self.nbVar*k : self.nbVar*k + self.nbVar], tempB[:, k].reshape(len(tempB[:, k]), 1),
                                np.linalg.inv(tempA[:, self.nbVar*k : self.nbVar*k + self.nbVar]), self.nbStates)
            self.slist.append(s(pmat, tempData, tempData.shape[1], self.nbStates))

        # Creating instance of TPGMM_GMR
        TPGMMGMR = TPGMM_GMR(self.nbStates, self.nbFrames, self.nbVar)

        # Learning the model
        TPGMMGMR.fit(self.slist)

        # Saving the model in .pkl
        model_file = TPGMMGMR
        with open(tasks_dir + f'{_task_name}/TPGMM_model.pkl', 'wb') as fp:
            pickle.dump(model_file, fp)

    def tpGMMGMR(self, request, response):

        _task_name = request.task_name #'pick'

        with open(tasks_dir + f'{_task_name}/TPGMM_model.pkl', 'rb') as fp:
            TPGMM_model = pickle.load(fp)

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
