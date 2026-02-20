#!/usr/bin/env python3
# ROS stuff
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose, PointStamped, PoseArray
from sensor_msgs.msg import JointState
# from gaussian_mixture_model.msg import GaussianMixture
from tp_gmm.msg import GaussianMixture
from tp_gmm.srv import *
from moveit_msgs.msg import MoveGroupActionResult
from trajectory_msgs.msg import JointTrajectoryPoint

import pickle
from nbclient import NotebookClient
import nbformat
import papermill as pm

## System and directories stuff
import sys
from pathlib import Path

TPGMM_DIR = Path(__file__).resolve().parent.parent
# print(str(TPGMM_DIR))
sys.path.append(str(TPGMM_DIR / 'include'))
data_dir = str(TPGMM_DIR / 'data/') + '/'
scripts_dir = str(TPGMM_DIR / 'scripts') + '/'
tasks_dir = str(TPGMM_DIR / 'tasks') + '/'
# print(sys.path)

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

class TPGMM:
    def __init__(self):

        rospy.init_node("tp_gmm_node")
        print(" --> Node tp_gmm_node is initialized")
        rospy.Service("StartTPGMM_service", StartTPGMM, self.startTPGMM)
        rospy.Service("ReproduceTPGMM_service", ReproduceTPGMM, self.tpGMMGMR)

        self.regress_traj_pub = rospy.Publisher('gmm/regressed_trajectory', PoseArray, queue_size=1)
        self.move_group_q_viz_pub = rospy.Publisher('/move_group/result', MoveGroupActionResult, queue_size=1)
        self.tpgmm_pub = rospy.Publisher('/gmm/mix', GaussianMixture, queue_size=1)
        self.tpgmm_viz_pub = rospy.Publisher('/gmm/cartesian_space', GaussianMixture, queue_size=1)
        self.learned_traj_pub = rospy.Publisher('/gmm/learned_trajectory', PoseArray, queue_size=1)
        self.solveFK_pub = rospy.Publisher('/joint_samples', JointState, queue_size=1)
        self.tpgmm_time_pub = rospy.Publisher('/tpgmm/planning_time', Float32, queue_size=1)
        self.Data_posearray_pub = rospy.Publisher('/gmm/traj_input', PoseArray, queue_size=1)

        # self.demonsToSamples_flag = False
        # self.demonsToSamples()


    def startTPGMM(self, req):
        ## Receiving service Request

        self.frame1_pose = Pose()
        self.frame2_pose = Pose()        
        _task_name = req.task_name #'pick'
        _train = True #req.train

        ## Fetching Samples and paramters

        ## The if condition here is to run demonsToSamples only when training is required, otherwise skip directly to load the TPGMM model
        if _train:
            self.demonsToSamples(_task_name)
            rospy.sleep(0.5)

        # self.demonsToSamples_flag = True
        # print("startTPGMM")
        # rospy.sleep(0.5)
        # while not rospy.is_shutdown() and self.demonsToSamples_flag: pass   # wait until demonsToSamples finishes

        with open(tasks_dir + f'{_task_name}/demons_info.pkl', 'rb') as fp:
            self.demons_info = pickle.load(fp)
            print("demons_info: ", self.demons_info)

        ## Initialization of parameters and properties------------------------------------------------------------------------- #
        self.nbSamples = self.demons_info['nbDemons']  # nb of demonstrations
        self.nbVar = 4      # Dim !!
        self.nbFrames = 2
        self.nbStates = 5  # nb of Gaussians
        self.nbData = self.demons_info['ref_nbpoints']#-1 # If the -1 is put in the DTW in demons_to_samples.ipynb, then -1 here has to be put too.
        self.down_sample_factor = self.demons_info['down_sample_factor']

        self.tpGMM(_task_name)
        return StartTPGMMResponse(True)
    
    ## Running the demons_to_samples.ipynb-------------------------------------------------------------------------------- #
    def demonsToSamples(self, _task_name):
        # # Waiting for a startTPGMM request - the while was the only way to do that to avoid 'no current event loop in thread' error
        # while not rospy.is_shutdown() and not self.demonsToSamples_flag:
        #     # print("demonsToSamples")
        #     pass

        ## Sending _task_name to demons_to_samples.ipynb in .pkl file
        ## UPDATE: No need for this when using papermill to execute the notebook as it pases '_task_name' as a parameter
        # task_name_file = {"_task_name": _task_name}
        # with open(tasks_dir + '_task_name.pkl', 'wb') as fp:
        #     pickle.dump(task_name_file, fp)
        #     print("task_name_file: ", task_name_file)

        # NOTE: Uncomment/Run this only when need to train with new data
        ### COMMENTING this only to make the code run faster while debugging
        ## Running demons_to_samples.ipynb
        # print("Running demons_to_samples_.ipynb ...")
        # with open(scripts_dir + "demons_to_samples_ur10_demons.ipynb") as f:
        #     nb_in = nbformat.read(f, as_version=4)
        # ep = ExecutePreprocessor(timeout=600, kernel_name='python3')
        # nb_out = ep.preprocess(nb_in)
        # print("demons_to_samples finished running!")
        # # self.demonsToSamples_flag = False
        ###
        # print("Running demons_to_samples_.ipynb ...")
        # with open(scripts_dir + "demons_to_samples_ur10_demons.ipynb", encoding="utf-8") as f:
        #     nb = nbformat.read(f, as_version=4)
        # client = NotebookClient(nb, timeout=600, kernel_name='python3')
        #                         # metadata={"path": scripts_dir})
        # client.execute()
        # print("demons_to_samples finished running!")
        ###
        params = {"task_name": _task_name}
        print("Running demons_to_samples_.ipynb ...")
        pm.execute_notebook(input_path=scripts_dir + "demons_to_samples_ur10_demons.ipynb",
                            output_path=scripts_dir + "demons_to_samples_ur10_demons_output.ipynb",
                            parameters=params)
        print("demons_to_samples finished running!")

        # # Optionally save the output
        # output_path = scripts_dir + "executed_demons_to_samples_ur10_demons.ipynb"
        # with open(output_path, "w", encoding="utf-8") as f:
        #     nbformat.write(nb, f)

    ## Preparing the samples and fit ----------------------------------------------------------------------------------------- #
    def tpGMM(self, _task_name):
        demons_nums = self.demons_info['demons_nums']
        self.slist = []
        for i in range(self.nbSamples):
            pmat = np.empty(shape=(self.nbFrames, self.nbData), dtype=object)
            tempData = np.loadtxt(data_dir + f'{_task_name}/' + demons_nums[i] + '_sample' + '_Data.txt', delimiter=',')
            print(tempData.shape)
            for j in range(self.nbFrames):
                tempA = np.loadtxt(data_dir + f'{_task_name}/' + demons_nums[i] + '_sample' + '_frame' + str(j + 1) + '_A.txt', delimiter=',')
                tempB = np.loadtxt(data_dir + f'{_task_name}/' + demons_nums[i] + '_sample' + '_frame' + str(j + 1) + '_b.txt', delimiter=',')

                for k in range(self.nbData):
                    pmat[j, k] = p(tempA[:, self.nbVar*k : self.nbVar*k + self.nbVar], tempB[:, k].reshape(len(tempB[:, k]), 1),
                                np.linalg.inv(tempA[:, self.nbVar*k : self.nbVar*k + self.nbVar]), self.nbStates)
            self.slist.append(s(pmat, tempData, tempData.shape[1], self.nbStates))

        # Creating instance of TPGMM_GMR-------------------------------------------------------------------------------------- #
        # self.TPGMMGMR = TPGMM_GMR(self.nbStates, self.nbFrames, self.nbVar)
        TPGMMGMR = TPGMM_GMR(self.nbStates, self.nbFrames, self.nbVar)

        # Learning the model-------------------------------------------------------------------------------------------------- #
        # self.TPGMMGMR.fit(self.slist)
        TPGMMGMR.fit(self.slist)

        # Saving the model in .pkl -------------------------------------------------------------------------------------------------- #
        model_file = TPGMMGMR
        with open(tasks_dir + f'{_task_name}/TPGMM_model.pkl', 'wb') as fp:
            pickle.dump(model_file, fp)
        # print("TPGMMGMR.s: ", TPGMMGMR.s)

        # # Model Selection (nb of Gaussians selection) based on BIC ----------------------------------------------------------- #
        # Data_posearray, DataAll = self.TPGMMGMR.getDataAll(self.slist)
        # Data_posearray.header.frame_id = "panda_link0"
        # self.Data_posearray_pub.publish(Data_posearray)

        # self.tpGMMGMR()
    # @njit
    def tpGMMGMR(self, req):

        _task_name = req.task_name #'pick'

        with open(tasks_dir + f'{_task_name}/TPGMM_model.pkl', 'rb') as fp:
            TPGMM_model = pickle.load(fp)

        with open(tasks_dir + f'{_task_name}/demons_info.pkl', 'rb') as fp:
            task_demons_info = pickle.load(fp)

        # Sorting the Task Parameters into Frames format ----------------------------------------------------------------------------------------- #
        frames_array = [req.start_pose.pose, req.goal_pose.pose]
        # frames_array.extend(req.obstacles_poses.poses) # Use this when obstacles are used in the TPGMM
        frames_array.extend(PoseArray().poses) # Use this when no obstacles are used in the TPGMM
        # print(frames_array)

        # Reproduction with generated parameters in Cartesian Space ------------------------------------------------------------------------------ #

        # self.frame1_pose = req.start_pose.pose
        # self.frame2_pose = req.goal_pose.pose
        # # self.getFramePoses()
        # newP = deepcopy(self.slist[self.demons_info['demons_nums'].index(self.demons_info['ref'])].p)
        newP = deepcopy(TPGMM_model.s[task_demons_info['demons_nums'].index(task_demons_info['ref'])].p)

        print("newP.shape = ", newP.shape) # (3,47) = (nbFrames, nbData)

        # for f, frame in enumerate(frames_array):
        for f in range(task_demons_info['nbFrames']):
            ## This snippet is to move the center point of the obstale as if it lies in the top of the obstacle of z height 0.2. This is just a quick fix to make the GMM avoid colliding with the virtual obstacle in RViz
            ## This can be properly fixed by taking proper demons and emphasize of the obstacle avoidance.
            if f > 1:
                newb1 = np.array([[0], [frames_array[f].position.x], [frames_array[f].position.y], [frames_array[f].position.z+0.1]], dtype=object)
            else:
                newb1 = np.array([[0], [frames_array[f].position.x], [frames_array[f].position.y], [frames_array[f].position.z]], dtype=object)
            ##\ This snippet is to move the center of the obstale as if it lies in the top of the obstacle of z height 0.2. This is just a quick fix!

            # newb1 = np.array([[0], [frames_array[f].position.x], [frames_array[f].position.y], [frames_array[f].position.z]], dtype=object)
            # # newb2 = np.array([[0], [self.frame2_pose.position.x], [self.frame2_pose.position.y], [self.frame2_pose.position.z]], dtype=object)

            rA1 = R.from_quat([frames_array[f].orientation.x, frames_array[f].orientation.y, frames_array[f].orientation.z, frames_array[f].orientation.w])
            # rA2 = R.from_quat([self.frame2_pose.orientation.x, self.frame2_pose.orientation.y, self.frame2_pose.orientation.z, self.frame2_pose.orientation.w])
            newA1 = np.vstack(( np.array([1,0,0,0]), np.hstack(( np.zeros((3,1)), rA1.as_matrix() )) )) # TODO: Quat2rotMat
            # newA2 = np.vstack(( np.array([1,0,0,0]), np.hstack(( np.zeros((3,1)), rA2.as_matrix() )) )) # TODO: Quat2rotMat

            # newP_loop_time = time.time()
            # for k in range(self.nbData):
            #     newP[0, k].b = newb1
            #     newP[1, k].b = newb2
            #     newP[0, k].A = newA1
            #     newP[1, k].A = newA2
            #     newP[0, k].invA = np.linalg.inv(newA1) # TOTRY: with and without invA
            #     newP[1, k].invA = np.linalg.inv(newA2) # TOTRY: with and without invA
            # print("... newP_loop_time: ", time.time() - newP_loop_time)

            # newP_tile_time = time.time()
            newP[f,0].b = newb1
            # newP[1,0].b = newb2
            newP[f,0].A = newA1
            # newP[1,0].A = newA2
            newP[f,0].invA = np.linalg.inv(newA1)
            # newP[1,0].invA = np.linalg.inv(newA2)

        newPP = np.tile(newP[:,0][:, np.newaxis], newP.shape[1])
        # print("... newPP_loop_time: ", time.time() - newP_tile_time)

        reproduce_time = time.time()
        start_point = np.array([[0], [frames_array[0].position.x], [frames_array[0].position.y], [frames_array[0].position.z]], dtype=object)
        # rnew = self.TPGMMGMR.reproduce(newPP, start_point[1:,:])
        rnew = TPGMM_model.reproduce(newPP, start_point[1:,:])
        print("... reproduce_time: ", time.time() - reproduce_time)

        ## Saving and Publishing GMM in Cartesian Space
        # gmm = self.TPGMMGMR.convertToGM(rnew, self.down_sample_factor, req.frame_id)
        gmm = TPGMM_model.convertToGM(rnew, task_demons_info['down_sample_factor'], req.frame_id)
        self.tpgmm_viz_pub.publish(gmm)
        print("GMM is Published!")
        # rospy.sleep(3)

        regressed_trajectory = PoseArray() ; regressed_trajectory.header.frame_id = req.frame_id #'base_link'
        regressed_point = Pose()
        for i, point in enumerate(rnew.Data.T): # Looping over the points (colums of Data) in renew.Data
            regressed_point.position.x = point[1]
            regressed_point.position.y = point[2]
            regressed_point.position.z = point[3]

            regressed_trajectory.poses.append(deepcopy(regressed_point))
        print("No. of points in Regressed Trajectory: ", i)
        self.regress_traj_pub.publish(regressed_trajectory)
        print("Regressed Trajectory is Published!")

        # self.tpGMMPlot()
        # rospy.signal_shutdown("TP-GMM Node is Shutting Down!")
        return ReproduceTPGMMResponse()


    ## Check if frame1_pose and frame2_pose hasn't been requested from startTPGMM rosservice, fill them with these values
    def getFramePoses(self):
        if (sqrt(self.frame1_pose.position.x**2 + self.frame1_pose.position.y**2 + self.frame1_pose.position.z**2) == 0):
            print("... Didn't receive a requested start_pose")
            self.frame1_pose.position.x, self.frame1_pose.position.y, self.frame1_pose.position.z = self.slist[self.demons_info['demons_nums'].index(self.demons_info['ref'])].p[0,0].b[1:,:]
            self.frame1_pose.position.x, self.frame1_pose.position.y, self.frame1_pose.position.z = self.frame1_pose.position.x[0], self.frame1_pose.position.y[0], self.frame1_pose.position.z[0]
            # TODO: Fill the .orientation after adding rotMat2Quat() function
            r = R.from_matrix(self.slist[self.demons_info['demons_nums'].index(self.demons_info['ref'])].p[0,0].A[1:,1:])
            self.frame1_pose.orientation.x, self.frame1_pose.orientation.y, self.frame1_pose.orientation.z, self.frame1_pose.orientation.w = r.as_quat()

        if (sqrt(self.frame2_pose.position.x**2 + self.frame2_pose.position.y**2 + self.frame2_pose.position.z**2) == 0):
            print(" ... Waiting on goal_pose from /clicked_point topic ... ")
            clkd_point = rospy.wait_for_message("/clicked_point", PointStamped)
            self.frame2_pose.position = clkd_point.point
            # TODO: Add .orientation after adding rotMat2Quat() function
            r = R.from_matrix(self.slist[self.demons_info['demons_nums'].index(self.demons_info['ref'])].p[1,0].A[1:,1:])
            self.frame2_pose.orientation.x, self.frame2_pose.orientation.y, self.frame2_pose.orientation.z, self.frame2_pose.orientation.w = r.as_quat()

if __name__ == "__main__":

    tpgmm = TPGMM()
    rospy.spin()