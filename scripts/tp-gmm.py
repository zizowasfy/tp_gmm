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

# System and directories stuff
import sys
sys.path.append("/home/zizo/tpgmm_rrt_ws/src/batteryDis-LfD/tp_gmm/include")
data_dir = "/home/zizo/tpgmm_rrt_ws/src/batteryDis-LfD/tp_gmm/data/"
scripts_dir = "/home/zizo/tpgmm_rrt_ws/src/batteryDis-LfD/tp_gmm/scripts/"
import nbformat
from nbconvert.preprocessors import ExecutePreprocessor
import pickle

# tpgmm-related stuff
from numba import njit
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
from data_handle.srv import *

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

        self.demonsToSamples_flag = False   
        self.demonsToSamples()


    def startTPGMM(self, req):

        ## Receiving service Request
        # if rospy.has_param("/task_param"):
        #     self.task_name = rospy.get_param("/task_param")
        #     print("/task_param: ", self.task_name)
        # else:
        # self.task_name = req.task_name
        # self.frame1_pose = req.start_pose
        # self.frame2_pose = req.goal_pose
        self.frame1_pose = Pose()
        self.frame2_pose = Pose()        
        self.task_name = rospy.get_param("/task_param")
        ## Fetching Samples and paramters
        self.demonsToSamples_flag = True
        print("startTPGMM")

        rospy.sleep(0.5)
        while not rospy.is_shutdown() and self.demonsToSamples_flag: pass   # wait until demonsToSamples finishes

        with open(scripts_dir + 'demons_info2.pkl', 'rb') as fp:
            self.demons_info2 = pickle.load(fp)
            print("demons_info2: ", self.demons_info2)

        ## Initialization of parameters and properties------------------------------------------------------------------------- #
        self.nbSamples = self.demons_info2['nbDemons']  # nb of demonstrations
        self.nbVar = 4      # Dim !!
        self.nbFrames = 2
        self.nbStates = 5  # nb of Gaussians
        self.nbData = self.demons_info2['ref_nbpoints']-1 # If the -1 is put in the DTW in demons_to_samples.ipynb, then -1 here has to be put too.
        self.down_sample_factor = self.demons_info2['down_sample_factor']

        self.tpGMM()
        return StartTPGMMResponse(True)
    
    ## Running the demons_to_samples.ipynb-------------------------------------------------------------------------------- #
    def demonsToSamples(self):
        # Waiting for a startTPGMM request - the while was the only way to do that to avoid 'no current event loop in thread' error
        while not rospy.is_shutdown() and not self.demonsToSamples_flag:
            # print("demonsToSamples")
            pass

        # Sending task_name to demons_to_samples.ipynb in .pkl file
        demons_info1 = {"task_name": self.task_name}
        with open(scripts_dir + 'demons_info1.pkl', 'wb') as fp:
            pickle.dump(demons_info1, fp)
            print("demons_info1: ", demons_info1)

        # NOTE: Uncomment/Run this only when need to train with new data
        ## COMMENTING this only to make the code run faster while debugging
        # Running demons_to_samples.ipynb
        # with open(scripts_dir + "demons_to_samples.ipynb") as f:
        #     nb_in = nbformat.read(f, as_version=4)
        # ep = ExecutePreprocessor(timeout=600, kernel_name='python3')
        # nb_out = ep.preprocess(nb_in)
        # print("demons_to_samples finished running!")
        self.demonsToSamples_flag = False

    ## Preparing the samples and fit ----------------------------------------------------------------------------------------- #
    def tpGMM(self):
        demons_nums = self.demons_info2['demons_nums']
        self.slist = []
        for i in range(self.nbSamples):
            pmat = np.empty(shape=(self.nbFrames, self.nbData), dtype=object)
            tempData = np.loadtxt(data_dir + demons_nums[i] + '_sample' + '_Data.txt', delimiter=',')
            print(tempData.shape)
            for j in range(self.nbFrames):
                tempA = np.loadtxt(data_dir + demons_nums[i] + '_sample' + '_frame' + str(j + 1) + '_A.txt', delimiter=',')
                tempB = np.loadtxt(data_dir + demons_nums[i] + '_sample' + '_frame' + str(j + 1) + '_b.txt', delimiter=',')

                for k in range(self.nbData):
                    pmat[j, k] = p(tempA[:, self.nbVar*k : self.nbVar*k + self.nbVar], tempB[:, k].reshape(len(tempB[:, k]), 1),
                                np.linalg.inv(tempA[:, self.nbVar*k : self.nbVar*k + self.nbVar]), self.nbStates)                         
            self.slist.append(s(pmat, tempData, tempData.shape[1], self.nbStates))

        # Creating instance of TPGMM_GMR-------------------------------------------------------------------------------------- #
        self.TPGMMGMR = TPGMM_GMR(self.nbStates, self.nbFrames, self.nbVar)

        # Learning the model-------------------------------------------------------------------------------------------------- #
        self.TPGMMGMR.fit(self.slist)
        
        # # Model Selection (nb of Gaussians selection) based on BIC ----------------------------------------------------------- #
        # Data_posearray, DataAll = self.TPGMMGMR.getDataAll(self.slist)
        # Data_posearray.header.frame_id = "panda_link0"
        # self.Data_posearray_pub.publish(Data_posearray)

        # self.tpGMMGMR()
    # @njit
    def tpGMMGMR(self, req):
        total_rep_time = time.time()

        # Sorting the Task Parameters into Frames format ----------------------------------------------------------------------------------------- #
        frames_array = [req.start_pose.pose, req.goal_pose.pose]
        frames_array.extend(req.obstacles_poses.poses)
        # print(frames_array)

        # Reproduction with generated parameters in Cartesian Space ------------------------------------------------------------------------------ #

        # self.frame1_pose = req.start_pose.pose
        # self.frame2_pose = req.goal_pose.pose    
        # # self.getFramePoses()
        newP = deepcopy(self.slist[self.demons_info2['demons_nums'].index(self.demons_info2['ref'])].p)

        print("newP.shape = ", newP.shape) # (3,47) = (nbFrames, nbData)

        # for f, frame in enumerate(frames_array):
        for f in range(self.nbFrames):
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
        rnew = self.TPGMMGMR.reproduce(newPP, start_point[1:,:])
        print("... reproduce_time: ", time.time() - reproduce_time)
        
        ## Saving and Publishing GMM in Cartesian Space
        gmm = self.TPGMMGMR.convertToGM(rnew, self.down_sample_factor, req.frame_id)
        self.tpgmm_viz_pub.publish(gmm)
        # print("GMM is Published!")
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
        finally:
            return ReproduceTPGMMResponse()


    ## Check if frame1_pose and frame2_pose hasn't been requested from startTPGMM rosservice, fill them with these values
    def getFramePoses(self):
        if (sqrt(self.frame1_pose.position.x**2 + self.frame1_pose.position.y**2 + self.frame1_pose.position.z**2) == 0):
            print("... Didn't receive a requested start_pose")
            self.frame1_pose.position.x, self.frame1_pose.position.y, self.frame1_pose.position.z = self.slist[self.demons_info2['demons_nums'].index(self.demons_info2['ref'])].p[0,0].b[1:,:]
            self.frame1_pose.position.x, self.frame1_pose.position.y, self.frame1_pose.position.z = self.frame1_pose.position.x[0], self.frame1_pose.position.y[0], self.frame1_pose.position.z[0]
            # TODO: Fill the .orientation after adding rotMat2Quat() function
            r = R.from_matrix(self.slist[self.demons_info2['demons_nums'].index(self.demons_info2['ref'])].p[0,0].A[1:,1:])
            self.frame1_pose.orientation.x, self.frame1_pose.orientation.y, self.frame1_pose.orientation.z, self.frame1_pose.orientation.w = r.as_quat()

        if (sqrt(self.frame2_pose.position.x**2 + self.frame2_pose.position.y**2 + self.frame2_pose.position.z**2) == 0):
            print(" ... Waiting on goal_pose from /clicked_point topic ... ")
            clkd_point = rospy.wait_for_message("/clicked_point", PointStamped)
            self.frame2_pose.position = clkd_point.point
            # TODO: Add .orientation after adding rotMat2Quat() function
            r = R.from_matrix(self.slist[self.demons_info2['demons_nums'].index(self.demons_info2['ref'])].p[1,0].A[1:,1:])
            self.frame2_pose.orientation.x, self.frame2_pose.orientation.y, self.frame2_pose.orientation.z, self.frame2_pose.orientation.w = r.as_quat()

if __name__ == "__main__":

    tpgmm = TPGMM()
    rospy.spin()