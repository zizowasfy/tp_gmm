#! /usr/bin/env python3
import rospy
import rosbag

from std_msgs.msg import Float32
from geometry_msgs.msg import PoseArray, Pose, PoseStamped, Quaternion
from tp_gmm.msg import *
from tp_gmm.srv import *

import sys
import argparse
import random
from copy import deepcopy
from math import *
from scipy.spatial.transform import Rotation as R

DEMONS_DIR = "/home/zizo/Disassembly Teleop/Demons/"
LL_DIR = "/home/zizo/Disassembly Teleop/LL/"

class ExpHandler:
    # def __init__(self, args):
    def __init__(self):
        self.task_name = rospy.get_param("/task_param")
        self.subtask_name = rospy.get_param("/subtask_param")
        # self.trainGMM = args.trainGMM

        self.start_pub = rospy.Publisher("/start_pose", PoseStamped, queue_size=1)
        self.goal_pub = rospy.Publisher("/goal_pose", PoseStamped, queue_size=1)
        
        rospy.sleep(0.5) # !!Important to add to allow subscribers enough time to connect to published topics

        self.nbexp = 1      # number of experiments to implement
        self.countexp = 1   # experiments counter

        # self.getDemonsInfo()
        # self.setStartnGoal(publish_goal=False)    # Send tp_gmm node to get the trained TPGMM
        self.startExperiments()
    
    def EulerToQuat(self, roll, pitch, yaw):
        print("Z = ", degrees(yaw))
        cr = cos(roll * 0.5)
        sr = sin(roll * 0.5)
        cp = cos(pitch * 0.5)
        sp = sin(pitch * 0.5)
        cy = cos(yaw * 0.5)
        sy = sin(yaw * 0.5)

        q = Quaternion()
        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy

        return q

    def RottoQuat(self, mat):
        r = R.from_matrix([[mat[0], mat[4], mat[8]],
                        [mat[1], mat[5], mat[9]], 
                        [mat[2], mat[6], mat[10]]])
        return r.as_quat()

    def setStartnGoal(self, publish_goal=True):


        start_pose = PoseStamped()
        start_pose.header.frame_id = "base_link" #"panda_link0"
        start_pose.pose.position.x = random.uniform(0.55, 0.75)
        start_pose.pose.position.y = random.uniform(-0.47, 0.47)
        start_pose.pose.position.z = random.uniform(0.4, 1.0)
        # start_pose.pose.position.x = 0.4885305099837242
        # start_pose.pose.position.y = -0.2166985163726255
        # start_pose.pose.position.z = 0.49258181439255067
        # start_pose.pose.orientation.x = 1
        # start_pose.pose.orientation.y = 0
        # start_pose.pose.orientation.z = 0
        # start_pose.pose.orientation.w = 0
        # q = self.EulerToQuat(radians(180), radians(0), radians(random.randint(-70,70))) # roll (x), pitch (y), yaw (z)
        q = self.EulerToQuat(radians(0), radians(90), radians(0)) # roll (x), pitch (y), yaw (z)
        start_pose.pose.orientation.x = q.x # 1
        start_pose.pose.orientation.y = q.y # 0
        start_pose.pose.orientation.z = q.z # 0
        start_pose.pose.orientation.w = q.w # 0
        # self.start_pub.publish(start_pose)
        # print(start_pose)
        self.start_pose = PoseStamped(); self.start_pose = start_pose
        self.start_pub.publish(start_pose)

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "base_link" #"panda_link0"
        goal_pose.pose.position.x = random.uniform(0.4, 0.65)
        goal_pose.pose.position.y = random.uniform(-0.37, 0.37)
        goal_pose.pose.position.z = random.uniform(0.18, 0.1) + 0.0 # random.uniform(0.08, 0.1) + 0.1
        # goal_pose.pose.position.x = 0.2531102379156528
        # goal_pose.pose.position.y = 0.3635753608782901
        # goal_pose.pose.position.z = 0.09641878103935082
        # goal_pose.pose.orientation.x = 0 #1
        # goal_pose.pose.orientation.y = 0.707 #0
        # goal_pose.pose.orientation.z = 0
        # goal_pose.pose.orientation.w = 0.707 #0              
        q = self.EulerToQuat(radians(0), radians(90), radians(0)) # roll (x), pitch (y), yaw (z)
        goal_pose.pose.orientation.x = q.x # 1
        goal_pose.pose.orientation.y = q.y # 0
        goal_pose.pose.orientation.z = q.z # 0
        goal_pose.pose.orientation.w = q.w # 0
        self.goal_pose = PoseStamped(); self.goal_pose = goal_pose   
        if publish_goal:
            self.goal_pub.publish(goal_pose)
            # print(goal_pose)


        rospy.wait_for_service('ReproduceTPGMM_service')
        reproduceTPGMM_req = rospy.ServiceProxy('ReproduceTPGMM_service', ReproduceTPGMM)
        reproduceTPGMM_req(start_pose, goal_pose)
        return None



    def startExperiments(self):
        # rrt_planner = False # Default: False i.e. plan with tpgmm-rrt planner
        # selected_exp_numbers = [1, 2, 3, 4, 5, 9, 12, 13, 16, 20]
        # for exp in selected_exp_numbers:
        #     self.countexp = exp
        while (self.countexp < self.nbexp + 1):
            print("______________")
            print("Experiment %d" % self.countexp)
            self.setStartnGoal()
            rospy.sleep(1)
            self.countexp += 1

        rospy.signal_shutdown("Node is Shutting Down!")

        return None

if __name__ == '__main__':
    # parser = argparse.ArgumentParser(prog='Experiment Handler Node',
    #     description='Handles the start and end poses, saves learned GMM and trajectories')
    # parser.add_argument('trainGMM', help='(T/F) if called, GMM (gmm_node) will be trained')
    # # parser.set_defaults(trainGMM=False)
    # args = parser.parse_args()
    
    # print(args.trainGMM)

    rospy.init_node('ExperimentHandler')
    # ExpHandler(args)
    ExpHandler() 
    rospy.spin()