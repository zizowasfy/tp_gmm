#!/usr/bin/env python3

######
# This script is written to record trajectories (which are used as demonstrations) for tpgmm training, using the real teleop panda-panda.
######

# Python 2/3 compatibility imports
from __future__ import print_function
from six.moves import input

import sys
from copy import deepcopy
import random
from math import *
import numpy as np

import rospy
import rosbag
import tf
import moveit_commander
from moveit_msgs.msg import MoveGroupActionResult, BoundingVolume, Constraints, PositionConstraint
from moveit_msgs.srv import GetPositionFK
from geometry_msgs.msg import Pose, PoseStamped, PoseArray, Quaternion
from std_msgs.msg import String, Float32MultiArray
from franka_msgs.msg import FrankaState
from sensor_msgs.msg import JointState
from tp_gmm.srv import ReproduceTPGMM

from moveit_commander.conversions import pose_to_list
from scipy.spatial.transform import Rotation as R

# sys.path.insert(0, '/home/zizo/tpgmm_rrt_ws/src')
# from motion_planning.scripts import kinematics

# try:
#     from math import pi, tau, dist, fabs, cos
# except:  # For Python 2 compatibility
#     from math import pi, fabs, cos, sqrt

#     tau = 2.0 * pi

#     def dist(p, q):
#         return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


## END_SUB_TUTORIAL

save_dir = "/home/zizo/Data_dir/panda_real_demons/"

def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if the values in two lists are within a tolerance of each other.
    For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle
    between the identical orientations q and -q is calculated correctly).
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True


class MoveGroupPythonInterfaceTutorial(object):
    """MoveGroupPythonInterfaceTutorial"""

    def __init__(self, ns, gn, fi):
        super(MoveGroupPythonInterfaceTutorial, self).__init__()

        ## BEGIN_SUB_TUTORIAL setup
        ##
        ## First initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_group_python_interface_tutorial", anonymous=True)

        ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
        ## kinematic model and the robot's current joint states

        # robot = moveit_commander.RobotCommander(robot_description= ns + "/robot_description", ns=ns)
        robot = moveit_commander.RobotCommander(robot_description= ns + "robot_description", ns=ns)

        ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
        ## for getting, setting, and updating the robot's internal understanding of the
        ## surrounding world:
        scene = moveit_commander.PlanningSceneInterface()

        ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
        ## to a planning group (group of joints).  In this tutorial the group is the primary
        ## arm joints in the Panda robot, so we set the group's name to "panda_arm".
        ## If you are using a different robot, change this value to the name of your robot
        ## arm planning group.
        ## This interface can be used to plan and execute motions:
        group_name = gn
        # move_group = moveit_commander.MoveGroupCommander(group_name, robot_description= ns + "/robot_description", ns=ns[1:], wait_for_servers=20)
        move_group = moveit_commander.MoveGroupCommander(group_name, robot_description= ns + "robot_description", ns=ns[1:], wait_for_servers=20)

        ## END_SUB_TUTORIAL

        ## BEGIN_SUB_TUTORIAL basic_info
        ##
        ## Getting Basic Information
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^

        # We can get the name of the reference frame for this robot:
        move_group.set_pose_reference_frame(fi )
        planning_frame = move_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)
        print("============ Pose reference frame: %s" % move_group.get_pose_reference_frame())

        # We can also print the name of the end-effector link for this group:
        eef_link = move_group.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        print("============ Available Planning Groups:", robot.get_group_names())

        move_group.set_planning_time(5.0)
        print("============ Planning Timeout:", move_group.get_planning_time())

        move_group.set_planner_id("RRT")
        move_group.set_max_velocity_scaling_factor(1.0)
        move_group.set_max_acceleration_scaling_factor(1.0)
        # print("============ Planning Attempts:", move_group.get_num_planning_attempts())

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        # print("============ Printing robot state")
        # print(robot.get_current_state())
        print("")
        ## END_SUB_TUTORIAL


        ## Variables
        self.ns = ns
        self.frame_id = fi
        self.box_name = ""
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

        self.demon_rosbag = None
        self.start_pose = None
        self.goal_pose = None
        self.record_bags = False
        self.save_bag = True
        self.demon_num = 1
        self.gmm_bounding_volume = None

        ## Publishers and Subscribers
        self.start_pose_pub = rospy.Publisher(ns + "/start_pose", PoseStamped, queue_size=1)
        self.goal_pose_pub = rospy.Publisher(ns + "/goal_pose", PoseStamped, queue_size=1)
        # self.posearray_pub = rospy.Publisher(ns + "/planned_trajectory/posearray", PoseArray, queue_size=1)
        self.obstacles_pub = rospy.Publisher("/obstacles", PoseArray, queue_size=1)

        # rospy.Subscriber(ns + "/panda_teleop/follower_state_controller/franka_states", FrankaState, self.frankaStates_callback)
        # rospy.Subscriber(ns + "/panda_teleop/follower_state_controller/joint_states", JointState, self.jointStates_callback)
        rospy.Subscriber("/gmm_moveit", BoundingVolume, self.get_gmm_constraint)

        self.reproduceTPGMM_req = rospy.ServiceProxy('ReproduceTPGMM_service', ReproduceTPGMM)

        rospy.sleep(0.5)


    # def frankaStates_callback(self, msg):
    #     if self.record_bags:
    #         self.demon_rosbag.write(self.ns + "/panda_teleop/follower_state_controller/franka_states", msg)

    #     return None

    # def jointStates_callback(self, msg):
    #     if self.record_bags:
    #         self.demon_rosbag.write(self.ns + "/panda_teleop/follower_state_controller/joint_states", msg)
    #         # print("joint_states", msg.position[0])
    #     return None

    # def setObstacles(self):
    def setObstacles(self, X, Y, Z):
        ## reset/remove all obstacles in the planning scene
        self.scene.remove_world_object()

        # nb_obstacles = random.randint(1,5)
        nb_obstacles = 1
        self.obstacles_posesarray = PoseArray()
        self.obstacles_posesarray.header.frame_id = self.frame_id
        for obs in range(nb_obstacles):
            obstacle_pose = PoseStamped()
            obstacle_pose.header.frame_id = self.frame_id
            obstacle_pose.pose.position.x = random.uniform(X[0], X[1]) # random.uniform(0.35, 0.55)
            obstacle_pose.pose.position.y = random.uniform(Y[0], Y[1]) # random.uniform(-0.27, 0.27)
            obstacle_pose.pose.position.z = random.uniform(Z[0], Z[1]) # random.uniform(0.15, 0.4) # (0.08, 0.5)
            # q = self.EulerToQuat(radians(0), radians(0), radians(random.randint(-70,70))) # roll (x), pitch (y), yaw (z)      (batch4)
            q = self.EulerToQuat(radians(180), radians(0), radians(random.randint(-70,70))) # roll (x), pitch (y), yaw (z)      (batch5)

            obstacle_pose.pose.orientation.x = q.x # 1
            obstacle_pose.pose.orientation.y = q.y # 0
            obstacle_pose.pose.orientation.z = q.z # 0
            obstacle_pose.pose.orientation.w = q.w # 0
            self.obstacles_posesarray.poses.append(deepcopy(obstacle_pose.pose))

            self.scene.add_box("obstacle_{}".format(obs), obstacle_pose, size=(0.15, 0.05, 0.2)) #(0.1, 0.05, 0.15)

        self.obstacles_pub.publish(self.obstacles_posesarray)

    def setStartnGoalFromRosbag(self, exp_number):

        saved_exps_dir = save_dir + "saved_exps/" + "demon_{}.bag".format(exp_number)

        exp_rbag = rosbag.Bag(saved_exps_dir, 'r')

        exp_dict = {}
        for topic, msg, t in exp_rbag.read_messages():
            # print(topic)
            # print(msg)
            exp_dict[topic] = msg
        # print(exp_dict)
        exp_rbag.close()

        # NOTE: Had to create a new message and manually fill it because reading from rosbags load the msgs dynamically (i.e. under different namespace) which some functions (in move_group interface) do not accept.
        start_pose = PoseStamped()
        start_pose.header = exp_dict[self.ns + '/start_pose'].header
        start_pose.pose = exp_dict[self.ns + '/start_pose'].pose
        self.start_pose_pub.publish(start_pose)

        goal_pose = PoseStamped()
        goal_pose.header = exp_dict[self.ns + '/goal_pose'].header
        goal_pose.pose = exp_dict[self.ns + '/goal_pose'].pose
        self.goal_pose_pub.publish(goal_pose)

        self.obstacles_posesarray = exp_dict[self.ns + '/obstacles']

        pose = PoseStamped()
        # self.move_group.set_pose_target(pose)

        ## debugging
        # print(type(start_pose))
        # print(type(pose))
        ##\ debugging

        ## reset/remove all obstacles in the planning scene
        self.scene.remove_world_object()

        for obs, obstacle in enumerate(self.obstacles_posesarray.poses):
            obstacle_pose = PoseStamped()
            obstacle_pose.header.frame_id = self.frame_id
            obstacle_pose.pose = obstacle

            self.scene.add_box("obstacle_{}".format(obs), obstacle_pose, size=(0.15, 0.05, 0.2)) #(0.1, 0.05, 0.15)

        return start_pose, goal_pose

    def setStartnGoalPoses(self):
        # self.setObstacles()

        ## Setting start & goal poses
        start_pose = PoseStamped()
        start_pose.header.frame_id = self.frame_id
        start_pose.pose.position.x = random.uniform(0.35, 0.55)
        start_pose.pose.position.y = random.uniform(-0.43, 0.0) #(-0.47, 0.47)
        start_pose.pose.position.z = random.uniform(0.5, 0.6)
        q = self.EulerToQuat(radians(180), radians(0), radians(random.randint(-70,70))) # roll (x), pitch (y), yaw (z) # panda
        # q = self.EulerToQuat(radians(0), radians(0), radians(random.randint(-70,70))) # roll (x), pitch (y), yaw (z) # ur10

        # q = self.EulerToQuat(radians(180), radians(0), radians(random.randint(-70,70))) # roll (x), pitch (y), yaw (z) # ur10_tool0
        # q = self.EulerToQuat(radians(0), radians(0), radians(random.randint(-70,70))) # roll (x), pitch (y), yaw (z) # ur10 eef_link_gmm

        start_pose.pose.orientation.x = q.x # 1
        start_pose.pose.orientation.y = q.y # 0
        start_pose.pose.orientation.z = q.z # 0
        start_pose.pose.orientation.w = q.w # 0
        self.start_pose_pub.publish(start_pose)

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = self.frame_id
        goal_pose.pose.position.x = random.uniform(0.35, 0.65)
        goal_pose.pose.position.y = random.uniform(-0.0, 0.43) #-0.47, 0.47
        goal_pose.pose.position.z = random.uniform(0.08, 0.1) # + 0.2
        q = self.EulerToQuat(radians(180), radians(0), radians(random.randint(-70,70))) # roll (x), pitch (y), yaw (z) # panda
        # q = self.EulerToQuat(radians(0), radians(0), radians(random.randint(-70,70))) # roll (x), pitch (y), yaw (z) # ur10

        goal_pose.pose.orientation.x = q.x # 1
        goal_pose.pose.orientation.y = q.y # 0
        goal_pose.pose.orientation.z = q.z # 0
        goal_pose.pose.orientation.w = q.w # 0
        self.goal_pose_pub.publish(goal_pose)

        # Setting the position of the obstacle to be between the start and goal poses for more efficient demons collection
        self.setObstacles((start_pose.pose.position.x, goal_pose.pose.position.x),
                          (start_pose.pose.position.y, goal_pose.pose.position.y),
                          (start_pose.pose.position.z, goal_pose.pose.position.z))

        ## Save the variable to write it in the rosbag
        self.start_pose = start_pose
        self.goal_pose = goal_pose
        return start_pose, goal_pose

    def EulerToQuat(self, roll, pitch, yaw): # Very IMPORTANT NOTE: this function's inputs (ordered as roll(x),pitch(y),yaw(z)) are rotations around the reference/origin frame (equivalent to ZYX Euler convention)
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

    def recordTrajectories(self):

        rospy.sleep(0.1)

        nb_of_experiments = 10
        self.demon_num = 1
        start_time = rospy.Time.now().to_sec()
        while self.demon_num <= (self.demon_num-1) + nb_of_experiments:
            print("============ Experiment No. %d ..."%self.demon_num)

            start_pose, goal_pose = self.setStartnGoalPoses()
            print("")
            ans = input("======== Do you Want to Proceed with these Task Parameters (start and goal poses)?! ======== ")
            if ans == 'y' or ans =='Y':

                # self.demon_rosbag = rosbag.Bag(save_dir + "demon_{}.bag".format(self.demon_num), 'w')
                self.record_bags = True
                rospy.sleep(0.1)

                # self.demon_rosbag.write(self.ns + "/obstacles", self.obstacles_posesarray)
                self.obstacles_pub.publish(self.obstacles_posesarray)

                print(" ### Demon recording has started! ###\n ")

                ## This to unsave the Demons that are not good.
                save = input("======== Do you Want to Save this Demon?! ======== ")
                if save == 'n' or save == 'N': self.save_bag = False

                if self.save_bag: self.demon_num += 1

                self.record_bags = False
                self.save_bag = True
                rospy.sleep(0.1)
                # self.demon_rosbag.close()
            else:
                print("===== Skipping this experiment =====")

            # self.record_bags = False


        print("Demonstrations were recorded in Time: ", rospy.Time.now().to_sec() - start_time)

    def testTPGMM(self):
        nb_of_experiments = 10
        exp = 1
        start_time = rospy.Time.now().to_sec()
        while exp <= nb_of_experiments:
            self.move_group.clear_path_constraints()

            print("============ Experiment No. %d ..."%exp)
            ## Going to Ready joints positions
            self.go_to_joint_state()
            rospy.sleep(1.0)

            # tutorial.collect_trajectories(3)

            # start_pose, goal_pose = self.setStartnGoalPoses()
            start_pose, goal_pose = self.setStartnGoalFromRosbag(1)

            print("")
            ans = 'y' # ans = input("======== Do you Want to Proceed with these Task Parameters (start and goal poses)?! ======== ")
            if ans == 'y' or ans =='Y':
                self.go_to_pose(start_pose)
                rospy.sleep(1.0)

                # self.record_bags = True # Flag to get ready to start recording
                self.TPGMM(start_pose, goal_pose)
                # self.go_to_pose(goal_pose)
                # self.record_bags = False # Resetting the Flag to not record

                self.demon_num += 1
                exp += 1
            else:
                print("===== Skipping this experiment =====")

        print("Demonstrations were recorded in Time: ", rospy.Time.now().to_sec() - start_time)

    ## This function converts the bounding volume coming from "/gmm_moveit" topic to move_group constraint
    def set_gmm_constraint(self, bv):
        constraints = Constraints()
        constraints.name = "position_constraint"

        position_constraint = PositionConstraint()

        position_constraint.header.frame_id = self.frame_id   # self.ns[1:] + "_base_link"
        position_constraint.link_name = self.eef_link #self.ns[1:] + "_ee_link"

        position_constraint.constraint_region = bv
        position_constraint.weight = 1.0
        constraints.position_constraints.append(position_constraint)

        return constraints

    def get_gmm_constraint(self, msg):
        self.gmm_bounding_volume = msg

    def TPGMM(self, start_pose, target_pose):
        self.reproduceTPGMM_req(self.frame_id, start_pose, target_pose, self.obstacles_posesarray)

        # print(" ... Waiting for /gmm_moveit ... ")
        # self.move_group.clear_path_constraints()
        # gmm_bounding_volume = rospy.wait_for_message("/gmm_moveit", BoundingVolume,timeout=2.0)
        # self.move_group.set_path_constraints(self.set_gmm_constraint(gmm_bounding_volume))



def main():
    try:
        print("")
        print("----------------------------------------------------------")
        print("Welcome to the MoveIt MoveGroup Python Interface Tutorial")
        print("----------------------------------------------------------")
        print("Press Ctrl-D to exit at any time")
        print("")
        print(
            "============ Press `Enter` to begin the tutorial by setting up the moveit_commander ..."
        )

        namespace = ''
        frame_id = 'follower_link0'
        group_name = 'follower'

        tutorial = MoveGroupPythonInterfaceTutorial(ns=namespace, gn=group_name, fi=frame_id) # '/ur10_1'

        tutorial.recordTrajectories()
        # tutorial.testTPGMM()

        print("============ Python tutorial demo complete!")
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == "__main__":
    main()
    # rospy.spin()
