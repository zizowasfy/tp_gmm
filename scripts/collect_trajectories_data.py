#!/usr/bin/env python3

######
# This script is written to record trajectories (which are used as demonstrations) for tpgmm training, using UR10e in simulation not real. 
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
from tp_gmm.srv import ReproduceTPGMM

from moveit_commander.conversions import pose_to_list
from scipy.spatial.transform import Rotation as R

sys.path.insert(0, '/home/erl/Multicobot-UR10/src')
from motion_planning.scripts import kinematics

# try:
#     from math import pi, tau, dist, fabs, cos
# except:  # For Python 2 compatibility
#     from math import pi, fabs, cos, sqrt

#     tau = 2.0 * pi

#     def dist(p, q):
#         return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


## END_SUB_TUTORIAL

save_dir = "/home/erl/Multicobot-UR10/src/motion_planning/bag_files/Trajectory_Data_Collection/"

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

    def __init__(self, ns):
        super(MoveGroupPythonInterfaceTutorial, self).__init__()

        ## BEGIN_SUB_TUTORIAL setup
        ##
        ## First initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_group_python_interface_tutorial", anonymous=True)

        ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
        ## kinematic model and the robot's current joint states
        robot = moveit_commander.RobotCommander(robot_description= ns + "/robot_description", ns=ns)

        ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
        ## for getting, setting, and updating the robot's internal understanding of the
        ## surrounding world:
        # scene = moveit_commander.PlanningSceneInterface()

        ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
        ## to a planning group (group of joints).  In this tutorial the group is the primary
        ## arm joints in the Panda robot, so we set the group's name to "panda_arm".
        ## If you are using a different robot, change this value to the name of your robot
        ## arm planning group.
        ## This interface can be used to plan and execute motions:
        group_name = "ur10_1_manipulator"
        move_group = moveit_commander.MoveGroupCommander(group_name, robot_description= ns + "/robot_description", ns=ns[1:], wait_for_servers=20)

        ## END_SUB_TUTORIAL

        ## BEGIN_SUB_TUTORIAL basic_info
        ##
        ## Getting Basic Information
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^

        # We can get the name of the reference frame for this robot:
        move_group.set_pose_reference_frame(ns[1:] + "_base_link" )    
        planning_frame = move_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)
        print("============ Pose reference frame: %s" % move_group.get_pose_reference_frame())

        # We can also print the name of the end-effector link for this group:
        move_group.set_end_effector_link("ur10_1_ee_link")
        eef_link = move_group.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        print("============ Available Planning Groups:", robot.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print("============ Printing robot state")
        print(robot.get_current_state())
        print("")
        ## END_SUB_TUTORIAL

        self.start_pose_pub = rospy.Publisher(ns + "/start_pose", PoseStamped, queue_size=1)
        self.goal_pose_pub = rospy.Publisher(ns + "/goal_pose", PoseStamped, queue_size=1)
        self.posearray_pub = rospy.Publisher(ns + "/planned_trajectory/posearray", PoseArray, queue_size=1)
        rospy.Subscriber(ns + "/move_group/result", MoveGroupActionResult, self.move_group_result_callback)
        rospy.Subscriber("/gmm_moveit", BoundingVolume, self.get_gmm_constraint)

        self.reproduceTPGMM_req = rospy.ServiceProxy('ReproduceTPGMM_service', ReproduceTPGMM)


        rospy.sleep(0.5)

        # Misc variables
        self.ns = ns
        self.box_name = ""
        self.robot = robot
        # self.scene = scene
        self.move_group = move_group
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

        self.start_pose = None
        self.goal_pose = None
        self.record_bags = False
        self.demon_num = 1
        self.gmm_bounding_volume = None

    def go_to_joint_state(self):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL plan_to_joint_state
        ##
        ## Planning to a Joint Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^^
        ## The Panda's zero configuration is at a `singularity <https://www.quora.com/Robotics-What-is-meant-by-kinematic-singularity>`_, so the first
        ## thing we want to do is move it to a slightly better configuration.
        ## We use the constant `tau = 2*pi <https://en.wikipedia.org/wiki/Turn_(angle)#Tau_proposals>`_ for convenience:
        # We get the joint values from the group and change some of the values:
        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] =  0.0  # 0
        joint_goal[1] = -1.673186058389443  # -tau / 8
        joint_goal[2] =  1.1933644906726677  # 0
        joint_goal[3] = -1.0713981950229976 # -tau / 4
        joint_goal[4] = -1.570796327 # 0
        joint_goal[5] = -6.257416894683843e-05 # tau / 6  # 1/6 of a turn
        # joint_goal[6] = 0

        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        move_group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        move_group.stop()

        ## END_SUB_TUTORIAL

        # For testing:
        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)

    def go_to_pose(self, target_pose):
        # Copy class variables to local variables to make the web tutorials more clear.
        # NOTE: In practice, you should use the class variables directly unless you have a good
        # reason not to.
        # move_group = self.move_group

        self.move_group.set_pose_target(target_pose)

        ## Now, we call the planner to compute the plan and execute it.
        ## `go()` returns a boolean indicating whether the planning and execution was successful.
        # success = self.move_group.go(wait=wait)
        plan = self.move_group.plan(target_pose)
        self.move_group.execute(plan[1], wait=True)
        # Calling `stop()` ensures that there is no residual movement
        self.move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets().
        self.move_group.clear_pose_targets()

        ## END_SUB_TUTORIAL

        # For testing:
        # Note that since this section of code will not be included in the tutorials
        # we use the class variable rather than the copied state variable
        current_pose = self.move_group.get_current_pose()
        return all_close(target_pose, current_pose, 0.01)

    # def solveFK(self, joint_value_target):

    #     x,y,z,rot = kinematics.get_pose(joint_value_target)
    #     # print(rot.shape)
    #     rot_mat = np.eye(4)
    #     rot_mat[:3, :3] = rot
    #     q = tf.transformations.quaternion_from_matrix(rot_mat)

    #     pose_fk = PoseStamped()
    #     pose_fk.header.frame_id = self.ns[1:] + "_base_link"
    #     pose_fk.pose.position.x = -x
    #     pose_fk.pose.position.y = -y
    #     pose_fk.pose.position.z = z
    #     pose_fk.pose.orientation.x = q[0]
    #     pose_fk.pose.orientation.y = q[1]
    #     pose_fk.pose.orientation.z = q[2]
    #     pose_fk.pose.orientation.w = q[3]

    #     # print(pose_fk)
    #     # self.test_pub.publish(pose_fk)
    #     rot_mat[:3,-1] = [x,y,z]
    #     return pose_fk, rot_mat
    
    def solveFK(self, joint_value_target):

        x,y,z,rot = kinematics.get_pose(joint_value_target)
        # print(rot.shape)
        trans_mat = np.eye(4)
        trans_mat[:3, :3] = rot
        trans_mat[:3,-1] = [x,y,z]

        # NOTE: Since this is only for visualization, the orientations of the trajectory points here are not necessary to be correct, i.e. they do not contribute in the training. 
        # Only the start and goal poses (task parameters) are important
        rot_mat_around_origin = np.array([[-1, 0, 0, 0],
                                          [0, -1, 0, 0],
                                          [0, 0, 1, 0],
                                          [0, 0, 0, 1]]) # 180 around origin z
        rot_mat_around_current = np.array([[0,  1, 0, 0],
                                           [-1,  0, 0, 0],
                                           [0, 0, 1, 0],
                                           [0, 0, 0, 1]]) # -90 around current x
        ## These rotations are manually added, according to what kinematics.get_pose() outputs, to adjust the arrows in the RViz to be intuitive.
        trans_mat = np.dot(rot_mat_around_origin, trans_mat)
        trans_mat = np.dot(trans_mat, rot_mat_around_current)

        q = tf.transformations.quaternion_from_matrix(trans_mat)

        pose_fk = PoseStamped()
        pose_fk.header.frame_id = self.ns[1:] + "_base_link"
        pose_fk.pose.position.x = trans_mat[0,3]
        pose_fk.pose.position.y = trans_mat[1,3]
        pose_fk.pose.position.z = trans_mat[2,3]
        pose_fk.pose.orientation.x = q[0]
        pose_fk.pose.orientation.y = q[1]
        pose_fk.pose.orientation.z = q[2]
        pose_fk.pose.orientation.w = q[3]

        # print(pose_fk)
        # self.test_pub.publish(pose_fk)
        # rot_mat[:3,-1] = [x,y,z]
        return pose_fk, trans_mat

    # def solveFK(self, joint_value_target):
    #     rospy.wait_for_service(self.ns + "/compute_fk")
    #     fk  = rospy.ServiceProxy(self.ns + "/compute_fk", GetPositionFK)
    #     fk_solution = fk()
    
        
    def move_group_result_callback(self, msg):
        if self.record_bags:
            wbag = rosbag.Bag(save_dir + "demon_{}.bag".format(self.demon_num), 'w')
            wbag.write(self.ns + "/start_pose", self.start_pose)
            wbag.write(self.ns + "/goal_pose", self.goal_pose)
            wbag.write(self.ns + "/move_group/result", msg)

            nbpoints = len(msg.result.planned_trajectory.joint_trajectory.points)
            print("Number of Points in Planned Trajectory: ", nbpoints)

            ## Solving FK to get the trajectory in cartesian space
            homog_matrix = Float32MultiArray()
            posearray = PoseArray()
            posearray.header.frame_id = self.ns[1:] + "_base_link"
            for point in msg.result.planned_trajectory.joint_trajectory.points:
                pose_fk, homog_mat = self.solveFK(point.positions)
                homog_matrix.data = homog_mat.flatten(order='F').tolist()
                wbag.write(self.ns + "/planned_trajectory/homog_matrix", homog_matrix)
                posearray.poses.append(deepcopy(pose_fk.pose))

            wbag.write(self.ns + "/planned_trajectory/posearray", posearray)
            self.posearray_pub.publish(posearray)
            wbag.close()

    def setStartnGoalPoses(self):
        ## Setting start & goal poses
        start_pose = PoseStamped()
        start_pose.header.frame_id = self.ns[1:] + "_base_link"
        start_pose.pose.position.x = random.uniform(0.35, 0.55)
        start_pose.pose.position.y = random.uniform(-0.47, 0.47) #(-0.20, 0.20)
        start_pose.pose.position.z = random.uniform(0.5, 0.6)
        q = self.EulerToQuat(radians(0), radians(90), radians(random.randint(-70,70))) # roll (x), pitch (y), yaw (z) # eef_link
        # q = self.EulerToQuat(radians(180), radians(0), radians(random.randint(-70,70))) # roll (x), pitch (y), yaw (z) # tool0
        # q = self.EulerToQuat(radians(0), radians(0), radians(random.randint(-70,70))) # roll (x), pitch (y), yaw (z) # eef_link_gmm

        start_pose.pose.orientation.x = q.x # 1
        start_pose.pose.orientation.y = q.y # 0
        start_pose.pose.orientation.z = q.z # 0
        start_pose.pose.orientation.w = q.w # 0
        self.start_pose_pub.publish(start_pose)

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = self.ns[1:] + "_base_link"
        goal_pose.pose.position.x = random.uniform(0.35, 0.65)
        goal_pose.pose.position.y = random.uniform(-0.47, 0.47) #(-0.20, 0.20)
        goal_pose.pose.position.z = random.uniform(0.08, 0.1)+0.1  # + 0.2
        q = self.EulerToQuat(radians(0), radians(90), radians(random.randint(-70,70))) # roll (x), pitch (y), yaw (z) # eef_link
        # q = self.EulerToQuat(radians(180), radians(0), radians(random.randint(-70,70))) # roll (x), pitch (y), yaw (z) # tool0

        goal_pose.pose.orientation.x = q.x # 1
        goal_pose.pose.orientation.y = q.y # 0
        goal_pose.pose.orientation.z = q.z # 0
        goal_pose.pose.orientation.w = q.w # 0
        self.goal_pose_pub.publish(goal_pose)
        
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
        nb_of_experiments = 10
        self.demon_num = 51
        start_time = rospy.Time.now().to_sec()
        while self.demon_num <= (self.demon_num-1) + nb_of_experiments:
            print("============ Experiment No. %d ..."%self.demon_num)
            ## Going to Ready joints positions
            self.go_to_joint_state()
            rospy.sleep(1.0)

            # tutorial.collect_trajectories(3)

            start_pose, goal_pose = self.setStartnGoalPoses()
            print("")
            ans = 'y' # ans = input("======== Do you Want to Proceed with these Task Parameters (start and goal poses)?! ======== ")
            if ans == 'y' or ans =='Y':
                self.go_to_pose(start_pose)
                rospy.sleep(1.0)

                self.record_bags = True # Flag to get ready to start recording
                self.go_to_pose(goal_pose)
                self.record_bags = False # Resetting the Flag to not record

                self.demon_num += 1
            else: 
                print("===== Skipping this experiment =====")

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

            start_pose, goal_pose = self.setStartnGoalPoses()

            print("")
            ans = 'y' # ans = input("======== Do you Want to Proceed with these Task Parameters (start and goal poses)?! ======== ")
            if ans == 'y' or ans =='Y':
                self.go_to_pose(start_pose)
                rospy.sleep(1.0)

                self.record_bags = True # Flag to get ready to start recording
                self.TPGMM(start_pose, goal_pose)
                self.go_to_pose(goal_pose)
                self.record_bags = False # Resetting the Flag to not record

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
        
        position_constraint.header.frame_id = self.ns[1:] + "_base_link"   # self.ns[1:] + "_base_link"  
        position_constraint.link_name = self.ns[1:] + "_ee_link" 

        position_constraint.constraint_region = bv
        position_constraint.weight = 1.0
        constraints.position_constraints.append(position_constraint)        

        return constraints

    def get_gmm_constraint(self, msg):
        self.gmm_bounding_volume = msg

    def TPGMM(self, start_pose, target_pose):
        self.reproduceTPGMM_req(self.ns[1:] + '_base_link', start_pose, target_pose)
        
        print(" ... Waiting for /gmm_moveit ... ")
        self.move_group.clear_path_constraints()
        gmm_bounding_volume = rospy.wait_for_message("/gmm_moveit", BoundingVolume,timeout=2.0)
        self.move_group.set_path_constraints(self.set_gmm_constraint(gmm_bounding_volume))

    

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
        tutorial = MoveGroupPythonInterfaceTutorial('/ur10_1')

        # tutorial.recordTrajectories()
        tutorial.testTPGMM()

        print("============ Python tutorial demo complete!")
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == "__main__":
    main()
    # rospy.spin()
