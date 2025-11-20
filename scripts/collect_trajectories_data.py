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
from moveit_msgs.msg import MoveGroupActionResult, BoundingVolume, Constraints, PositionConstraint, DisplayTrajectory
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

    def __init__(self, ns, frame_id, group_name):
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
        scene = moveit_commander.PlanningSceneInterface(ns=ns)

        ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
        ## to a planning group (group of joints).  In this tutorial the group is the primary
        ## arm joints in the Panda robot, so we set the group's name to "panda_arm".
        ## If you are using a different robot, change this value to the name of your robot
        ## arm planning group.
        ## This interface can be used to plan and execute motions:
        # group_name = "ur10_1_manipulator"
        move_group = moveit_commander.MoveGroupCommander(group_name, robot_description= ns + "/robot_description", ns=ns[1:], wait_for_servers=20)
        move_group.set_max_velocity_scaling_factor(1.0)
        move_group.set_max_acceleration_scaling_factor(1.0)
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
        # print(robot.get_current_state())
        print("")
        ## END_SUB_TUTORIAL

        self.start_pose_pub = rospy.Publisher(ns + "/start_pose", PoseStamped, queue_size=1)
        self.goal_pose_pub = rospy.Publisher(ns + "/goal_pose", PoseStamped, queue_size=1)
        self.posearray_pub = rospy.Publisher(ns + "/planned_trajectory/posearray", PoseArray, queue_size=1)
        self.obstacles_pub = rospy.Publisher(ns + "/obstacles", PoseArray, queue_size=1)
        
        # rospy.Subscriber(ns + "/move_group/result", MoveGroupActionResult, self.move_group_result_callback)
        rospy.Subscriber(f"{ns}/gmm_moveit", BoundingVolume, self.get_gmm_constraint)
        rospy.Subscriber(ns + "/move_group/display_planned_path", DisplayTrajectory, self.displayPlannedPath)

        self.reproduceTPGMM_req = rospy.ServiceProxy(f'{ns}/ReproduceTPGMM_service', ReproduceTPGMM)

        rospy.sleep(0.5)

        # Misc variables
        self.ns = ns
        self.group_name = group_name
        self.frame_id = frame_id
        self.box_name = ""
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

        self.start_pose = None
        self.goal_pose = None
        self.plan = None
        self.record_bags = False
        self.save_bag = True
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
        # print(plan)
        self.plan = plan[1]
        self.move_group.execute(plan[1], wait=True)

        # Calling `stop()` ensures that there is no residual movement
        self.move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets().
        self.move_group.clear_pose_targets()

        ## END_SUB_TUTORIAL

        # For testing:
        ## Note that since this section of code will not be included in the tutorials
        ## we use the class variable rather than the copied state variable
        # current_pose = self.move_group.get_current_pose()
        # return all_close(target_pose, current_pose, 0.01)
        
        try:
            current_joints = self.move_group.get_current_joint_values()
            return all_close(self.plan.joint_trajectory.points[-1], current_joints, 0.01)
        except:
            return False

    def plan_to_pose(self, target_pose):
        # Copy class variables to local variables to make the web tutorials more clear.
        # NOTE: In practice, you should use the class variables directly unless you have a good
        # reason not to.
        # move_group = self.move_group

        self.move_group.set_pose_target(target_pose)

        ## Now, we call the planner to compute the plan and execute it.
        ## `go()` returns a boolean indicating whether the planning and execution was successful.
        # success = self.move_group.go(wait=wait)
        plan = self.move_group.plan(target_pose)
        # print(plan)
        self.plan = plan[1]

        return plan[0] # True or False
            
    def execute_plan(self):

        self.move_group.execute(self.plan, wait=True)

        # Calling `stop()` ensures that there is no residual movement
        self.move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets().
        self.move_group.clear_pose_targets()

        ## END_SUB_TUTORIAL

        # For testing:
        ## Note that since this section of code will not be included in the tutorials
        ## we use the class variable rather than the copied state variable
        # current_pose = self.move_group.get_current_pose()
        # return all_close(target_pose, current_pose, 0.01)
        
        try:
            current_joints = self.move_group.get_current_joint_values()
            return all_close(self.plan.joint_trajectory.points[-1], current_joints, 0.01)
        except:
            return False
                
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
        pose_fk.header.frame_id = self.frame_id
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
    
    def displayPlannedPath(self, msg):
        # print(msg)
        homog_matrix = Float32MultiArray()
        posearray = PoseArray()
        posearray.header.frame_id = self.frame_id        
        for point in msg.trajectory[0].joint_trajectory.points:
            pose_fk, homog_mat = self.solveFK(point.positions)
            homog_matrix.data = homog_mat.flatten(order='F').tolist()
            posearray.poses.append(deepcopy(pose_fk.pose))

        self.posearray_pub.publish(posearray)
    
    # def move_group_result_callback(self, msg):
    def recordRosbag(self):
        wbag = rosbag.Bag(save_dir + "demon_{}.bag".format(self.demon_num), 'w')
        wbag.write(self.ns + "/start_pose", self.start_pose)
        wbag.write(self.ns + "/goal_pose", self.goal_pose)
        wbag.write(self.ns + "/obstacles", self.obstacles_posesarray)
        wbag.write(self.ns + "/move_group/result", self.plan)

        nbpoints = len(self.plan.joint_trajectory.points)
        print("Number of Points in Planned Trajectory: ", nbpoints)

        ## Solving FK to get the trajectory in cartesian space
        homog_matrix = Float32MultiArray()
        posearray = PoseArray()
        posearray.header.frame_id = self.frame_id
        for point in self.plan.joint_trajectory.points:
            pose_fk, homog_mat = self.solveFK(point.positions)
            homog_matrix.data = homog_mat.flatten(order='F').tolist()
            wbag.write(self.ns + "/planned_trajectory/homog_matrix", homog_matrix)
            posearray.poses.append(deepcopy(pose_fk.pose))

        wbag.write(self.ns + "/planned_trajectory/posearray", posearray)
        self.posearray_pub.publish(posearray)
        wbag.close()
    
    def setObstacles(self, X, Y, Z):
        ## reset/remove all obstacles in the planning scene
        # self.scene.remove_world_object()
        self.scene.remove_world_object(name="obstacle_0")

        # nb_obstacles = random.randint(1,5)
        nb_obstacles = 1
        self.obstacles_posesarray = PoseArray()
        self.obstacles_posesarray.header.frame_id = self.frame_id
        for obs in range(nb_obstacles):
            obstacle_pose = PoseStamped()
            obstacle_pose.header.frame_id = self.frame_id
            obstacle_pose.pose.position.x = random.uniform(X[0], X[1]) + 0.1
            obstacle_pose.pose.position.y = random.uniform(Y[0], Y[1]) 
            obstacle_pose.pose.position.z = random.uniform(Z[0], Z[1]) + 0.1
            # q = self.EulerToQuat(radians(0), radians(0), radians(random.randint(-70,70))) # roll (x), pitch (y), yaw (z)      (batch4)
            q = self.EulerToQuat(radians(180), radians(0), radians(random.randint(-30,30))) # roll (x), pitch (y), yaw (z)      (batch5)

            obstacle_pose.pose.orientation.x = q.x # 1
            obstacle_pose.pose.orientation.y = q.y # 0
            obstacle_pose.pose.orientation.z = q.z # 0
            obstacle_pose.pose.orientation.w = q.w # 0
            self.obstacles_posesarray.poses.append(deepcopy(obstacle_pose.pose))
        
            self.scene.add_box("obstacle_{}".format(obs), obstacle_pose, size=(0.1, 0.05, 0.5)) #(0.05, 0.05, 0.05) #(0.1, 0.05, 0.15)

        self.obstacles_pub.publish(self.obstacles_posesarray)


    ## This function returns the range of random values corresponding to the task being trained on
    def getTaskRandomPoses(self, task_name):
        start = Pose()
        goal = Pose()

        if task_name == "pick":
            start.position.x = random.uniform(0.35, 0.55)
            start.position.y = random.uniform(-0.47, 0.47) #(-0.20, 0.20)
            start.position.z = random.uniform(0.5, 0.6)
            start.orientation = self.EulerToQuat(radians(0), radians(90), radians(random.randint(-70,70))) # roll (x), pitch (y), yaw (z) # eef_link

            goal.position.x = random.uniform(0.35, 0.65)
            goal.position.y = random.uniform(-0.47, 0.47) #(-0.20, 0.20)
            goal.position.z = random.uniform(0.05, 0.1)
            goal.orientation = self.EulerToQuat(radians(0), radians(90), radians(random.randint(-70,70))) # roll (x), pitch (y), yaw (z) # eef_link

        elif task_name == "place_ur10_1":
            ## [0.916, 0.341, 0.755], [0.573, 0.371, 0.750], [0.777, -0.076, 0.890] positions of Cable, Busbar, ServicePlug 
            # start.position.x = random.uniform(0.5, 0.9)
            # start.position.y = random.uniform(-0.1, 0.5)
            # start.position.z = random.uniform(0.5, 0.773)
            # start.orientation = self.EulerToQuat(radians(0), radians(90), radians(random.randint(-30,30))) # roll (x), pitch (y), yaw (z) # eef_link
            objects = [[0.916, 0.341, 0.755], [0.573, 0.371, 0.750], [0.777, -0.076, 0.890]]
            start.position.x = objects[self.demon_num-1][0]
            start.position.y = objects[self.demon_num-1][1]
            start.position.z = objects[self.demon_num-1][2]
            # start.position.x = objects[2][0]
            # start.position.y = objects[2][1]
            # start.position.z = objects[2][2]            
            start.orientation = self.EulerToQuat(radians(0), radians(90), radians(0)) # roll (x), pitch (y), yaw (z) # eef_link

            ## [0.554, -0.167, 0.836]
            goal.position.x = 0.554  # random.uniform(0.55, 0.6)
            goal.position.y = -0.167 # random.uniform(-0.3, -0.2)
            goal.position.z = 0.836  # random.uniform(0.8, 0.9)
            goal.orientation = self.EulerToQuat(radians(0), radians(90), radians(-45)) # roll (x), pitch (y), yaw (z) # eef_link            

        elif task_name == "place_ur10_2":
            ## [0.830, -0.411, 0.583], [0.505, -0.434, 0.552], [0.486, -0.086, 0.597] positions of the 3 LeafCells
            # start.position.x = random.uniform(0.25, 0.95) #(0.25, 0.85)
            # start.position.y = random.uniform(-0.31, -0.0) #(-0.51, -0.16)
            # start.position.z = random.uniform(0.55, 0.7)
            # start.orientation = self.EulerToQuat(radians(0), radians(90), radians(0)) # roll (x), pitch (y), yaw (z) # eef_link
            objects = [[0.486, -0.086, 0.597], [0.505, -0.434, 0.552], [0.830, -0.411, 0.583]]
            start.position.x = objects[self.demon_num-1][0]
            start.position.y = objects[self.demon_num-1][1]
            start.position.z = objects[self.demon_num-1][2]
            # start.position.x = objects[2][0] 
            # start.position.y = objects[2][1]
            # start.position.z = objects[2][2]           
            start.orientation = self.EulerToQuat(radians(0), radians(90), radians(0)) # roll (x), pitch (y), yaw (z) # eef_link

            ## [0.558, -0.947, 0.416]
            goal.position.x = 0.558  # random.uniform(0.5, 0.6)
            goal.position.y = -0.947 # random.uniform(-1.0, -0.9)
            goal.position.z = 0.416  # random.uniform(0.3, 0.4)
            goal.orientation = self.EulerToQuat(radians(0), radians(90), radians(-90)) # roll (x), pitch (y), yaw (z) # eef_link

        return start, goal
    
    def setStartnGoalPoses(self, task):
        task_start, task_goal = self.getTaskRandomPoses(task)

        ## Setting start & goal poses
        start_pose = PoseStamped()
        start_pose.header.frame_id = self.frame_id
        start_pose.pose = task_start
        self.start_pose_pub.publish(start_pose)

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = self.frame_id
        goal_pose.pose = task_goal
        self.goal_pose_pub.publish(goal_pose)
        
        ## Save the variable to write it in the rosbag
        self.start_pose = start_pose            
        self.goal_pose = goal_pose
        
        ## Setting the position of the obstacle to be between the start and goal poses for more efficient demons collection
        self.setObstacles((start_pose.pose.position.x, goal_pose.pose.position.x),
                          (start_pose.pose.position.y, goal_pose.pose.position.y),
                          (start_pose.pose.position.z, goal_pose.pose.position.z))
        
        return start_pose, goal_pose

    def EulerToQuat(self, roll, pitch, yaw): # Very IMPORTANT NOTE: this function's inputs (ordered as roll(x),pitch(y),yaw(z)) are rotations around the reference/origin frame (equivalent to ZYX Euler convention)
        # print("Z = ", degrees(yaw))
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

    def recordTrajectories(self, task):
        nb_of_experiments = 3 #10
        demon_num_start = 1
        self.demon_num = demon_num_start

        start_time = rospy.Time.now().to_sec()
        while self.demon_num < (demon_num_start) + nb_of_experiments:
            print("============ Experiment No. %d ..."%self.demon_num)
            ## Going to Ready joints positions
            rospy.sleep(0.5)
            self.go_to_joint_state()
            

            start_pose, goal_pose = self.setStartnGoalPoses(task)
            print("")
            ans = 'y'
            # ans = input("======== Do you Want to Proceed with these Task Parameters (start and goal poses)?! ======== ")  
            if ans == 'y' or ans =='Y':
                moved = self.go_to_pose(start_pose)
                rospy.sleep(0.5)

                if moved:
                    # moved = self.go_to_pose(goal_pose)
                    planned = self.plan_to_pose(goal_pose)
                    # if moved:
                    if planned:
                        ans = input("   ======== Do you Want to Excute this Plan?! ======== ")
                        if ans:
                            executed = self.execute_plan()
                            if executed:
                                ## This to unsave the Demons that are not good.
                                # save = input("        ======== Do you Want to Save this Demon?! ======== ")
                                save = 'y'
                                if save == 'y' or save == 'Y': 
                                    self.recordRosbag()
                                    self.demon_num += 1
                    else:
                        print("===== No Plan Found for 'goal_pose' =====")
                rospy.sleep(0.1)
            else: 
                print("===== Skipping this experiment =====")

        print("Demonstrations were recorded in Time: ", rospy.Time.now().to_sec() - start_time)

    def testTPGMM(self, task):
        nb_of_experiments = 10
        exp = 1
        start_time = rospy.Time.now().to_sec()
        while exp <= nb_of_experiments:
            self.move_group.clear_path_constraints()

            print("============ Experiment No. %d ..."%exp)
            ## Going to Ready joints positions
            self.go_to_joint_state()
            rospy.sleep(1.0)

            start_pose, goal_pose = self.setStartnGoalPoses(task)

            print("")
            ans = 'y' # ans = input("======== Do you Want to Proceed with these Task Parameters (start and goal poses)?! ======== ")
            if ans == 'y' or ans =='Y':
                self.go_to_pose(start_pose)
                rospy.sleep(1.0)

                self.record_bags = True # Flag to get ready to start recording
                self.TPGMM(task, start_pose, goal_pose)
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
        
        position_constraint.header.frame_id = self.frame_id   # self.ns[1:] + "_base_link"  
        position_constraint.link_name = self.ns[1:] + "_ee_link" 

        position_constraint.constraint_region = bv
        position_constraint.weight = 1.0
        constraints.position_constraints.append(position_constraint)        

        return constraints

    def get_gmm_constraint(self, msg):
        self.gmm_bounding_volume = msg

    def TPGMM(self, task, start_pose, target_pose):
        self.reproduceTPGMM_req(task, self.ns[1:] + '_base_link', start_pose, target_pose)
        
        print(" ... Waiting for /gmm_moveit ... ")
        self.move_group.clear_path_constraints()
        # gmm_bounding_volume = rospy.wait_for_message(f"{self.ns}/gmm_moveit", BoundingVolume,timeout=2.0)
        self.move_group.set_path_constraints(self.set_gmm_constraint(self.gmm_bounding_volume))

    

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

        namespace = "/ur10_1"
        frame_id = namespace[1:] + "_base_link"
        group_name = "ur10_1_manipulator"

        tutorial = MoveGroupPythonInterfaceTutorial(ns=namespace, frame_id=frame_id, group_name=group_name)

        task_name = 'place_ur10_2'

        # tutorial.recordTrajectories(task=task_name)
        tutorial.testTPGMM(task=task_name)

        print("============ Python tutorial demo complete!")
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == "__main__":
    main()
    # rospy.spin()
