#! /usr/bin/env python

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import rospkg
import cv2

import yaml
import os
import math
import time
import sys
import copy

import tf2_ros
import tf2_msgs.msg

from std_srvs.srv import Empty
from pkg_vb_sim.srv import vacuumGripper
from pkg_vb_sim.srv import conveyorBeltPowerMsg
from pkg_vb_sim.msg import LogicalCameraImage
from pkg_task_4.msg import packageMsg

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from pyzbar.pyzbar import decode

class Ur5Moveit:
    """
    Constructor
    """
    def __init__(self, arg_robot_name):

        rospy.init_node('node_ur5_1', anonymous=True)

        self._robot_ns = '/'  + arg_robot_name
        self._planning_group = "manipulator"

        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander(robot_description= self._robot_ns + "/robot_description", ns=self._robot_ns)
        self._scene = moveit_commander.PlanningSceneInterface(ns=self._robot_ns)
        self._group = moveit_commander.MoveGroupCommander(self._planning_group, robot_description= self._robot_ns + "/robot_description", ns=self._robot_ns)
        self._display_trajectory_publisher = rospy.Publisher( self._robot_ns + '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)
        self._exectute_trajectory_client = actionlib.SimpleActionClient( self._robot_ns + '/execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()

        #Setup for tf
        self._tf_buffer = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._tf_buffer)

        # Attribute to store computed trajectory by the planner
        self._computed_plan = ''
        self.presence_of_package = False
        # Current State of the Robot is needed to add box to planning scene
        self._curr_state = self._robot.get_current_state()

        rospy.loginfo(
            '\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')


        rp = rospkg.RosPack()
        self._pkg_path = rp.get_path('pkg_moveit_examples')
        self._file_path = self._pkg_path + '/config/'
        rospy.loginfo( "Package Path: {}".format(self._file_path) )

        self.i = -1
        self.j = -1

        self._pkg_detect_flag = False
        self._pkg_pickup_flag = False

        self.color_w = []

        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

    def clear_octomap(self):
        clear_octomap_service_proxy = rospy.ServiceProxy(self._robot_ns + "/clear_octomap", Empty)
        return clear_octomap_service_proxy()

    def go_to_pose(self, arg_pose):
        """
        Plan and Execute : Go to Pose
        """
        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Current Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        self._group.set_pose_target(arg_pose)
        #rospy.loginfo(self._group.plan())
        flag_plan = self._group.go(wait=True)

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        if flag_plan:
            rospy.loginfo(
                '\033[94m' + ">>> go_to_pose() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> go_to_pose() Failed. Solution for Pose not Found." + '\033[0m')

        return flag_plan

    def set_joint_angles(self, arg_list_joint_angles):
        """
        Plan and Execute : Set Joint Angles
        """
        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Current Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        self._group.set_joint_value_target(arg_list_joint_angles)
        self._group.plan()
        flag_plan = self._group.go(wait=True)

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        if flag_plan:
            rospy.loginfo(
                '\033[94m' + ">>> set_joint_angles() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> set_joint_angles() Failed." + '\033[0m')

        return flag_plan

    def go_to_predefined_pose(self, arg_pose_name):
        """
        Plan and Execute : Pre-defined Pose
        """
        rospy.loginfo('\033[94m' + "Going to Pose: {}".format(arg_pose_name) + '\033[0m')
        self._group.set_named_target(arg_pose_name)
        plan = self._group.plan()
        goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
        goal.trajectory = plan
        self._exectute_trajectory_client.send_goal(goal)
        self._exectute_trajectory_client.wait_for_result()
        rospy.loginfo('\033[94m' + "Now at Pose: {}".format(arg_pose_name) + '\033[0m')


    def func_callback_logical_camera(self, msg):
        """
        Callback Function for Logical Camera Subscription
        """
        if msg.models:

            self._pkg_detect_flag = True
            self.i = msg.models[0].type[-2]
            self.j = msg.models[0].type[-1]
        else:
            self._pkg_detect_flag = False


    def activate_vacuum_gripper(self, state):
        """
        Enable/Disable Gripper Module
        """
        rospy.wait_for_service('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_2')
        try:
            activate_vacuum_gripper = rospy.ServiceProxy('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_2',
                                                         vacuumGripper)
            res = activate_vacuum_gripper(state)
            return res
        except rospy.ServiceException as err:
            print "Service call failed: %s" + err

    def set_conveyor_belt_speed(self, speed):
        """
        Control Conveyor Belt Speed
        """
        rospy.wait_for_service('/eyrc/vb/conveyor/set_power')
        try:
            set_conveyor_belt_speed = rospy.ServiceProxy('/eyrc/vb/conveyor/set_power',
                                                         conveyorBeltPowerMsg)
            res = set_conveyor_belt_speed(speed)
            return res
        except rospy.ServiceException as err:
            print "Service call failed: %s" + err

    def moveit_play_planned_path_from_file(self, arg_file_path, arg_file_name):
        """
        Loading the trajectories from the file and executing
        """
        file_path = arg_file_path + arg_file_name

        with open(file_path, 'r') as file_open:
          plan = yaml.load(file_open)

        ret = self._group.execute(plan)

        return ret

    def moveit_hard_play_planned_path_from_file(self, arg_file_path, arg_file_name, arg_max_attempts):
        number_attempts = 0
        flag_success = False

        while ( (number_attempts <= arg_max_attempts) and (flag_success is False) ):
            number_attempts += 1
            flag_success = self.moveit_play_planned_path_from_file(arg_file_path, arg_file_name)
            rospy.logwarn("attempts: {}".format(number_attempts) )

        return True

    def __del__(self):
        """
        Destructor
        """
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')

    def func_get_color(self, msg):

        self.color_w.append(msg.color)

def main():
    """
    Main Function
    """
    rp = rospkg.RosPack()

    arg_package_path = rp.get_path('pkg_task4')
    arg_file_path = arg_package_path + '/config/ur5_2_saved_trajectories/'

    ur5 = Ur5Moveit('ur5_2')

    rospy.Subscriber('/eyrc/vb/logical_camera_2', LogicalCameraImage, ur5.func_callback_logical_camera)
    rospy.Subscriber('/packageMsg_topic', packageMsg, ur5.func_get_color)

    box_length = 0.15               # Length of the Package
    vacuum_gripper_width = 0.115    # Vacuum Gripper Width
    delta_z = vacuum_gripper_width + (box_length/2)

    ur5_2_home_pose = geometry_msgs.msg.Pose()
    ur5_2_home_pose.position.x = -0.8
    ur5_2_home_pose.position.y = 0
    ur5_2_home_pose.position.z = 1 + delta_z
    ur5_2_home_pose.orientation.x = -0.5
    ur5_2_home_pose.orientation.y = -0.5
    ur5_2_home_pose.orientation.z = 0.5
    ur5_2_home_pose.orientation.w = 0.5

    ur5_2_bin_red = copy.deepcopy(ur5_2_home_pose)
    ur5_2_bin_yellow = copy.deepcopy(ur5_2_home_pose)
    ur5_2_bin_green = copy.deepcopy(ur5_2_home_pose)

    ur5_2_bin_red.position.x = 0.11
    ur5_2_bin_red.position.y = 0.65

    ur5_2_bin_yellow.position.x = 0.75
    ur5_2_bin_yellow.position.y = 0.03

    ur5_2_bin_green.position.x = 0.04
    ur5_2_bin_green.position.y = -0.65

    count = 0
    color = "zero"

    while count<9:
        rospy.sleep(0.01)
        rospy.loginfo(count)

        if ur5._pkg_detect_flag:

            rospy.sleep(0.6)                # Delay to reach centre
            ur5.set_conveyor_belt_speed(0)  # Stop belt

            pre_color = color
            color = ur5.color_w[0]

            arg_file_name = pre_color+"_to_drop.yaml"
            ur5.moveit_hard_play_planned_path_from_file(arg_file_path, arg_file_name, 100)

            ur5._pkg_pickup_flag = True  # Mark as "ready for pick up"
            ur5._pkg_detect_flag = False # Mark as "already detected"

            ur5.activate_vacuum_gripper(True)   # Activate gripper
            ur5.set_conveyor_belt_speed(100)
            if color == "red":

                arg_file_name = "drop_to_"+color+".yaml"
                ur5.moveit_hard_play_planned_path_from_file(arg_file_path, arg_file_name,  100)      # Go to bin
                ur5.activate_vacuum_gripper(False)  # Deactivate gripper


            elif color == "yellow":

                arg_file_name = "drop_to_"+color+".yaml"
                ur5.moveit_hard_play_planned_path_from_file(arg_file_path, arg_file_name, 100)      # Go to bin
                ur5.activate_vacuum_gripper(False)  # Deactivate gripper

            elif color == "green":

                arg_file_name = "drop_to_"+color+".yaml"
                ur5.moveit_hard_play_planned_path_from_file(arg_file_path, arg_file_name, 100)      # Go to bin
                ur5.activate_vacuum_gripper(False)  # Deactivate gripper

            ur5.color_w.pop(0)
            count = count + 1
            ur5.set_conveyor_belt_speed(100)    # Resume conveyor


    del ur5


if __name__ == '__main__':
    main()
