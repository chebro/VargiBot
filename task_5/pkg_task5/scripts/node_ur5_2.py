#! /usr/bin/env python

"""
This program intends to control UR5_2 arm.
UR5_2 arm function: To sort the packages according to their
                    respective colours.
"""

import time
import sys
import datetime

import yaml
import rospkg
import rospy
import moveit_commander
import moveit_msgs.msg
import actionlib

from pkg_vb_sim.srv import vacuumGripper
from pkg_vb_sim.srv import conveyorBeltPowerMsg
from pkg_vb_sim.msg import LogicalCameraImage
from pkg_task5.msg import packageMsg
from pkg_task5.msg import shippedMsg

class Ur5Moveit(object):
    """
    This class defines the robot and planning function of the ur5 arm.
    """
    def __init__(self, arg_robot_name):
        """
        Constructor
        This intiates the robot and required arguments in this class.
        """
        rospy.init_node('node_ur5_2', anonymous=True)

        self._robot_ns = '/'  + arg_robot_name
        self._planning_group = "manipulator"

        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander(
            robot_description=self._robot_ns + "/robot_description", ns=self._robot_ns
            )
        self._scene = moveit_commander.PlanningSceneInterface(ns=self._robot_ns)
        self._group = moveit_commander.MoveGroupCommander(
            self._planning_group, robot_description=self._robot_ns + "/robot_description",
            ns=self._robot_ns
            )
        self._display_trajectory_publisher = rospy.Publisher(
            self._robot_ns + '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory,
            queue_size=1
            )
        self._exectute_trajectory_client = actionlib.SimpleActionClient(
            self._robot_ns + '/execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction
            )
        self._exectute_trajectory_client.wait_for_server()

        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()

        self.presence_of_package = False
        # Current State of the Robot is needed to add box to planning scene
        self._curr_state = self._robot.get_current_state()

        rospy.loginfo(
            '\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')


        relative_path = rospkg.RosPack()
        self._pkg_path = relative_path.get_path('pkg_moveit_examples')
        self._file_path = self._pkg_path + '/config/'
        rospy.loginfo("Package Path: {}".format(self._file_path))

        self.pkg_detect_flag = False

        self.color_w = []
        self.city_w = []
        self.orderid_w = []

        self.ship_pub = rospy.Publisher('topic_shipped_data', shippedMsg, queue_size=10)

        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

    def go_to_pose(self, arg_pose):
        """
        Plan and Execute : Go to Pose
        """
        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Current Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        self._group.set_pose_target(arg_pose)

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

        Parameter(string): Predefined pose name
        """
        rospy.loginfo('\033[94m' + "Going to Pose: {}".format(arg_pose_name) + '\033[0m')
        self._group.set_named_target(arg_pose_name)
        plan = self._group.plan()
        goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
        goal.trajectory = plan
        self._exectute_trajectory_client.send_goal(goal)
        self._exectute_trajectory_client.wait_for_result()
        rospy.loginfo('\033[94m' + "Now at Pose: {}".format(arg_pose_name) + '\033[0m')

    def play_planned_path_from_file(self, arg_file_path, arg_file_name):
        """
        Loading the trajectories from the file and executing.
        This method executes the the plan stored in the config folder.

        Parameters:
            arg_file_path(string): Path of the file containing the trajectory.
            arg_file_name(string): Name of the file containing the trajectory.
        Returns:
            bool: True-> Path executes || False-> Failed to execute the path
        """
        file_path = arg_file_path + arg_file_name

        with open(file_path, 'r') as file_open:
            plan = yaml.load(file_open)

        flag = self._group.execute(plan)

        return flag

    def hard_play_planned_path(self, arg_file_path,
                               arg_file_name, arg_max_attempts):
        """
        Hard playing the saved trajectories from file.
        This method hard plays the trajectories till the path is played or
        maximum attemps have been made.

        Parameters:
            arg_file_path(string): Path of the file containing the trajectory.
            arg_file_name(string): Name of the file containing the trajectory.
            arg_max_attempts(int): Maximum number of attempts to play the file.

        Returns:
            bool: True-> Path executes || False-> Failed to execute the path
        """
        number_attempts = 0
        flag_success = False

        while((number_attempts <= arg_max_attempts) and (flag_success is False)):
            number_attempts += 1
            flag_success = self.play_planned_path_from_file(arg_file_path, arg_file_name)
            rospy.logwarn("attempts: {}".format(number_attempts))

        return flag_success

    def func_callback_logical_camera(self, msg):
        """
        Callback Function for Logical Camera Subscription
        """
        self.pkg_detect_flag = msg.models and msg.models[-1].type != 'ur5'

    def func_callback_package_details(self, msg):
        """
        Callback function
        This method assigns the string which is published on the topic topic_package_details.

        Parameters:
            msg(string): Msg obtained from the topic topic_shipped_data containing various
                        attributes.
                        packageName(string): Name of the package sent.
                        color(string): Color of the package.
                        city(string): Destination city of the package.
                        orderid(string): Order ID assigned to the package.
        """

        self.color_w.append(msg.color)
        self.city_w.append(msg.city)
        self.orderid_w.append(msg.orderid)

    def func_publish_orders_shipped(self, city, color, orderid):
        """
        Publish function
        This method returns the string which is published on the topic topic_shipped_data.

        Parameters:
            city(string): Destination city of the package.
            color(string): Color of the package.
            orderid(string): Order ID assigned to the package.

        """
        ship_obj = {}
        ship_obj['Team Id'] = 'VB#1004'
        ship_obj['Unique Id'] = 'CeAhsAGA'
        ship_obj['Order ID'] = orderid
        ship_obj['id'] = 'OrdersShipped'
        ship_obj['Shipped Quantity'] = '1'
        ship_obj['Shipped Date and Time'] = get_time_str()

        ship_obj['City'] = city
        ship_obj['Shipped Status'] = 'YES'

        if color == 'red':
            ship_obj['Item'] = 'Medicine'
            ship_obj['Priority'] = 'HP'
            ship_obj['Cost'] = 300
            ship_obj['Estimated Time of Delivery'] = get_est_time_str(1)

        if color == 'yellow':
            ship_obj['Item'] = 'Food'
            ship_obj['Priority'] = 'MP'
            ship_obj['Cost'] = 200
            ship_obj['Estimated Time of Delivery'] = get_est_time_str(2)

        if color == 'green':
            ship_obj['Item'] = 'Clothes'
            ship_obj['Priority'] = 'LP'
            ship_obj['Cost'] = 100
            ship_obj['Estimated Time of Delivery'] = get_est_time_str(3)

        str_ship_obj = str(ship_obj)
        rospy.sleep(2)
        self.ship_pub.publish(str_ship_obj)

    def __del__(self):
        """
        Destructor
        """
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')

def activate_vacuum_gripper(state):
    """
    Enable/Disable Gripper Module
    """
    rospy.wait_for_service('/eyrc/vb/activate_vacuum_gripper/ur5_2')
    try:
        activate_vacuum_gripper_service = rospy.ServiceProxy(
            '/eyrc/vb/activate_vacuum_gripper/ur5_2', vacuumGripper
            )
        res = activate_vacuum_gripper_service(state)
        return res
    except rospy.ServiceException as err:
        print "Service call failed: %s" + err

def set_conveyor_belt_speed(speed):
    """
    Control Conveyor Belt Speed

    This function allows us to control the speed of the conveyor belt
    using the service /eyrc/vb/conveyor/set_power.

    Parameter:
        speed(int): This value ranges from 0-100. Input value is
                    directly proportional to power set.
    Returns:
        The service call's output is reflected in the world's conveyor belt.
    """
    rospy.wait_for_service('/eyrc/vb/conveyor/set_power')
    try:
        set_conveyor_belt_speed_value = rospy.ServiceProxy('/eyrc/vb/conveyor/set_power',
                                                           conveyorBeltPowerMsg)
        res = set_conveyor_belt_speed_value(speed)
        return res
    except rospy.ServiceException as err:
        print "Service call failed: %s" + err



def get_time_str():
    """
    Date string.
    This function is used to get the date after adding the offset date and
    extract current time in yyyymmdd format.

    Parameter:
    offset(int): Days which are to be added to the date.
    Returns:
    The date after offset number of days.
    """
    timestamp = int(time.time())
    value = datetime.datetime.fromtimestamp(timestamp)
    str_time = value.strftime('%Y-%m-%d %H:%M:%S')
    return str_time

def get_est_time_str(offset):
    """
    Date string.
    This function is used to get the date after adding the offset date and
    extract current time in yyyymmdd format.

    Parameter:
    offset(int): Days which are to be added to the date.
    Returns:
    The date after offset number of days.
    """
    timestamp = int(time.time())
    value = datetime.datetime.fromtimestamp(timestamp) + datetime.timedelta(offset)
    str_time = value.strftime('%Y-%m-%d')
    return str_time

def main():
    """
    Main Function
    """
    relative_path = rospkg.RosPack()

    arg_package_path = relative_path.get_path('pkg_task5')
    arg_file_path = arg_package_path + '/config/ur5_2_saved_trajectories/'

    ur5 = Ur5Moveit('ur5_2')

    rospy.Subscriber('/eyrc/vb/logical_camera_2', LogicalCameraImage,
                     ur5.func_callback_logical_camera)
    rospy.Subscriber('/topic_package_details', packageMsg,
                     ur5.func_callback_package_details)

    count = 0
    color = "zero"
    arg_file_name = color+"_to_drop.yaml"
    ur5.hard_play_planned_path(arg_file_path, arg_file_name, 100)

    while count < 9:
        rospy.sleep(0.01)
        if ur5.pkg_detect_flag:
            rospy.loginfo(count)
            rospy.sleep(0.6)                # Delay to reach centre
            set_conveyor_belt_speed(0)  # Stop belt

            color = ur5.color_w[0]
            ur5.pkg_detect_flag = False
            rospy.sleep(0.3)
            activate_vacuum_gripper(True)   # Activate gripper
            set_conveyor_belt_speed(50)
            arg_file_name = "drop_to_int.yaml"
            ur5.hard_play_planned_path(arg_file_path, arg_file_name, 100)
            if color == "red":

                arg_file_name = "int_to_"+color+".yaml"
                ur5.hard_play_planned_path(arg_file_path,
                                           arg_file_name, 100)      # Go to bin
                activate_vacuum_gripper(False)  # Deactivate gripper


            elif color == "yellow":
                #set_conveyor_belt_speed(80)
                arg_file_name = "int_to_"+color+".yaml"
                ur5.hard_play_planned_path(arg_file_path,
                                           arg_file_name, 100)      # Go to bin
                activate_vacuum_gripper(False)  # Deactivate gripper

            elif color == "green":

                arg_file_name = "int_to_"+color+".yaml"
                ur5.hard_play_planned_path(arg_file_path,
                                           arg_file_name, 100)      # Go to bin
                activate_vacuum_gripper(False)  # Deactivate gripper

            count = count + 1
            rospy.sleep(0.1)
            arg_file_name = color+"_to_drop.yaml"
            ur5.hard_play_planned_path(arg_file_path, arg_file_name, 100)
            set_conveyor_belt_speed(100)    # Resume conveyor
            ur5.func_publish_orders_shipped(ur5.city_w[0], color, ur5.orderid_w[0])
            ur5.color_w.pop(0)
            ur5.city_w.pop(0)
            ur5.orderid_w.pop(0)

    del ur5


if __name__ == '__main__':
    main()
