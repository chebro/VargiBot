#! /usr/bin/env python

"""
This program intends to control UR5_2 arm.
UR5_2 arm function: To sort the packages according to their respective colours.

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

from pkg_ros_iot_bridge.msg import msgRosIotAction, msgRosIotGoal, msgRosIotResult

class Ur5Moveit(object):
    """
    This class defines the robot and planning function of the ur5 arm.
    """
    # pylint: disable=too-many-instance-attributes
    def __init__(self, arg_robot_name):
        """
        Constructor

        This intiates the robot and required arguments in this class.
        """
        rospy.init_node('node_ur5_2', anonymous=True)

        self._ac_ros_iot = actionlib.ActionClient('/action_ros_iot', msgRosIotAction)

        self._robot_ns = '/'  + arg_robot_name
        self._planning_group = "manipulator"

        moveit_commander.roscpp_initialize(sys.argv)
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

        self.pkgname_w = []
        self.color_w = []
        self.city_w = []
        self.orderid_w = []

        param_config_iot = rospy.get_param('config_iot')
        self._config_shipped_pub_topic = param_config_iot['http']['shipped']
        self._goal_handles = {}
        self._ac_ros_iot.wait_for_server()

        rospy.loginfo("Action server is up, we can send new goals!")

        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

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

    def func_callback_logical_camera_2(self, LogicalCameraImage):
        """
        Callback Function for Logical Camera Subscription

        This method is used for determining the presence of a package on the conveyor belt. 

        Parameters: 
            LogicalCameraImage: This is a LogicalCameraImage received from the logical camera containing the 
                 the objects and their position and orientation on the conveyor belt.

        """
        # if LogicalCameraImage.models and LogicalCameraImage.models[-1].type != 'ur5':
        #     while LogicalCameraImage.models[-1].type != self.pkgname_w[0]:
        #         self.pkgname_w.pop()
        #         self.color_w.pop()
        #         self.city_w.pop()
        #         self.orderid_w.pop()
        self.pkg_detect_flag = LogicalCameraImage.models and LogicalCameraImage.models[-1].type != 'ur5'

    def func_callback_package_details(self, msg):
        """
        Callback function
        
        This method assigns the string which is published on the topic topic_package_details.
        
        Parameters:
            msg(string): Msg obtained from the topic topic_shipped_data containing various attributes.
                packageName(string): Name of the package sent,
                color(string): Color of the package,
                city(string): Destination city of the package,
                orderid(string): Order ID assigned to the package.
        """
        self.pkgname_w.append(msg.packageName)
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

        ship_info = str(ship_obj)
        ship_goal_handle = self.send_spreadsheet_pub_goal("http",
                                                          "pub",
                                                          self._config_shipped_pub_topic,
                                                          ship_info)
        ship_goal_key = str(len(self._goal_handles))
        self._goal_handles[ship_goal_key] = ship_goal_handle
        rospy.loginfo('Goal ' + ship_goal_key + ' Sent')

    def send_spreadsheet_pub_goal(self, arg_protocol, arg_mode, arg_topic, arg_message):
        """
        Method to send Goals to Action Server: /action_ros_iot

        This method is called for publishing the messages to the action server.

        Parameters:
            arg_protocol(string): Protocol used for sending the data. eg: mqtt, http.
            arg_mode(string): Mode of communication. eg: pub:Publishing the data, sub: Subscribing
            arg_topic(string): Name of the channel of communication.
            arg_message(string): Message to be sent through the topic.
    
        Return: 
            goal handle: A goal handle is returned.
        NOTE:
            The data type of arg_message is dependent on the defination of the message. In this case we are 
            using a string. 
        """
        goal = msgRosIotGoal()

        goal.protocol = arg_protocol
        goal.mode = arg_mode
        goal.topic = arg_topic
        goal.message = arg_message

        rospy.loginfo("Sending HTTP Goal.")

        goal_handle = self._ac_ros_iot.send_goal(goal, self.on_transition, None)
        return goal_handle

    def on_transition(self, goal_handle):
        """
        State Machine : /action_ros_iot

        This method will be called when there is a change of state in the Action Client.

        Parameters:
            goal_handle: This is a structure containing attributes related to the goal sent.
        
        """
        result = msgRosIotResult()

        index = 0
        for i in self._goal_handles:
            if self._goal_handles[i] == goal_handle:
                index = i
                break

        rospy.loginfo("Transition Callback. Client Goal Handle #: " + str(index))
        rospy.loginfo("Comm. State: " + str(goal_handle.get_comm_state()))
        rospy.loginfo("Goal Status: " + str(goal_handle.get_goal_status()))

        # if (Comm State == ACTIVE)
        if goal_handle.get_comm_state() == 2:
            rospy.loginfo(": Goal " + str(index) + " is active.")

        # if (Comm State == DONE)
        if goal_handle.get_comm_state() == 7:
            rospy.loginfo(str(index) + ": Goal is DONE")
            rospy.loginfo(goal_handle.get_terminal_state())

            # get_result() gets the result produced by the Action Server
            result = goal_handle.get_result()
            rospy.loginfo(result.flag_success)

            if result.flag_success:
                rospy.loginfo("Goal Suceeded. Client Goal Handle #: " + str(index))
            else:
                rospy.loginfo("Goal failed. Client Goal Handle #: " + str(index))

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
        
    This function enables and disables the vaccum gripper on the arm.
       
    Parameters:
        state(bool): Required state of the vaccum gripper.
        
    Returns:
        The service of activating the vaccum gripper is provided.
    """
    rospy.wait_for_service('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_2')
    try:
        activate_vacuum_gripper_service = rospy.ServiceProxy(
            '/eyrc/vb/ur5/activate_vacuum_gripper/ur5_2', vacuumGripper
            )
        res = activate_vacuum_gripper_service(state)
        return res
    except rospy.ServiceException as err:
        print("Service call failed: %s" + err)

def set_conveyor_belt_speed(speed):
    """
    Control Conveyor Belt Speed
    This function allows us to control the speed of the conveyor belt
    using the service /eyrc/vb/conveyor/set_power.

    Parameters:
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
        print("Service call failed: %s" + err)

def get_time_str():
    """
    Date string.
    
    This function is used to get the current time and time in yyyymmdd format.
    
    Returns:
        string of the data and time.
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
    
    Parameters:
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
                     ur5.func_callback_logical_camera_2)
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
            set_conveyor_belt_speed(100)
            arg_file_name = "drop_to_int.yaml"
            ur5.hard_play_planned_path(arg_file_path, arg_file_name, 100)
            if color == "red":

                arg_file_name = "int_to_"+color+".yaml"
                ur5.hard_play_planned_path(arg_file_path,
                                           arg_file_name, 100)      # Go to bin
                activate_vacuum_gripper(False)  # Deactivate gripper


            elif color == "yellow":
                set_conveyor_belt_speed(70)
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
