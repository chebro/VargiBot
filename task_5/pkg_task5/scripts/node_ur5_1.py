#! /usr/bin/env python

"""
This program intends to control UR5_1 arm.
UR5_1 arm function: To sort the packages according to their
                    respective colours.
"""

import datetime
import time
import sys
import yaml
import rospkg

from pyzbar.pyzbar import decode
import numpy as np
from cv_bridge import CvBridge, CvBridgeError

import rospy
import moveit_commander
import moveit_msgs.msg
import actionlib

from pkg_vb_sim.srv import vacuumGripper, conveyorBeltPowerMsg
from pkg_vb_sim.msg import LogicalCameraImage

from sensor_msgs.msg import Image
from pkg_task5.msg import packageMsg, incomingMsg

from pkg_ros_iot_bridge.msg import msgRosIotAction, msgRosIotGoal, msgRosIotResult

class PriorityQueue(object):
    def __init__(self):
        self.queue = []

    def __str__(self):
        return ' '.join([str(i) for i in self.queue])
    # for checking if the queue is empty
    def is_empty(self):
        return len(self.queue) == 0

    # for inserting an element in the queue
    def insert(self, data):
        self.queue.append(data)
    # for popping an element based on Priority
    def delete(self):
        try:
            max_index = 0
            for i in range(len(self.queue)):
                if self.queue[i]["Cost"] > self.queue[max_index]["Cost"]:
                    max_index = i
                elif self.queue[i]["Cost"] == self.queue[max_index]["Cost"]:
                    if self.queue[i]['Order Date and Time'] < self.queue[i]['Order Date and Time']:
                        max_index = i
            item = self.queue[max_index]
            del self.queue[max_index]
            return item
        except IndexError:
            print()
            exit()

class Ur5Moveit(object):
    """
    Constructor
    """
    # pylint: disable=too-many-instance-attributes
    def __init__(self, arg_robot_name):

        rospy.init_node('node_ur5_1', anonymous=True)

        self._ac_ros_iot = actionlib.ActionClient('/action_ros_iot', msgRosIotAction)

        self._robot_ns = '/'  + arg_robot_name
        self._planning_group = "manipulator"

        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander(
        	   robot_description=self._robot_ns + "/robot_description", ns=self._robot_ns)
        self._scene = moveit_commander.PlanningSceneInterface(ns=self._robot_ns)
        self._group = moveit_commander.MoveGroupCommander(
        	   self._planning_group, robot_description=self._robot_ns + "/robot_description",
        	   ns=self._robot_ns)
        self._display_trajectory_publisher = rospy.Publisher(
        	   self._robot_ns + '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory,
        	   queue_size=1)
        self._exectute_trajectory_client = actionlib.SimpleActionClient(
        	   self._robot_ns + '/execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()

        #All the box coordinates:

        self._box_x = [0.28, 0, -0.28]
        self._box_y = 6.59-7.00
        self._box_z = [1.92, 1.65, 1.43, 1.20]

        #incoming orders queue

        self._orders = PriorityQueue()

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


        relative_path = rospkg.RosPack()
        self._pkg_path = relative_path.get_path('pkg_moveit_examples')
        self._file_path = self._pkg_path + '/config/'
        rospy.loginfo("Package Path: {}".format(self._file_path))

        param_config_iot = rospy.get_param('config_iot')
        self._config_inventory_pub_topic = param_config_iot['http']['inventory']
        self._config_dispatch_pub_topic = param_config_iot['http']['dispatch']
        self._goal_handles = {}
        self._ac_ros_iot.wait_for_server()

        rospy.loginfo("Action server is up, we can send new goals!")

        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

    def play_planned_path_from_file(self, arg_file_path, arg_file_name):
        """
        Loading the trajectories from the file and executing
        """
        file_path = arg_file_path + arg_file_name

        with open(file_path, 'r') as file_open:
            plan = yaml.load(file_open)

        ret = self._group.execute(plan)

        return ret

    def hard_play_planned_path_from_file(self, arg_file_path, arg_file_name, arg_max_attempts):
        number_attempts = 0
        flag_success = False

        while ((number_attempts <= arg_max_attempts) and (flag_success is False)):
            number_attempts += 1
            flag_success = self.play_planned_path_from_file(arg_file_path, arg_file_name)
            rospy.logwarn("attempts: {}".format(number_attempts))
        return True

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

    def func_callback_topic_logical_camera_1(self, LogicalCameraImage):

        if len(LogicalCameraImage.models) == 0:
            self.presence_of_package = False
        else:
            self.presence_of_package = True

    def func_callback_topic_incoming_orders(self, data):

        incoming_dict = eval(data.incomingData)
        rospy.logwarn('\n\nATTENTION! NEW ORDER!\n\n')
        rospy.logwarn(incoming_dict)

        self._orders.insert(incoming_dict)

    def send_spreadsheet_pub_goal(self, arg_protocol, arg_mode, arg_topic, arg_message):
        """
        Function to send Goals to Action Server: /action_ros_iot
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
        This function will be called when there is a change of state in the Action Client
        State Machine : /action_ros_iot
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

class Camera2D(object):
    """docstring for Camera2D"""
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(
        	   "/eyrc/vb/camera_1/image_raw", Image, self.func_callback_2D_camera)
        self.image = []

    def get_qr_data(self, arg_image):
        qr_result = decode(arg_image)

        if qr_result:
            rospy.logwarn(qr_result[0].data)
            color = qr_result[0].data
        else:
          # setting values for base colors
            y_color = arg_image[:, :, :2]
            g_color = arg_image[:, :, 1:2]
            r_color = arg_image[:, :, 2:]

            # computing the mean
            y_mean = np.mean(y_color)
            g_mean = np.mean(g_color)
            r_mean = np.mean(r_color)

            # displaying the most prominent color
            if (g_mean > r_mean and g_mean > y_mean):
                rospy.logwarn('\n\nDETECTED GREEN\n\n')
                color = 'green'
            elif (y_mean > r_mean and y_mean > g_mean):
                rospy.logwarn('\n\nDETECTED YELLOW\n\n')
                color = 'yellow'
            elif (r_mean > y_mean and r_mean > g_mean):
                rospy.logwarn('\n\nDETECTED RED\n\n')
                color = 'red'
            else:
                color = 'NA'
                rospy.logwarn('\n\nDETECTED NIL\n\n')

        return color

    def func_callback_2D_camera(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as error:
            rospy.logerr(error)

        image = cv_image

        req_imag = image[301:900, 100:600, :]

        self.image = req_imag

    def __del__(self):
        rospy.loginfo('Information Received.')

def activate_vacuum_gripper(state):
    """
    Enable/Disable Gripper Module
    """
    rospy.wait_for_service('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1')
    try:
        activate_vacuum_gripper_service = rospy.ServiceProxy(
            '/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1', vacuumGripper
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

def get_sku_str():
    timestamp = int(time.time())
    value = datetime.datetime.fromtimestamp(timestamp)
    str_time = value.strftime('%m%y')

    return str_time

def get_time_str():
    timestamp = int(time.time())
    value = datetime.datetime.fromtimestamp(timestamp)
    str_time = value.strftime('%Y-%m-%d %H:%M:%S')

    return str_time


def main():
    """
    Main Function
    """
    relative_path = rospkg.RosPack()

    arg_package_path = relative_path.get_path('pkg_task5')
    arg_file_path = arg_package_path + '/config/ur5_1_saved_trajectories/'

    two_dim_camera = Camera2D()
    ur5 = Ur5Moveit('ur5_1')

    rospy.sleep(5)

    color_pub = rospy.Publisher('topic_package_details', packageMsg, queue_size=10)

    rospy.Subscriber(
    	   '/topic_incoming_orders', incomingMsg, ur5.func_callback_topic_incoming_orders)
    rospy.Subscriber(
    	   '/eyrc/vb/logical_camera_1', LogicalCameraImage, ur5.func_callback_topic_logical_camera_1)

    """
    Inventory data publisher.
    """

    red = []
    yellow = []
    green = []

    inv_obj = {}

    for i in range(3):
        for j in reversed(range(3)):
            color = two_dim_camera.get_qr_data(
            	   two_dim_camera.image[i*150:i*150+149, j*167: (j+1)*167-1, :])
            inv_obj['Team Id'] = 'VB#1004'
            inv_obj['Unique Id'] = 'CeAhsAGA'
            inv_obj['id'] = 'Inventory'

            if color == 'red':
                inv_obj['SKU'] = 'R'+str(i)+str(j)+get_sku_str()
                inv_obj['Item'] = 'Medicine'
                inv_obj['Priority'] = 'HP'
                inv_obj['Cost'] = 300
                red.append((i, j))

            if color == 'yellow':
                inv_obj['SKU'] = 'Y'+str(i)+str(j)+get_sku_str()
                inv_obj['Item'] = 'Food'
                inv_obj['Priority'] = 'MP'
                inv_obj['Cost'] = 200
                yellow.append((i, j))

            if color == 'green':
                inv_obj['SKU'] = 'G'+str(i)+str(j)+get_sku_str()
                inv_obj['Item'] = 'Clothes'
                inv_obj['Priority'] = 'LP'
                inv_obj['Cost'] = 100
                green.append((i, j))

            inv_obj['Storage Number'] = 'R'+str(i)+'C'+str(j)
            inv_obj['Quantity'] = 1
            rospy.sleep(1.5)
            inv_info = str(inv_obj)

            inv_goal_handle = ur5.send_spreadsheet_pub_goal("http",
                                                            "pub",
                                                            ur5._config_inventory_pub_topic,
                                                            inv_info)
            key = str(len(ur5._goal_handles))
            ur5._goal_handles[key] = inv_goal_handle
            rospy.loginfo('Goal ' + key + ' Sent')

    set_conveyor_belt_speed(100)
    package_info = packageMsg()

    count = 0

    while not rospy.is_shutdown():

        if not ur5._orders.is_empty() and count < 9:

            order = ur5._orders.delete()
            rospy.logwarn(ur5._orders)

            if order["Priority"] == 'HP':
                (i, j) = red[0]
                red.pop(0)
                color = 'red'

            if order["Priority"] == 'MP':
                (i, j) = yellow[0]
                yellow.pop(0)
                color = 'yellow'

            if order["Priority"] == 'LP':
                (i, j) = green[0]
                green.pop(0)
                color = 'green'

            package_info.packageName = 'Packagen'+str(i)+str(j)
            package_info.color = color
            package_info.city = order["City"]
            package_info.orderid = order["Order ID"]
            '''The arm files are to be change'''

            if count == 0:
                arg_file_name = 'zero_to_packagen' + str(i) + str(j) + '.yaml'
            else:
                arg_file_name = 'drop_to_packagen' + str(i) + str(j) + '.yaml'

            ur5.hard_play_planned_path_from_file(arg_file_path, arg_file_name, 30)
            rospy.logwarn('\n\nArm is going to ' + str(i)+ str(j)+'\n\n')
            activate_vacuum_gripper(True)
            rospy.sleep(0.5)

            while ur5.presence_of_package:          #Avoiding: Package being placed over a package
                rospy.sleep(1)

            arg_file_name = 'packagen' + str(i) + str(j) + '_to_drop.yaml'
            ur5.hard_play_planned_path_from_file(arg_file_path, arg_file_name, 30)
            activate_vacuum_gripper(False)
            color_pub.publish(package_info)

            dispatch_info_dict = {
                'id' : 'OrdersDispatched',
                'Team Id': order["Team Id"],
                'Unique Id': order["Unique Id"],
                'Order ID': order["Order ID"],
                'City': order['City'],
                'Item': order['Item'],
                'Priority': order['Priority'],
                'Dispatch Quantity': 1,
                'Cost' : order['Cost'],
                'Dispatch Status': 'YES',
                'Dispatch Date and Time': get_time_str()
            }

            rospy.logwarn('\n\nDISPATCHING THE FOLLOWING!\n\n')
            rospy.logwarn(str(dispatch_info_dict))
            dispatch_info = str(dispatch_info_dict)
            disp_goal_handle = ur5.send_spreadsheet_pub_goal("http",
                                                             "pub",
                                                             ur5._config_dispatch_pub_topic,
                                                             dispatch_info)
            disp_goal_key = str(len(ur5._goal_handles))
            ur5._goal_handles[disp_goal_key] = disp_goal_handle
            rospy.loginfo('Goal ' + disp_goal_key + ' Sent')

            rospy.sleep(0.5)
            count = count + 1
        else:
            if count > 8:
                break
            pass
    #ur5.go_to_predefined_pose("allZeros")
    rospy.sleep(1)  #waiting for the arm to go to required position

    del ur5

    del two_dim_camera

if __name__ == '__main__':
    main()
