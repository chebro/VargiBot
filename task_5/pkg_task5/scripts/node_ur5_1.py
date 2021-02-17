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

from std_msgs.msg import String
from sensor_msgs.msg import Image
from pkg_task5.msg import packageMsg
from pkg_task5.msg import inventoryMsg
from pkg_ros_iot_bridge import incomingMsg
from pkg_task5.msg import dispatchMsg

from cv_bridge import CvBridge, CvBridgeError
from pyzbar.pyzbar import decode
import numpy as np

from datetime import date
import json

class PriorityQueue: 
    def __init__(self): 
        self.queue = [] 
  
    def __str__(self): 
        return ' '.join([str(i) for i in self.queue]) 
  
    # for checking if the queue is empty 
    def isEmpty(self): 
        return len(self.queue) == 0
  
    # for inserting an element in the queue 
    def insert(self, data): 
        self.queue.append(data) 
  
    # for popping an element based on Priority 
    def delete(self): 
        try: 
            max = 0
            for i in range(len(self.queue)): 
                if self.queue[i][Cost] > self.queue[max][Cost]: 
                    max = i 
                elif self.queue[i][Cost] == self.queue[max][Cost]:
                    if self.queue[i]['Order Date and Time'] < self.queue[i]['Order Date and Time']:
                        max = i
            item = self.queue[max] 
            del self.queue[max] 
            return item 
        except IndexError: 
            print() 
            exit() 


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

        #Box names
        self._box_name = [ ['', '', '', ''],
                        ['', '', '', ''],
                        ['', '', '', '']]

        #All the box coordinates:

        self._box_x = [0.28, 0, -0.28]
        self._box_y = 6.59-7.00
        self._box_z = [1.92, 1.65, 1.43, 1.20]

        #incoming orders queue

        self._orders = PriorityQueue()

        #Setup for tf
        self._tfBuffer = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._tfBuffer)

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


        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

    def clear_octomap(self):
        clear_octomap_service_proxy = rospy.ServiceProxy(self._robot_ns + "/clear_octomap", Empty)
        return clear_octomap_service_proxy()

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

    def attach_box(self, i, j, timeout=4):
        """
        Attaching Objects to the Robot
        """
        grasping_group = "manipulator"
        touch_links = self._robot.get_link_names(group=grasping_group)
        self._scene.attach_box(self._eef_link, self._box_name[i][j], touch_links=touch_links)
        rospy.loginfo('The box attached')

    def detach_box(self, i, j, timeout=4):
        """
        Detaching Objects from the Robot
        """
        self._scene.remove_attached_object(self._eef_link, name=self._box_name[i][j])

    def set_joint_angles(self, arg_list_joint_angles, arg_file_path, arg_file_name):
        """
        Plan and Execute : Set Joint Angles
        """
        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Current Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        self._group.set_joint_value_target(arg_list_joint_angles)
        plan = self._group.plan()
        self.save_trag(arg_file_path, arg_file_name, plan)
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

    def func_callback_topic_logical_camera_1(self, LogicalCameraImage):

        if len(LogicalCameraImage.models) == 0:
            self.presence_of_package = False
        else:
            self.presence_of_package = True


    def activate_vacuum_gripper(self, state):
        """
        Enable/Disable Gripper Module
        """
        if state:
          rospy.loginfo('Activated.')
        else:
          rospy.loginfo('Deactivated')
        rospy.wait_for_service('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1')


        try:
            activate_vacuum_gripper = rospy.ServiceProxy('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1',
                                                         vacuumGripper)
            resp1 = activate_vacuum_gripper(state)
            return resp1
        except rospy.ServiceException as e:
            print "Service call failed: %s" + e


    def activate_conveyor_belt(self, state):
        """
        Enable/Disable Gripper Module
        """
        rospy.loginfo('Activated conveyor belt.')
        rospy.wait_for_service('/eyrc/vb/conveyor/set_power')


        try:
            activate_vacuum_gripper = rospy.ServiceProxy('/eyrc/vb/conveyor/set_power',
                                                         conveyorBeltPowerMsg)
            resp1 = activate_vacuum_gripper(state)
            return resp1
        except rospy.ServiceException as e:
            print "Service call failed: %s" + e

    def hard_set_joint_angles(self, arg_list_joint_angles, arg_max_attempts, arg_file_path, arg_file_name):

        number_attempts = 0
        flag_success = False

        while ( (number_attempts <= arg_max_attempts) and  (flag_success is False) ):
            number_attempts += 1
            flag_success = self.set_joint_angles(arg_list_joint_angles, arg_file_path, arg_file_name)
            rospy.logwarn("attempts: {}".format(number_attempts) )

    def func_callback_topic_incoming_orders(data):

        incomingDict = eval(data.incomingData)

        self._orders.insert(incomingData)


    def __del__(self):
        """
        Destructor
        """
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')

class Camera2D:
    """docstring for Camera2D"""
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/eyrc/vb/camera_1/image_raw", Image,self.callback)
        self.image = []

    def get_qr_data(self, arg_image):
        qr_result = decode(arg_image)

        if ( len( qr_result ) > 0):
            return (qr_result[0].data)
        else :
          # setting values for base colors
            y = arg_image[:, :, :2]
            g = arg_image[:, :, 1:2]
            r = arg_image[:, :, 2:]

            # computing the mean
            y_mean = np.mean(y)
            g_mean = np.mean(g)
            r_mean = np.mean(r)

            # displaying the most prominent color
            if (g_mean > r_mean and g_mean > y_mean):
                return 'green'
            elif (y_mean > r_mean and y_mean > g_mean):
                return 'yellow'
            elif (r_mean > y_mean and r_mean > g_mean):
                return 'red'
            else:
                return 'NA'


    def callback(self,data):
        try:
          cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
          rospy.logerr(e)

        (rows,cols,channels) = cv_image.shape

        image = cv_image

        # Resize a 720x1280 image to 360x640 to fit it on the screen
        resized_image = cv2.resize(image, (720/2, 1280/2))

        req_imag = image[ 301:900, 100:600, :]

        self.image = req_imag
        cv2.imshow("/eyrc/vb/camera_1/image_raw", resized_image)

        cv2.waitKey(3)

    def __del__(self):
        rospy.loginfo('Information Received.')

  

def get_time_str():
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

    rp = rospkg.RosPack()

    arg_package_path = rp.get_path('pkg_task5')
    arg_file_path = arg_package_path + '/config/ur5_1_saved_trajectories/'

    rospy.sleep(10)

    ic = Camera2D()
    ur5 = Ur5Moveit('ur5_1')

    inv_pub = rospy.Publisher('topic_inventory_data', inventoryMsg, queue_size = 10)

    """
    Inventory data publisher.
    """

    red = []
    yellow = []
    green = []

    inv_obj = {}

    for i in range(3):
        for j in range(3):
            color = ic.get_qr_data(ic.image[i*150:i*150+149, j*167: (j+1)*167-1, :])
            
            inv_obj['Team ID'] = 'VB#1004'
            inv_obj['Unique Id'] = 'CeAhsAGA'
            inv_obj['SKU'] = color.upper()[0]+str(i)+str(j)+get_time_str

            if color == 'red':
                inv_obj['Item'] = 'Medicine'
                inv_obj['Priority'] = 'HP'
                inv_obj['Cost'] = 300
                red.append((i, j))

            if color == 'yellow':
                inv_obj['Item'] = 'Food'
                inv_obj['Priority'] = 'MP'
                inv_obj['Cost'] = 200
                yellow.append((i, j))
            if color == 'green':
                inv_obj['Item'] = 'Clothes'
                inv_obj['Priority'] = 'LP'
                inv_obj['Cost'] = 100
                green.append((i, j))

            inv_obj['Storage Number'] = 'R'+i+'C'+j
            inv_obj['Quantity'] = 1
            inv_obj['id'] = 'Inventory'
            str_inv_obj = str(inv_obj)

            inv_pub.publish(str_inv_obj)
            


    color_pub = rospy.Publisher('topic_package_details', packageMsg, queue_size = 10)
    dispatch_pub = rospy.Publisher('topic_dispatch_orders', dispatchMsg, queue_size = 10)

    rospy.Subscriber('topic_incoming_orders', incomingMsg, ur5.func_callback_topic_incoming_orders)
    rospy.Subscriber('/eyrc/vb/logical_camera_1', LogicalCameraImage, ur5.func_callback_topic_logical_camera_1)

    ur5.activate_conveyor_belt(100)
    package_info = packageMsg()
    dispatch_info = dispatchMsg()

    while not rospy.is_shutdown():

        if not ur5._orders.isEmpty():

            order = ur5._orders.delete()
            
            if order.Priority == 'HP':
                (i, j) = red[0]
                red.pop(0)
                color = 'red'
            if order.Priority == 'MP':
                (i, j) = yellow[0]
                yellow.pop(0)
                color = 'yellow'

            if order.Priority == 'LP':
                (i, j) = green[0]
                green.pop(0)
                color = 'green'

            package_info.packageName = 'Packagen'+str(i)+str(j)
            package_info.color = color
            package_info.city = order[City]
            '''The arm files are to be change'''
            '''
            if(j == 2 and i == 0):
                arg_file_name = 'zero_to_packagen02.yaml'
            else:
                arg_file_name = 'drop_to_packagen' + str(i) + str(j) + '.yaml'
                ur5.moveit_hard_play_planned_path_from_file(arg_file_path, arg_file_name, 9)
                ur5.attach_box(i = j, j = i)
                ur5.activate_vacuum_gripper(True)
                rospy.sleep(0.5)

                while ur5.presence_of_package:          #Avoiding: Package being placed over a package
                    rospy.sleep(1)
            '''
                arg_file_name = 'packagen' + str(i) + str(j) + '_to_drop.yaml'
                ur5.moveit_hard_play_planned_path_from_file(arg_file_path, arg_file_name, 9)
                ur5.activate_vacuum_gripper(False)
                ur5.detach_box(i = j, j = i)
                color_pub.publish(package_info)
                dispatch_info_dict = {
                    'id' : 'OrdersDispatched'
                    'Team ID': order["Team ID"]
                    'Unique ID': order["Unique ID"]
                    'Order ID': order["Order ID"]
                    'City': order['City']
                    'Item': order['Item']
                    'Priority': order['Priority']
                    'Dispatch Quantity': 1
                    'Cost' : order['Cost']
                    'Dispatch Status': order['Dispatch Status']
                    'Dispatch Date and Time': get_time_str()

                }

                dispatch_info = str(dispatch_info_dict)
                dispatch_pub.publish(dispatch_info)

                rospy.sleep(0.5)
        else
            pass
    
    #ur5.go_to_predefined_pose("allZeros")
    rospy.sleep(1)  #waiting for the arm to go to required position

    del ur5

    del ic

if __name__ == '__main__':
    main()

