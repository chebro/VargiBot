#! /usr/bin/env python

"""
ROS Node - Sorting Boxes Module - MoveIt!
"""

import sys
import math
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib

import tf2_ros
import tf2_msgs.msg

from pkg_vb_sim.srv import vacuumGripper
from pkg_vb_sim.srv import conveyorBeltPowerMsg
from pkg_vb_sim.msg import LogicalCameraImage

class Ur5Moveit:
    """
    Ur5Moveit Class Definition
    """
    def __init__(self):
        rospy.init_node('node_eg4_go_to_pose', anonymous=True)

        self._planning_group = "ur5_1_planning_group"
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        self._group = moveit_commander.MoveGroupCommander(self._planning_group)
        self._display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

        self._exectute_trajectory_client = actionlib.SimpleActionClient(
            'execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()
        self._box_name = ''

        # Current State of the Robot is needed to add box to planning scene
        self._curr_state = self._robot.get_current_state()

        # Package state flags
        self._pkg_detect_flag = [False, False, False]
        self._pkg_pickup_flag = [False, False, False]
        self._curr_pkg_pose = [0, 0, 0]

        # TF
        self._tf_buffer = tf2_ros.Buffer()
        self.__transform_listener = tf2_ros.TransformListener(self._tf_buffer)

        rospy.loginfo(
            '\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')

        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

    def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
        """
        Ensuring Collision Updates Are Received
        """
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            attached_objects = self._scene.get_attached_objects([self._box_name])
            is_attached = len(attached_objects.keys()) > 0

            is_known = self._box_name in self._scene.get_known_object_names()

            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True

            rospy.sleep(0.1)
            seconds = rospy.get_time()

        return False

    def attach_box(self, timeout=4):
        """
        Attaching Objects to the Robot
        """
        grasping_group = "ur5_1_planning_group"
        touch_links = self._robot.get_link_names(group=grasping_group)
        self._scene.attach_box(self._eef_link, self._box_name, touch_links=touch_links)

        return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)


    def detach_box(self, timeout=4):
        """
        Detaching Objects from the Robot
        """
        self._scene.remove_attached_object(self._eef_link, name=self._box_name)

        return self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout)

    def ee_cartesian_translation(self, trans_x, trans_y, trans_z):
        """
        Cartesian Translation
        """
        # 1. Create a empty list to hold waypoints
        waypoints = []

        # 2. Add Current Pose to the list of waypoints
        waypoints.append(self._group.get_current_pose().pose)

        # 3. Create a New waypoint
        wpose = geometry_msgs.msg.Pose()
        wpose.position.x = waypoints[0].position.x + (trans_x)
        wpose.position.y = waypoints[0].position.y + (trans_y)
        wpose.position.z = waypoints[0].position.z + (trans_z)
        wpose.orientation.x = -0.5
        wpose.orientation.y = -0.5
        wpose.orientation.z = 0.5
        wpose.orientation.w = 0.5

        # 4. Add the new waypoint to the list of waypoints
        waypoints.append(copy.deepcopy(wpose))

        # 5. Compute Cartesian Path connecting the waypoints in the list of waypoints
        (plan, fraction) = self._group.compute_cartesian_path(
            waypoints,   # waypoints to follow
            0.01,        # Step Size, distance between two adjacent computed waypoints will be 1 cm
            0.0)         # Jump Threshold
        rospy.loginfo("Path computed successfully. Moving the arm.")

        num_pts = len(plan.joint_trajectory.points)
        if num_pts >= 3:
            del plan.joint_trajectory.points[0]
            del plan.joint_trajectory.points[1]

        # 6. Make the arm follow the Computed Cartesian Path
        self._group.execute(plan)

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
        """
        rospy.loginfo('\033[94m' + "Going to Pose: {}".format(arg_pose_name) + '\033[0m')
        self._group.set_named_target(arg_pose_name)
        plan = self._group.plan()
        goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
        goal.trajectory = plan
        self._exectute_trajectory_client.send_goal(goal)
        self._exectute_trajectory_client.wait_for_result()
        rospy.loginfo('\033[94m' + "Now at Pose: {}".format(arg_pose_name) + '\033[0m')

    def func_detect_package(self, models, number):
        """
        Function to detect if package is under camera
        """
        for i in models:
            if int(i.type[-1:]) == number:
                return True
        return False

    def func_callback_logical_camera(self, msg):
        """
        Callback Function for Logical Camera Subscription
        """
        if msg.models:
            rospy.loginfo("\n\nPACKAGE DETECTED\n\n")
            rospy.loginfo(msg.models[0])
            rospy.loginfo(self._pkg_detect_flag)
            print msg.models[0].pose.position
            if self.func_detect_package(msg.models, 1) and not self._pkg_pickup_flag[0]:
                self._pkg_detect_flag[0] = True
            elif self.func_detect_package(msg.models, 2) and not self._pkg_pickup_flag[1]:
                self._pkg_detect_flag[1] = True
            elif self.func_detect_package(msg.models, 3) and not self._pkg_pickup_flag[2]:
                self._pkg_detect_flag[2] = True
            self._curr_pkg_pose = [msg.models[0].pose.position.x, msg.models[0].pose.position.y, msg.models[0].pose.position.z]

    def activate_vacuum_gripper(self, state):
        """
        Enable/Disable Gripper Module
        """
        rospy.wait_for_service('/eyrc/vb/ur5_1/activate_vacuum_gripper')
        try:
            activate_vacuum_gripper = rospy.ServiceProxy('/eyrc/vb/ur5_1/activate_vacuum_gripper',
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

    def func_get_tf(self, arg_frame_1, arg_frame_2):
        """
        Get tf between the input arguments
        """
        try:
            trans = self._tf_buffer.lookup_transform(arg_frame_1, arg_frame_2, rospy.Time())
            rospy.loginfo("\n\nTF success\n\n")
            return [float(trans.transform.translation.x),
                    float(trans.transform.translation.y),
                    float(trans.transform.translation.z)]

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("TF error")

    def __del__(self):
        """
        Destructor
        """
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')


def main():
    """
    Main Function
    """
    rospy.sleep(10)
    ur5 = Ur5Moveit()

    rospy.Subscriber('/eyrc/vb/logical_camera_2',
                     LogicalCameraImage, ur5.func_callback_logical_camera)

    box_length = 0.15               # Length of the Package
    vacuum_gripper_width = 0.115    # Vacuum Gripper Width
    delta_z = vacuum_gripper_width + (box_length/2)

    ur5_2_home_pose = geometry_msgs.msg.Pose()
    ur5_2_home_pose.position.x = -0.8
    ur5_2_home_pose.position.y = 0
    ur5_2_home_pose.position.z = 1 + vacuum_gripper_width + (box_length/2)
    ur5_2_home_pose.orientation.x = -0.5
    ur5_2_home_pose.orientation.y = -0.5
    ur5_2_home_pose.orientation.z = 0.5
    ur5_2_home_pose.orientation.w = 0.5

    ur5_2_bin_red = copy.deepcopy(ur5_2_home_pose)
    ur5_2_bin_green = copy.deepcopy(ur5_2_home_pose)
    ur5_2_bin_blue = copy.deepcopy(ur5_2_home_pose)

    ur5_2_bin_red.position.x = 0.11
    ur5_2_bin_red.position.y = 0.65

    ur5_2_bin_green.position.x = 0.75
    ur5_2_bin_green.position.y = 0.03

    ur5_2_bin_blue.position.x = 0.04
    ur5_2_bin_blue.position.y = -0.65                      

    ur5.set_conveyor_belt_speed(100)

    while True:
        rospy.sleep(0.01)

        if ur5._pkg_detect_flag[0]:
            ur5._pkg_pickup_flag[0] = True  # Mark as "ready for pick up"
            ur5._pkg_detect_flag[0] = False # Mark as "already detected"

            rospy.sleep(0.5)                # Delay to reach centre
            ur5.set_conveyor_belt_speed(0)  # Stop belt
            # Move EE to package
            ur5.go_to_pose(ur5_2_home_pose)
            ee_delta_tf = ur5.func_get_tf("ur5_wrist_3_link", "logical_camera_2_packagen1_frame")
            ur5.ee_cartesian_translation(-ee_delta_tf[2], ee_delta_tf[0], delta_z-ee_delta_tf[1])

            ur5.activate_vacuum_gripper(True)   # Activate gripper
            ur5.go_to_pose(ur5_2_bin_red)       # Go to bin
            ur5.activate_vacuum_gripper(False)  # Deactivate gripper
            ur5.set_conveyor_belt_speed(100)    # Resume conveyor

        if ur5._pkg_detect_flag[1]:
            ur5._pkg_pickup_flag[1] = True  # Mark as "ready for pick up"
            ur5._pkg_detect_flag[1] = False # Mark as "already detected"

            rospy.sleep(0.5)                # Delay to reach centre
            ur5.set_conveyor_belt_speed(0)  # Stop belt

            # Move EE to package
            ur5.go_to_pose(ur5_2_home_pose)
            ee_delta_tf = ur5.func_get_tf("ur5_wrist_3_link", "logical_camera_2_packagen2_frame")
            ur5.ee_cartesian_translation(-ee_delta_tf[2], ee_delta_tf[0], delta_z-ee_delta_tf[1])

            ur5.activate_vacuum_gripper(True)   # Activate gripper
            ur5.go_to_pose(ur5_2_bin_green)     # Go to bin
            ur5.activate_vacuum_gripper(False)  # Deactivate gripper
            ur5.set_conveyor_belt_speed(100)    # Resume conveyor

        if ur5._pkg_detect_flag[2]:
            ur5._pkg_pickup_flag[2] = True  # Mark as "ready for pick up"
            ur5._pkg_detect_flag[2] = False # Mark as "already detected"

            rospy.sleep(0.5)                # Delay to reach centre
            ur5.set_conveyor_belt_speed(0)  # Stop belt

            # Move EE to package
            ur5.go_to_pose(ur5_2_home_pose)
            ee_delta_tf = ur5.func_get_tf("ur5_wrist_3_link", "logical_camera_2_packagen3_frame")
            ur5.ee_cartesian_translation(-ee_delta_tf[2], ee_delta_tf[0], delta_z-ee_delta_tf[1])

            ur5.activate_vacuum_gripper(True)   # Activate gripper
            ur5.go_to_pose(ur5_2_bin_blue)      # Go to bin
            ur5.activate_vacuum_gripper(False)  # Deactivate gripper
            ur5.set_conveyor_belt_speed(10)     # Resume conveyor
            break

    del ur5


if __name__ == '__main__':
    main()
