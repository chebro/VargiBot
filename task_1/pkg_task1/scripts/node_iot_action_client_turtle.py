#!/usr/bin/env python

"""
ROS Node - Simple Iot Action Client - Turtle
"""

import time
import rospy
import actionlib

from pkg_iot_ros_bridge.msg import msgMqttSub
from pkg_task1.msg import msgTurtleAction
from pkg_task1.msg import msgTurtleGoal
from pkg_ros_iot_bridge.msg import msgRosIotAction
from pkg_ros_iot_bridge.msg import msgRosIotGoal
from pkg_ros_iot_bridge.msg import msgRosIotResult
# from pkg_ros_iot_bridge.msg import msgRosIotFeedback


class SimpleActionClientTurtle:
    """
    Simple Action Client Class : /action_turtle, /action_ros_iot, /ros_iot_bridge/mqtt/sub
    """
    def __init__(self):
        self._ac = actionlib.SimpleActionClient('/action_turtle', msgTurtleAction)
        self._ac_ros_iot = actionlib.ActionClient('/action_ros_iot', msgRosIotAction)

        # Read and Store IoT Configuration data from Parameter Server
        param_config_iot = rospy.get_param('config_iot')
        self._config_mqtt_pub_topic = param_config_iot['mqtt']['topic_pub']
        self._goal_handles = {}
        self.is_start_received = False # Flag that determines if the turtle should start

        self._ac.wait_for_server()
        self._ac_ros_iot.wait_for_server()

        rospy.loginfo("Action server is up, we can send new goals!")


    def send_turtle_goal(self, arg_dis, arg_angle):
        """
        Function to send Goals to Action Server : /action_turtle
        """
        goal = msgTurtleGoal(distance=arg_dis, angle=arg_angle)
        self._ac.send_goal(goal, done_cb=self.done_callback, feedback_cb=self.feedback_callback)
        rospy.loginfo("Goal to Turtle Sent.")


    def send_mqtt_pub_goal(self, arg_protocol, arg_mode, arg_topic, arg_message):
        """
        Function to send Goals to Action Server: /action_ros_iot
        """
        goal = msgRosIotGoal()

        goal.protocol = arg_protocol
        goal.mode = arg_mode
        goal.topic = arg_topic
        goal.message = arg_message

        rospy.loginfo("Sending Mqtt Goal.")

        goal_handle = self._ac_ros_iot.send_goal(goal, self.on_transition, None)
        return goal_handle


    def done_callback(self, status, result):
        """
        Function print result on Goal completion + Handle Mqtt Goal Sending
        """
        rospy.loginfo("Status is : " + str(status))
        rospy.loginfo("Result is : " + str(result))
        split_result = str(result).split("\n")
        final_x = split_result[0].split(": ")[1]
        final_y = split_result[1].split(": ")[1]
        final_theta = split_result[2].split(": ")[1]
        send_result = '(' + final_x + ',' + final_y + ',' + final_theta + ')'
        goal_handle = self.send_mqtt_pub_goal("mqtt",
                                              "pub",
                                              self._config_mqtt_pub_topic,
                                              send_result)
        key = str(len(self._goal_handles))
        self._goal_handles[key] = goal_handle
        rospy.loginfo('Goal ' + key + ' Sent')


    def feedback_callback(self, feedback):
        """
        Function to print feedback while Goal is being processedS
        """
        rospy.loginfo(feedback)


    def func_callback_mqtt_sub(self, msg):
        """
        Function to invoke when a START message is received from the MQTT bridge
        """
        rospy.loginfo("Data Received: (%s, %s)", msg.topic, msg.message)
        if msg.message == "START":
            self.is_start_received = True


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


    def start_turtle(self):
        """
        Send Goals to Draw a heaxagon : /action_turtle
        """
        self.send_turtle_goal(2, 0)
        rospy.sleep(10)
        self.send_turtle_goal(2, 60)
        rospy.sleep(10)
        self.send_turtle_goal(2, 60)
        rospy.sleep(10)
        self.send_turtle_goal(2, 60)
        rospy.sleep(10)
        self.send_turtle_goal(2, 60)
        rospy.sleep(10)
        self.send_turtle_goal(2, 60)
        rospy.spin()


def main():
    """
    Main Function
    """
    # Initialize ROS Node
    rospy.init_node('node_simple_action_client_turtle')

    # Create a object for Iot Action Client.
    obj_client = SimpleActionClientTurtle()

    # Subscribe to MQTT messages coming from node_ros_iot_action_server_bridge
    rospy.Subscriber('/ros_iot_bridge/mqtt/sub',
                     msgMqttSub, obj_client.func_callback_mqtt_sub)

    while True:
        if obj_client.is_start_received:
            obj_client.start_turtle()
        time.sleep(1)


if __name__ == '__main__':
    main()
