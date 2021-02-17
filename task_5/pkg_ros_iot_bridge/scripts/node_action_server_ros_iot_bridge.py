#!/usr/bin/env python

"""
ROS Node - Action Server - IoT ROS Bridge
"""

import threading
import rospy
import actionlib
import json

from pkg_task5.msg import inventoryMsg
from pkg_task5.msg import shippedMsg
from pkg_task5.msg import dispatchMsg

from pkg_ros_iot_bridge.msg import msgRosIotAction
from pkg_ros_iot_bridge.msg import msgRosIotResult
# from pkg_ros_iot_bridge.msg import msgRosIotGoal
# from pkg_ros_iot_bridge.msg import msgRosIotFeedback

from pkg_ros_iot_bridge.msg import msgMqttSub

from pyiot import iot


class IotRosBridgeActionServer:
    """
    Simple Action Server Class : /action_ros_iot, /iot_to_ros, /ros_to_iot, /mqtt/sub
    """
    def __init__(self):
        # Initialize the Action Server
        self._as = actionlib.ActionServer('/action_ros_iot',
                                          msgRosIotAction,
                                          self.on_goal,
                                          self.on_cancel,
                                          auto_start=False)

        # Read and Store IoT Configuration data from Parameter Server
        param_config_iot = rospy.get_param('config_iot')
        self._config_mqtt_unique_id = param_config_iot['mqtt_unique_id']
        self._config_mqtt_server_url = param_config_iot['mqtt']['server_url']
        self._config_mqtt_server_port = param_config_iot['mqtt']['server_port']
        self._config_mqtt_sub_topic = param_config_iot['mqtt']['topic_sub']
        self._config_mqtt_pub_topic = param_config_iot['mqtt']['topic_pub']
        self._config_mqtt_qos = param_config_iot['mqtt']['qos']
        self._config_mqtt_sub_cb_ros_topic = param_config_iot['mqtt']['sub_cb_ros_topic']
        self._config_mqtt_spread_sheet_id = param_config_iot['mqtt']['spread_sheet_id']
        self._config_mqtt_incoming_orders = param_config_iot['mqtt']['incoming_orders']
        self._config_topic_incoming_orders = param_config_iot['mqtt']['topic_incoming']

        print("\n\nMQTT PUB: " + self._config_mqtt_sub_topic)
        print("MQTT SUB: " + self._config_mqtt_pub_topic + "\n\n")
        print("Publish message START to start the turtle\n\n")
        print param_config_iot

        # Initialize ROS Topic Publication
        self._handle_ros_pub = rospy.Publisher(self._config_mqtt_sub_cb_ros_topic,
                                               msgMqttSub,
                                               queue_size=10)
        
        # Initialize Incoming Orders Publisher
        self._handle_incoming = rospy.Publisher(self._config_topic_incoming_orders,
                                                    incomingMsg,
                                                    queue_size=10)

        # # Subscribe to MQTT Topic
        # ret = iot.mqtt_subscribe_thread_start(self.mqtt_sub_callback,
        #                                       self._config_mqtt_server_url,
        #                                       self._config_mqtt_server_port,
        #                                       self._config_mqtt_sub_topic,
        #                                       self._config_mqtt_qos)

        # if ret == 0:
        #     rospy.loginfo("MQTT Subscribe Thread 0 Started")
        # else:
        #     rospy.logerr("Failed to start MQTT Subscribe Thread")

        ret1 = iot.mqtt_subscribe_thread_start(self.func_incoming_order_callback,
                                              self._config_mqtt_server_url,
                                              self._config_mqtt_server_port,
                                              self._config_mqtt_incoming_orders,
                                              self._config_mqtt_qos)

        if ret1 == 0:
            rospy.loginfo("MQTT Subscribe Thread 1 Started")
        else:
            rospy.logerr("Failed to start MQTT Subscribe Thread")

        # Start the Action Server
        self._as.start()

        rospy.loginfo("Started ROS-IoT Bridge Action Server.")

    def mqtt_sub_callback(self, client, userdata, message):
        """
        This is a callback function for MQTT Subscriptions
        """
        payload = str(message.payload.decode("utf-8"))

        print("[MQTT SUB CB] Message: ", payload)
        print("[MQTT SUB CB] Topic: ", message.topic)

        msg_mqtt_sub = msgMqttSub()
        msg_mqtt_sub.timestamp = rospy.Time.now()
        msg_mqtt_sub.topic = message.topic
        msg_mqtt_sub.message = payload

        self._handle_ros_pub.publish(msg_mqtt_sub)


    def on_goal(self, goal_handle):
        """
        This function will be called when Action Server receives a Goa
        """
        goal = goal_handle.get_goal()

        rospy.loginfo("Received new goal from Client")
        rospy.loginfo(goal)

        # Validate incoming goal parameters
        if goal.protocol == "mqtt":

            if (goal.mode == "pub") or (goal.mode == "sub"):
                goal_handle.set_accepted()

                # Start a new thread to process new goal from the client
                thread = threading.Thread(name="worker",
                                          target=self.process_goal,
                                          args=(goal_handle,))
                thread.start()

            else:
                goal_handle.set_rejected()
                return

        else:
            goal_handle.set_rejected()
            return


    def process_goal(self, goal_handle):
        """
        This function is called is a separate thread to process Goal
        """

        flag_success = False
        result = msgRosIotResult()

        goal_id = goal_handle.get_goal_id()
        rospy.loginfo("Processing goal : " + str(goal_id.id))

        goal = goal_handle.get_goal()


        # Goal Processing
        if goal.protocol == "mqtt":
            rospy.logwarn("MQTT")

            if goal.mode == "pub":
                rospy.logwarn("MQTT PUB Goal ID: " + str(goal_id.id))

                rospy.logwarn(goal.topic + " > " + goal.message)

                ret = iot.mqtt_publish(self._config_mqtt_server_url,
                                       self._config_mqtt_server_port,
                                       goal.topic,
                                       goal.message,
                                       self._config_mqtt_qos)

                if ret == 0:
                    rospy.loginfo("MQTT Publish Successful.")
                    result.flag_success = True
                    msg_split = goal.message.split(",")
                    result_x = msg_split[0].split("(")[1]
                    result_y = msg_split[1]
                    result_theta = msg_split[2].split(")")[0]
                    rospy.logwarn('MESSAGE IMP: ' +result_x+result_y+result_theta)
                    iot.publish_message_to_spreadsheet(turtle_x=result_x,
                                                       turtle_y=result_y,
                                                       turtle_theta=result_theta)
                else:
                    rospy.logerr("MQTT Failed to Publish")
                    result.flag_success = False

            elif goal.mode == "sub":
                rospy.logwarn("MQTT SUB Goal ID: " + str(goal_id.id))
                rospy.logwarn(goal.topic)

                ret = iot.mqtt_subscribe_thread_start(self.mqtt_sub_callback,
                                                      self._config_mqtt_server_url,
                                                      self._config_mqtt_server_port,
                                                      goal.topic,
                                                      self._config_mqtt_qos)
                if ret == 0:
                    rospy.loginfo("MQTT Subscribe Thread Started")
                    result.flag_success = True
                else:
                    rospy.logerr("Failed to start MQTT Subscribe Thread")
                    result.flag_success = False

        rospy.loginfo("Send goal result to client")
        if result.flag_success:
            rospy.loginfo("Succeeded")
            goal_handle.set_succeeded(result)
        else:
            rospy.loginfo("Goal Failed. Aborting.")
            goal_handle.set_aborted(result)

        rospy.loginfo("Goal ID: " + str(goal_id.id) + " Goal Processing Done.")


    def on_cancel(self, goal_handle):
        """
        This function will be called when Goal Cancel request is send to the Action Server
        """
        rospy.loginfo("Received cancel request.")
        goal_id = goal_handle.get_goal_id()
        rospy.logerr("Canceled " + str(goal_id))


    # def mqtt_sub_callback(self, client, userdata, message):
    #     """
    #     This is a callback function for MQTT Subscriptions
    #     """
    #     payload = str(message.payload.decode("utf-8"))

    #     print("[MQTT SUB CB] Message: ", payload)
    #     print("[MQTT SUB CB] Topic: ", message.topic)

    #     msg_mqtt_sub = msgMqttSub()
    #     msg_mqtt_sub.timestamp = rospy.Time.now()
    #     msg_mqtt_sub.topic = message.topic
    #     msg_mqtt_sub.message = payload

    #     self._handle_ros_pub.publish(msg_mqtt_sub)

    def func_incoming_order_callback(self, client, userdata, message):
        """
        TODO
        """
        payload = str(message.payload.decode("utf-8"))
        msg_obj = json.loads(payload)

        # rospy.loginfo('\n\nPAYLOAD: \n\n' + payload + '\n\n')

        if msg_obj["item"] == 'Clothes':
            msg_obj["priority"]='LP'
            msg_obj["cost"]='100'
        elif msg_obj["item"] == 'Food':
            msg_obj["priority"]='MP'
            msg_obj["cost"]='200'
        else:
            msg_obj["priority"]='HP'
            msg_obj["cost"]='300'

        kwargs = {
            "id": "Incoming Orders",
            "Order ID": msg_obj["order_id"],
            "Order Date and Time": msg_obj["order_time"],
            "Item": msg_obj["item"],
            "Order Quantity": msg_obj["qty"],
            "City": msg_obj["city"],
            "Latitude": msg_obj["lat"],
            "Longitude": msg_obj["lon"],
            "Priority": msg_obj["priority"],
            "Unique ID": self._config_mqtt_unique_id,
            "Team ID": "VB#1004",
            "Cost": msg_obj["cost"]
        }
        iot.publish_message_to_spreadsheet(**kwargs)
        self._config_topic_incoming_orders.publish(str(kwargs))

    def func_inventory_data_callback(self, data):
        """
        TODO
        """
        kwargs = eval(data.inventoryData)
        iot.publish_message_to_spreadsheet(**kwargs)

    def func_dispatch_data_callback(self, data):
        """
        TODO
        """
        kwargs = eval(data.dispatchData)
        iot.publish_message_to_spreadsheet(**kwargs)

    def func_shipped_data_callback(self, data):
        """
        TODO
        """
        kwargs = eval(data.shippedData)
        iot.publish_message_to_spreadsheet(**kwargs)
        
def main():
    """
    Main Function
    """
    rospy.init_node('node_ros_iot_bridge_action_server')

    server = IotRosBridgeActionServer()

    rospy.Subscriber('/topic_inventory_data', inventoryMsg, server.func_inventory_data_callback)
    rospy.Subscriber('/topic_dispatch_data', dispatchMsg, server.func_dispatch_data_callback)

    rospy.Subscriber('/topic_shipped_data', shippedMsg, server.func_shipped_data_callback)

    rospy.spin()


if __name__ == '__main__':
    main()
