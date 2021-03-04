#!/usr/bin/env python

"""
ROS/IoT Bridge

This program intends act as a bridge bwtween ROS and IoT.
MQTT/HTTP requests are received/sent here.
"""

import threading
import json
import rospy
import actionlib

from pkg_task5.msg import incomingMsg
from pkg_ros_iot_bridge.msg import msgRosIotAction
from pkg_ros_iot_bridge.msg import msgRosIotResult

from pyiot import iot


class IotRosBridge(object):
    """
    ROS/Iot Bridge Class : Performs MQTT Sub + Spreadsheet Pub
    """
    # pylint: disable=too-many-instance-attributes
    def __init__(self):
        self._as = actionlib.ActionServer('/action_ros_iot',
                                          msgRosIotAction,
                                          self.on_goal,
                                          on_cancel,
                                          auto_start=False)

        param_config_iot = rospy.get_param('config_iot')
        self._config_mqtt_unique_id = param_config_iot['mqtt_unique_id']
        self._config_mqtt_server_url = param_config_iot['mqtt']['server_url']
        self._config_mqtt_server_port = param_config_iot['mqtt']['server_port']
        self._config_mqtt_qos = param_config_iot['mqtt']['qos']
        self._config_mqtt_incoming_orders = param_config_iot['mqtt']['incoming_orders']

        self._config_mqtt_sub_cb_ros_topic = param_config_iot['mqtt']['sub_cb_ros_topic']

        self._config_http_inventory_pub = param_config_iot['http']['inventory']
        self._config_http_dispatch_pub = param_config_iot['http']['dispatch']
        self._config_http_shipped_pub = param_config_iot['http']['shipped']

        self._handle_incoming_orders = rospy.Publisher(self._config_mqtt_sub_cb_ros_topic,
                                                       incomingMsg,
                                                       queue_size=10)

        try:
            iot.mqtt_subscribe_thread_start(self.func_incoming_order_callback,
                                            self._config_mqtt_server_url,
                                            self._config_mqtt_server_port,
                                            self._config_mqtt_incoming_orders,
                                            self._config_mqtt_qos)

            rospy.loginfo("MQTT Subscribe Thread Started")
        except: # pylint: disable=bare-except
            rospy.logerr("Failed to start MQTT Subscribe Thread")

        self._as.start()
        rospy.loginfo("Started ROS-IoT Bridge Action Server")

    def on_goal(self, goal_handle):
        """
        Callback Function

        This function will be called when Action Server receives a Goal.

        Parameters: 
            goal_handle(object): Goal handler for the incoming goal.
        """
        goal = goal_handle.get_goal()

        rospy.loginfo("Received new goal from Client")
        rospy.loginfo(goal)

        # Validate incoming goal parameters
        if goal.protocol == "http":

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
        Goal Processing Function

        This function is called is a separate thread to process Goal

        Parameters: 
            goal_handle(object): Goal handler for the incoming goal.
        """
        result = msgRosIotResult()

        goal_id = goal_handle.get_goal_id()
        rospy.loginfo("Processing goal : " + str(goal_id.id))

        goal = goal_handle.get_goal()

        if (goal.protocol == "http") and (goal.mode == "pub"):

            rospy.logwarn("HTTP PUB Goal ID: " + str(goal_id.id))
            rospy.logwarn(goal.topic + " > " + goal.message)

            if goal.topic == self._config_http_inventory_pub:
                http_status = func_inventory_data_callback(goal.message)

            if goal.topic == self._config_http_dispatch_pub:
                http_status = func_dispatch_data_callback(goal.message)

            if goal.topic == self._config_http_shipped_pub:
                http_status = func_shipped_data_callback(goal.message)

        if http_status == 0:
            rospy.loginfo("HTTP requests were successfully sent!")
            result.flag_success = True
        else:
            rospy.loginfo("HTTP requests failed")
            result.flag_success = False

        rospy.loginfo("Send goal result to client")

        if result.flag_success:
            rospy.loginfo("Succeeded")
            goal_handle.set_succeeded(result)
        else:
            rospy.loginfo("Goal Failed. Aborting.")
            goal_handle.set_aborted(result)

        rospy.loginfo("Goal ID: " + str(goal_id.id) + " Goal Processing Done.")

    def func_incoming_order_callback(self, client, userdata, message):
        """
        Callback Function

        This method is triggered when there is an incoming order.

        Parameters:
            client(string): Information regarding the MQTT client.
            userdata(string): Information regarding the userdata (if any).
            message(string): Contains the MQTT message.
        """
        rospy.loginfo(client)
        rospy.loginfo(userdata)
        payload = str(message.payload.decode("utf-8"))
        msg_obj = json.loads(payload)

        if msg_obj["item"] == 'Clothes':
            msg_obj["priority"] = 'LP'
            msg_obj["cost"] = '100'
        elif msg_obj["item"] == 'Food':
            msg_obj["priority"] = 'MP'
            msg_obj["cost"] = '200'
        else:
            msg_obj["priority"] = 'HP'
            msg_obj["cost"] = '300'

        kwargs = {
            "id": "IncomingOrders",
            "Order ID": msg_obj["order_id"],
            "Order Date and Time": msg_obj["order_time"],
            "Item": msg_obj["item"],
            "Order Quantity": msg_obj["qty"],
            "City": msg_obj["city"],
            "Latitude": msg_obj["lat"],
            "Longitude": msg_obj["lon"],
            "Priority": msg_obj["priority"],
            "Unique Id": self._config_mqtt_unique_id,
            "Team Id": "VB#1004",
            "Cost": msg_obj["cost"]
        }

        iot.publish_to_spreadsheet(**kwargs)

        self._handle_incoming_orders.publish(str(kwargs))

        kwargs['id'] = 'Dashboard'
        kwargs['Quantity'] = msg_obj["qty"]
        kwargs['Order Time'] = msg_obj["order_time"]

        iot.publish_to_spreadsheet(**kwargs)

def on_cancel(goal_handle):
    """
    Goal Canceling Function

    This function will be called when Goal Cancel request is send to the Action Server.

    Parameters:
        goal_handle(object): Goal handler for the incoming goal.
    """
    rospy.loginfo("Received cancel request.")
    goal_id = goal_handle.get_goal_id()
    rospy.logerr("Canceled " + str(goal_id))

def func_inventory_data_callback(data):
    """
    Callback Function

    This method is triggered when there is a detected package on the shelf.

    Parameters:
        data(string): Relevant information regarding the detected package.
    """
    kwargs = eval(data)

    rospy.logwarn('\n\nINVENTORY UPDATE RECEIVED!\n\n')
    rospy.logwarn(kwargs)

    resp = iot.publish_to_spreadsheet(**kwargs)
    return resp

def func_dispatch_data_callback(data):
    """
    Callback Function

    This method is triggered when a package is dispatched.

    Parameters:
        data(string): Relevant information regarding the dispatched package.
    """
    kwargs = eval(data)

    rospy.logwarn('\n\nDISPATCH UPDATE RECEIVED!\n\n')
    rospy.logwarn(kwargs)

    resp1 = iot.publish_to_spreadsheet(**kwargs)

    kwargs['id'] = 'Dashboard'
    kwargs['Order Dispatched'] = 'YES'
    kwargs['Dispatch Time'] = kwargs["Dispatch Date and Time"]

    resp2 = iot.publish_to_spreadsheet(**kwargs)

    return resp1 + resp2

def func_shipped_data_callback(data):
    """
    Callback Function

    This method is triggered when a package is shipped.

    Parameters:
        data(string): Relevant information regarding the shipped package.
    """
    kwargs = eval(data)

    rospy.logwarn('\n\nSHIPPING UPDATE RECEIVED!\n\n')
    rospy.logwarn(kwargs)

    resp1 = iot.publish_to_spreadsheet(**kwargs)

    kwargs['id'] = 'Dashboard'
    kwargs['Order Shipped'] = 'YES'
    kwargs['Shipping Time'] = kwargs["Shipped Date and Time"]

    resp2 = iot.publish_to_spreadsheet(**kwargs)

    return resp1 + resp2

def main():
    """
    Main Function
    """
    rospy.init_node('node_ros_iot_bridge_action_server')

    IotRosBridge()

    rospy.spin()

if __name__ == '__main__':
    main()
