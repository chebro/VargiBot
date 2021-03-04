#!/usr/bin/env python

"""
IoT Module
This module contains methods to handle MQTT + HTTP requests.
"""

import time
import requests

import paho.mqtt.client as mqtt

def mqtt_subscribe_thread_start(arg_callback_func,
                                arg_broker_url,
                                arg_broker_port,
                                arg_mqtt_topic,
                                arg_mqtt_qos):
    """
    MQTT Sub Thread : Subscribes to MQTT

    This method subscribes to any MQTT topic.

    Parameters:

        arg_callback_func(function): Function to execute on receiving a message.
        arg_broker_url(string): MQTT broker URL.
        arg_broker_port(int): Port to connect to the MQTT broker.
        arg_mqtt_topic(string): Topic to listen to on the MQTT port.
        arg_mqtt_qos(int): Quality of Service ranges between 0-2.
    """
    try:
        mqtt_client = mqtt.Client()
        mqtt_client.on_message = arg_callback_func
        mqtt_client.connect(arg_broker_url, arg_broker_port)
        mqtt_client.subscribe(arg_mqtt_topic, arg_mqtt_qos)
        time.sleep(1)
        mqtt_client.loop_start()
        return 0
    except: # pylint: disable=bare-except
        return -1


def mqtt_publish(arg_broker_url,
                 arg_broker_port,
                 arg_mqtt_topic,
                 arg_mqtt_message,
                 arg_mqtt_qos):
    """
    MQTT Sub : Publishes to MQTT

    This method published to any MQTT topic.

    Parameters:
        arg_broker_url(string): MQTT broker URL.
        arg_broker_port(int): Port to connect to the MQTT broker.
        arg_mqtt_topic(string): Topic to listen to on the MQTT port.
        arg_mqtt_message(string): The message string that needs to be published.
        arg_mqtt_qos(int): Quality of Service ranges between 0-2.
    """
    try:
        mqtt_client = mqtt.Client("mqtt_pub")
        mqtt_client.connect(arg_broker_url, arg_broker_port)
        mqtt_client.loop_start()

        print("Publishing message to topic", arg_mqtt_topic)
        mqtt_client.publish(arg_mqtt_topic, arg_mqtt_message, arg_mqtt_qos)
        time.sleep(0.1) # wait
        mqtt_client.loop_stop() #stop the loop
        return 0
    except: # pylint: disable=bare-except
        return -1

def publish_to_spreadsheet(**kwargs):
    """
    HTTP Pub : Publish to Spreadsheet (Variardic Function)
    
    Parameters:
        **kwargs: Varaible number of keyworded arguments containing HTTP request params.
    """

    #student_sheet_id_task5 = "AKfycbxzsgHgL5jaILoWdWIJuqrkUqmM-bkLQINWqemw6j3AW7ibDJJKoUIFmg"
    student_sheet_id = "AKfycbwtkdrTqvlYKNhoEXBXgObs8KUK0hUl11tLaqJQ3jj8rPqyQnBEIh0V"
    instructor_sheet_id = "AKfycbw5xylppoda-8HPjt2Tzq4ShU_Xef-Ik-hEtBPcPk0gdGw8095j4RZ7"

    surl = "https://script.google.com/macros/s/" + student_sheet_id + "/exec"
    iurl = "https://script.google.com/macros/s/" + instructor_sheet_id + "/exec"

    params = {}

    for (key, val) in kwargs.items():
        params[key] = val

    print params

    sresponse = requests.get(surl, params=params)
    iresponse = requests.get(iurl, params=params)

    print sresponse
    print iresponse

    if sresponse.status_code == 200 and iresponse.status_code == 200:
        print 'HTTP Requests Sent.'
        return 0

    return -1
