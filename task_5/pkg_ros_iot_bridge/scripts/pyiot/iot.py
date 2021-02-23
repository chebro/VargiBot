"""
IoT Module - MQTT + HTTP Requests
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
    MQTT Sub : Subscribe to MQTT
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
    MQTT Pub : Publish to MQTT
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
    HTTP Pub : Publish to Spreadsheet
    """
    student_sheet_id = "AKfycbxzsgHgL5jaILoWdWIJuqrkUqmM-bkLQINWqemw6j3AW7ibDJJKoUIFmg"
    url = "https://script.google.com/macros/s/" + student_sheet_id + "/exec"

    params = {}
    for (key, val) in kwargs.items():
        params[key] = val

    print params

    response = requests.get(url, params=params)
    print 'HTTP Requests Sent.'
    print response

    return
