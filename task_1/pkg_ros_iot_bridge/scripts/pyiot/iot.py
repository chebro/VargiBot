"""
IoT Module - MQTT + HTTP Requests
"""

# from multiprocessing.dummy import Pool
import time
import requests

# import sys
import paho.mqtt.client as mqtt #import the client1

def iot_func_callback_sub(client, userdata, message):
    """
    -----------------  MQTT SUB CALLBACK -------------------
    """
    print("message received ", str(message.payload.decode("utf-8")))
    print("message topic=", message.topic)
    print("message qos=", message.qos)
    print("message retain flag=", message.retain)

def mqtt_subscribe_thread_start(arg_callback_func,
                                arg_broker_url,
                                arg_broker_port,
                                arg_mqtt_topic, arg_mqtt_qos):
    """
    -----------------  MQTT SUB -------------------
    """
    try:
        mqtt_client = mqtt.Client()
        mqtt_client.on_message = arg_callback_func
        mqtt_client.connect(arg_broker_url, arg_broker_port)
        mqtt_client.subscribe(arg_mqtt_topic, arg_mqtt_qos)
        time.sleep(1) # wait
        # mqtt_client.loop_forever() # starts a blocking infinite loop
        mqtt_client.loop_start()    # starts a new thread
        return 0
    except:
        return -1


def mqtt_publish(arg_broker_url, arg_broker_port, arg_mqtt_topic, arg_mqtt_message, arg_mqtt_qos):
    """
    -----------------  MQTT PUB -------------------
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
    except:
        return -1

def publish_message_to_spreadsheet(**kwargs):
    """
    -----------------  SPREADSHEET PUB -------------------
    """
    url1 = "https://script.google.com/macros/s/AKfycbwQwrleLtrkgPW5I-lIMd8iMPDtO87bm0LpNpx5_Hb-QT9Gsvc/exec"
    url2 = "https://script.google.com/macros/s/AKfycbw850dk4moVgebU2GGe0PUQUvvg8jTpSjBQCawJt3_13vgujLk/exec"
    
    params1 = {"id": "Sheet1"}
    params2 = {"id": "task1", "team_id": "VB_1004", "unique_id": "CeAhsAGA"}
    for (key, val) in kwargs.items():
        params1[key] = val
        params2[key] = val
    print params1
    print params2
    response1 = requests.get(url1, params=params1)
    response2 = requests.get(url2, params=params2)
    print 'HTTP Requests Sent.'
    print response1
    print response2
    return
