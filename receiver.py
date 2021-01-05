# MQTT Client for LinkedIn Learning Raspberry Pi Weekly
import paho.mqtt.subscribe as subscribe
import numpy as np
import cv2
import matplotlib.pyplot as plt
from io import BytesIO

broker_location = "192.168.4.1"

def on_mqtt_message(client, userdata, message):
    # print(message.topic + " " +str(message.payload))
    f = BytesIO(message.payload)
    loaded_np = np.load(f, allow_pickle=True)
    cv2.imshow("image", loaded_np)


subscribe.callback(on_mqtt_message, "FirstTry/Test", hostname = broker_location)
