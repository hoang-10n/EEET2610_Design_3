import paho.mqtt.publish as publish
import time

broker_location = "192.168.5.1"

while True:
    direction = input("Direction")
    publish.single("Design3/Control", direction, hostname = broker_location)
    time.sleep(1)