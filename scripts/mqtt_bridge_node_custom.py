#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
import paho.mqtt.client as mqtt


broker_address = "192.168.1.3"
broker_port = 1883

def mqtt_on_connect(client, userdata, flags, rc):
    if rc == 0:
        rospy.loginfo("Connected to MQTT broker")
    else:
        rospy.logerr("Failed to connect to MQTT broker")

def mqtt_on_publish(client, userdata, mid):
    rospy.loginfo("Published data to MQTT broker")

def ros_topic_callback(msg):
    # Get the voltage value from the ROS message
    voltage = msg.data

    # Publish the voltage value to the MQTT broker
    mqtt_client.publish("/voltage", voltage)

if __name__ == '__main__':
    rospy.init_node('mqtt_bridge_node')

    # MQTT Broker Connection Setup
    mqtt_client = mqtt.Client()
    mqtt_client.on_connect = mqtt_on_connect
    mqtt_client.on_publish = mqtt_on_publish
    mqtt_client.connect(broker_address, broker_port, 60)  # Replace with your MQTT broker details
    mqtt_client.loop_start()

    # ROS Subscriber Setup
    rospy.Subscriber("/voltage", Float32, ros_topic_callback)

    rospy.spin()

    # Cleanup
    mqtt_client.loop_stop()
    mqtt_client.disconnect()

