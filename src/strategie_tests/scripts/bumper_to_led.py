#!/usr/bin/env python3

import sys, os, time, threading

import rospy
from std_msgs.msg import ColorRGBA, Bool, Byte


########################################################

rgbPub = rospy.Publisher('/hat/rgb', ColorRGBA, queue_size=2)

rospy.init_node('bumper_to_led', anonymous=False)


def onBumper(data):
    rgb = ColorRGBA()
    byte = int(data.data)
    if byte & 1: rgb.r = 255
    if byte & 2: rgb.g = 255
    if byte & 4: rgb.b = 255
    if byte & 8: rgb.r = 255 ; rgb.g = 255 ; rgb.b = 255

    if rgb.r + rgb.g + rgb.b > 0:
        rgbPub.publish(rgb)

rospy.Subscriber('/hat/bumpers', Byte, onBumper)

threading.Thread(target=rospy.spin).start()
