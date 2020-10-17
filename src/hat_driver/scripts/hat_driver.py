#!/usr/bin/env python3

import sys, os, time, threading

import pigpio

import rospy
from std_msgs.msg import ColorRGBA, Bool, Byte


class LEDs():
    def __init__(self, pi):
        self.gpioR = 19
        self.gpioG = 16
        self.gpioB = 13
        self.pi = pi
        pi.set_mode(self.gpioR, pigpio.OUTPUT)
        pi.set_mode(self.gpioG, pigpio.OUTPUT)
        pi.set_mode(self.gpioB, pigpio.OUTPUT)

        self.set(0, 0, 0)

    def set(self, r, g, b):
        # between 0 and 255
        pi.set_PWM_dutycycle(self.gpioR, r)
        pi.set_PWM_dutycycle(self.gpioG, g)
        pi.set_PWM_dutycycle(self.gpioB, b)

class Selecter():
    def __init__(self, pi):
        self.gpio = 6
        pi.set_mode(self.gpio, pigpio.INPUT)

    def get(self):
        return (pi.read(self.gpio) == 0)

class Tirette():
    def __init__(self, pi):
        self.gpio = 26
        pi.set_mode(self.gpio, pigpio.INPUT)

    def get(self):
        return (pi.read(self.gpio) == 0)


class Bumpers():
    def __init__(self, pi):
        self.gpios = [
            7,
            20,
            21,
            25,
        ]

    def get(self):
        value = 0
        for index, item in enumerate(self.gpios):
            if pi.read(item):
                value += 1 << index
        return value

########################################################

pi = pigpio.pi()
leds = LEDs(pi)
selecter = Selecter(pi)
tirette = Tirette(pi)
bumpers = Bumpers(pi)

########################################################


def LEDsCallback(data):
    leds.set(data.r, data.g, data.b)


selecterPub = rospy.Publisher('/hat/selecter', Bool, queue_size=2)
tirettePub = rospy.Publisher('/hat/tirette', Bool, queue_size=2)
bumpersPub = rospy.Publisher('/hat/bumpers', Byte, queue_size=2)

rospy.init_node('hat_driver', anonymous=False)

rospy.Subscriber('/hat/rgb', ColorRGBA, LEDsCallback)

threading.Thread(target=rospy.spin).start()

rate = rospy.Rate(10)
while not rospy.is_shutdown():
    selecterPub.publish(Bool(selecter.get()))
    tirettePub.publish(Bool(tirette.get()))
    bumpersPub.publish(Byte(bumpers.get()))
    rate.sleep()

pi.stop()
