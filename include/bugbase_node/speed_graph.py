#!/usr/bin/python3
# -*- coding: utf-8 -*-

import datetime as dt
import time
import plotext as plt
import rospy
from bugbase_node.msg import BugbaseMotorArray, BugbaseMotor


class SpeedVisualizer:
    def __init__(self):   
        
        sub = rospy.Subscriber("motor_state", BugbaseMotorArray, self.callback, queue_size=1)

        self.xs, self.ys = [], []
        # Create figure for plotting
        plt.date_form("Y/m/d H:M:S")
        # plt.clc()
        plt.title("test streaming")
        plt.xlim(lower=0, upper=100)
        plt.xlabel("X")
        plt.ylabel("Y")
            
    def callback(self, data):
        
        self.xs.append(float(data.motor[0].est_velocity))
        self.ys.append(float(data.motor[0].acceleration))

        # Limit x and y lists to 20 items
        if(len(self.xs) > 100):
            self.xs = self.xs[-100:]
            self.ys = self.ys[-100:]
        # Draw x and y lists
        plt.clt()
        plt.cld()

        plt.ylim(min(min(self.xs), min(self.ys)) - 10, max(max(self.xs), max(self.ys)) + 10)
        plt.plot(self.ys, label="first")
        plt.plot(self.xs, label="second")
        # rospy.loginfo(str(len(self.xs)) + " " + str(len(self.ys)))
        plt.sleep(0.001)
        plt.show()

    def update_data(self, data):
        self.data = data
        # self.xs.append(plt.datetime_to_string(plt.today_datetime(), "Y/m/d H:M:S"))
        self.xs.append(float(-data))
        self.ys.append(float(data))

        # Limit x and y lists to 20 items
        if(len(self.xs) > 100):
            self.xs = self.xs[-100:]
            self.ys = self.ys[-100:]

        # Draw x and y lists
        plt.clt()
        plt.cld()

        plt.ylim(self.xs[-1]-10, self.ys[-1]+10)
        plt.plot(self.ys, label="first")
        plt.plot(self.xs, label="second")
        plt.show()

if __name__ == "__main__":
    
    rospy.init_node("test_phone_stream")

    visualizer = SpeedVisualizer()

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        rate.sleep()
    # plt.show()

        