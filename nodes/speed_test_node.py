#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

import time

x, y = 0, 0
x_vel, w_vel = 0, 0
start_run = True

def odom_callback(data):
    global x, y, x_vel, w_vel
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    x_vel = data.twist.twist.linear.x
    w_vel = data.twist.twist.angular.z
    if not start_run:
        rospy.loginfo("Current velocity %f, %f", x_vel, w_vel)

if __name__ == "__main__":
    rospy.init_node('speed_test')
    cmd_pub = rospy.Publisher("/cmd_vel",Twist, queue_size = 1)
    odom_sub = rospy.Subscriber("/odom/wheel", Odometry, odom_callback, queue_size=1)

    # Set linear velocity to 0.2 m/s
    default_x = 0.1
    default_z = 0.1
    new_linear_x = default_x
    new_angular_z = default_z
    linear_x = 0
    angular_z = 0
    rate = rospy.Rate(20)

    time.sleep(2.0)

    while not rospy.is_shutdown():
        print("-----------------------------------------")
        new_linear_x = input("Enter new linear x to test: ")
        print(new_linear_x)
        new_angular_z = input("Enter new angular z to test: ")
        print(new_angular_z)

        command = raw_input("Enter a command to perform test: ")
        print(command)
        if command == "w":
            linear_x = new_linear_x
            angular_z = 0
        elif command == "s":
            linear_x =  -new_linear_x
            angular_z = 0.0
        elif command == "a":
            linear_x = new_angular_z
            angular_z = 0.78539816339
        elif command == "d":
            linear_x = new_angular_z
            angular_z = -0.78539816339
        
        vel = Twist()
        vel.linear.x = linear_x # m/s
        vel.linear.y = 0.0
        vel.linear.z = 0.0

        vel.angular.x = 0.0 # rad /s
        vel.angular.y = 0.0
        vel.angular.z = angular_z #0.78539816339

        print(vel)
        
        start_time = time.time()
        start_run = False
        while not rospy.is_shutdown():
            if(time.time() - start_time < 2.0 and x < abs(linear_x)):
                cmd_pub.publish(vel)
                time.sleep(0.05)
            else:
                break

        start_run = True

        vel.angular.z = 0.0
        vel.linear.x = 0.0
        # cmd_pub.publish(vel)

        time.sleep(1.0)
        rospy.loginfo("Current position: %f, %f", x, y)
        
        print("Gain x: " + str(new_linear_x/default_x))
        print("Gain z: " + str(new_angular_z/default_z))
