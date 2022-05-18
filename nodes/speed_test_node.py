#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

rospy.init_node('speed_test')
cmd_pub = rospy.Publisher("/cmd_vel",Twist, queue_size = 1)

# Set linear velocity to 0.2 m/s
default_x = 0.1
default_z = 0.1
new_linear_x = default_x
new_angular_z = default_z
linear_x = 0
angular_z = 0
rate = rospy.Rate(20)
while not rospy.is_shutdown():
    command = input("Enter a command to perform test: ")
    if command == "w":
        linear_x = new_linear_x
        angular_z = 0
    elif command == "s":
        linear_x = -new_linear_x
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

    cmd_pub.publish(vel)

    # Wait for duration seconds
    duration = rospy.Duration(4.0,0)
    rospy.sleep(duration)

    vel.angular.z = 0.0
    vel.linear.x = 0.0
    cmd_pub.publish(vel)

    print("-----------------------------------------")
    new_linear_x = input("Enter new linear x to test: ")
    new_angular_z = input("Enter new angular z to test: ")
    
    print("Gain x: " + str(new_linear_x/default_x))
    print("Gain z: " + str(new_angular_z/default_z))
