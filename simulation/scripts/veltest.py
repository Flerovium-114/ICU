#!/usr/bin/env python

# -*- coding: utf-8 -*-

import rospy
# Import the API.
from py_gnc import *
from geometry_msgs.msg import TwistStamped
from term_colors import *
import time


def takeoff_drone(): 
    # Initializing ROS node.
    rospy.init_node("drone_controller", anonymous=True)

    drone = gnc_api()
    drone.wait4connect()
    drone.wait4start()

    drone.initialize_local_frame()
    drone.takeoff(2)
    rate = rospy.Rate(2)
    rospy.loginfo(CGREEN2 + "Takeoff Completed" + CEND)
    rospy.sleep(3)

    rospy.loginfo("pubing vel")
    velocity_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=10)
    vel = TwistStamped()
    t_end = time.time() + 2

    while time.time() < t_end:
        vel.twist.linear.z=-0.2
        velocity_pub.publish(vel)
    rospy.loginfo("sleeping")
    
    vel.twist.linear.x=0
    velocity_pub.publish(vel)
    rospy.loginfo("done")

    





if __name__ == '__main__':
    try:
        takeoff_drone()
    except KeyboardInterrupt:
        exit()
