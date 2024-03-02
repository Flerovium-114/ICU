#!/usr/bin/env python

# -*- coding: utf-8 -*-

import rospy
# Import the API.
from py_gnc import *

from term_colors import *


def takeoff_drone(): 
    # Initializing ROS node.
    rospy.init_node("drone_controller", anonymous=True)

    drone = gnc_api()
    drone.wait4connect()
    drone.wait4start()

    drone.initialize_local_frame()
    drone.takeoff(2)
    # while(drone.current_pose_g.pose.pose.position.z <= 1.9):
    #     msg = PoseStamped()
    #     msg.pose.position.z=2
    #     drone.local_pos_pub.publish(msg)

    rospy.loginfo(CGREEN2 + "Takeoff Completed" + CEND)

    while(drone.current_pose_g.pose.pose.position.z >=0.6):
        msg = PoseStamped()
        msg.pose.position.z = 0.5
        drone.local_pos_pub.publish(msg)

    rospy.loginfo("Hogya")

if __name__ == '__main__':
    try:
        takeoff_drone()
    except KeyboardInterrupt:
        exit()
