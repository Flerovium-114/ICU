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
    rate = rospy.Rate(2)
    rospy.loginfo(CGREEN2 + "Takeoff Completed" + CEND)
    rospy.sleep(5)
    drone.land()



if __name__ == '__main__':
    try:
        takeoff_drone()
    except KeyboardInterrupt:
        exit()