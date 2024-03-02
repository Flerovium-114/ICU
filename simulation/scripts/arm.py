#!/usr/bin/env python

# -*- coding: utf-8 -*-

import rospy
# Import the API.
from py_gnc import *

from term_colors import *


def main(): 
    rospy.init_node("drone_controller", anonymous=True)
    drone = gnc_api()
    drone.wait4connect()
    drone.wait4start()
    drone.initialize_local_frame()
    drone.arm()

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        exit()