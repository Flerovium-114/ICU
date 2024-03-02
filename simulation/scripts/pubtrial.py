#!/usr/bin/env python

# -*- coding: utf-8 -*-

import rospy
from py_gnc import *
from term_colors import *

def func():
    rospy.init_node("drone_controller", anonymous=True)
    drone = gnc_api()
    drone.initialize_local_frame()
    drone.wait4connect()
    drone.wait4start()

    drone.takeoff(5)
    rate = rospy.Rate(3)
    
    while not rospy.is_shutdown():
        rospy.Subscriber(
            name="{}mavros/setpoint_position/local".format(drone.ns),
            data_class=PoseStamped,
            queue_size=10,
            callback=drone.set_cb,
        )

        rate.sleep() 
        # if(drone.check_waypoint_reached()):
        #     # drone.land()
        #     print("sexyush")
        #     break




if __name__ == '__main__':
    try:
        func()
    except KeyboardInterrupt:
        exit()