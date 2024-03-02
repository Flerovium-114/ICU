#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from py_gnc import *
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2.aruco as aruco
import time
from term_colors import *

times=0
drone = gnc_api()

aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_250)

def img_callback(data):
    br=CvBridge()
    cur_frame=br.imgmsg_to_cv2(data)
    gray = cv2.cvtColor(cur_frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = aruco.detectMarkers(gray, aruco_dict)
    if (ids!=None) and (times == 0):
        if ids[0]==0:
            print("Times = " + str(times))
            func(cur_frame, corners)
            rospy.loginfo("We reached here")
            # exit
            return
    
 
def func(cur_frame, corners):
    global times
    times += 1
    aruco.drawDetectedMarkers(cur_frame, corners)
    rospy.loginfo("MARKER DETECTED")
    time.sleep(5)
    rospy.loginfo("Sleeping over (5 secs)")
    drone.land()
    rospy.loginfo("Landing ho rha hai")
    return
            

def recv():
    # rospy.init_node('img_listener',anonymous=True)
    rospy.Subscriber('/webcam/image_raw',Image,img_callback)
    rospy.spin()
    cv2.destroyAllWindows()
    
     
def takeoff_drone(): 
    rospy.init_node("drone_controller", anonymous=True)
    drone.wait4connect()
    drone.wait4start()
    drone.initialize_local_frame()
    drone.takeoff(3)
    rate = rospy.Rate(3)
    rospy.loginfo(CGREEN2 + "Takeoff Completed" + CEND)


if __name__ == '__main__':
    # print(k)
    takeoff_drone()
    recv()