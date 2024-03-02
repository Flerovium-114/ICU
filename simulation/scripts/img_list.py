#!/usr/bin/env python

# -*- coding: utf-8 -*-

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2.aruco as aruco

aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_250)
def img_call(data):
    br=CvBridge()
    cur_frame=br.imgmsg_to_cv2(data)
    gray = cv2.cvtColor(cur_frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = aruco.detectMarkers(gray, aruco_dict)
    if corners:
        # Draw a bounding box around each ArUco marker
        aruco.drawDetectedMarkers(cur_frame, corners)
        
        # Print the number of ArUco markers detected
        rospy.loginfo("Number of ArUco markers detected: "+  str(len(corners)))

        # Print the IDs and locations of the ArUco markers
        for i in range(len(corners)):
            print("ID: ", ids[i], " Location: ", corners[i])
    else:
        rospy.loginfo("No marker was detected")
    
    cv2.imshow('image feed',cur_frame)
    cv2.waitKey(1)

def subs():
    rospy.init_node('img_listener',anonymous=True)
    rospy.Subscriber('/webcam/image_raw',Image,img_call)
    rospy.spin()
    cv2.destroyAllWindows()
    
    
if __name__=='__main__':
    subs()
    

