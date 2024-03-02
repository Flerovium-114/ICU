#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2.aruco as aruco
from math import floor

aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_250)
def aruco_centre(corners):
    x = (corners[0][0][0][0] + corners[0][0][2][0])/2
    y = (corners[0][0][0][1] + corners[0][0][2][1])/2
    return [x,y]
    
def img_call(data):
    br=CvBridge()
    cur_frame=br.imgmsg_to_cv2(data)
    gray = cv2.cvtColor(cur_frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = aruco.detectMarkers(gray, aruco_dict)
    img_hsv = cv2.cvtColor(cur_frame, cv2.COLOR_BGR2HSV)

    mask = cv2.inRange(img_hsv, (5, 50, 50), (15, 255, 255))

    contours,_ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    if corners:
        # aruco.drawDetectedMarkers(cur_frame, corners)
        cntr=aruco_centre(corners)
        cv2.arrowedLine(cur_frame, (320, 240), (int(floor(cntr[0])), int(floor(cntr[1]))), (0, 255, 0), 3)

    if len(contours):
            for c in contours:
                area = cv2.contourArea(c)
                if area > 60:
                    x,y,w,h = cv2.boundingRect(c)
                    x_c=x+w//2
                    y_c=y+h//2
                    cv2.arrowedLine(cur_frame, (320, 240), (int(floor(x_c)), int(floor(y_c))), (255, 0, 0), 3)
    

    cv2.imshow('image feed',cur_frame)
    cv2.waitKey(1)

def subs():
    rospy.init_node('img_listener',anonymous=False)
    rospy.Subscriber('/webcam/image_raw',Image,img_call)
    rospy.spin()
    cv2.destroyAllWindows()
    
if __name__=='__main__':
    subs()
