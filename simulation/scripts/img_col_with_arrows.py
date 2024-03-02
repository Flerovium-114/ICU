#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from math import floor
    
def img_call(data):
    br=CvBridge()
    cur_frame=br.imgmsg_to_cv2(data)
    img_hsv = cv2.cvtColor(cur_frame, cv2.COLOR_BGR2HSV)

    mask = cv2.inRange(img_hsv, (5, 50, 50), (15, 255, 255))

    contours,_ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    if len(contours):
            for c in contours:
                area = cv2.contourArea(c)
                if area > 60:
                    x,y,w,h = cv2.boundingRect(c)
                    x_c=x+w//2
                    y_c=y+h//2
                    cv2.arrowedLine(cur_frame, (320, 240), (int(floor(x_c)), int(floor(y_c))), (255, 0, 0), 3)

    cv2.imshow('color_feed',cur_frame)
    cv2.waitKey(1)

def subs():
    rospy.init_node('col_listener',anonymous=False)
    rospy.Subscriber('/webcam/image_raw',Image,img_call)
    rospy.spin()
    cv2.destroyAllWindows()
    
if __name__=='__main__':
    subs()
