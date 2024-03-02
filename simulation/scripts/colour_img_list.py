#!/usr/bin/env python

# -*- coding: utf-8 -*-

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def col_img_call(data):
    br=CvBridge()
    cur_frame=br.imgmsg_to_cv2(data)
    img_hsv = cv2.cvtColor(cur_frame, cv2.COLOR_BGR2HSV)


    mask1 = cv2.inRange(img_hsv, (0,90,50), (5,255,255))
    mask2 = cv2.inRange(img_hsv, (175,90,50), (180,255,255))


    mask = cv2.bitwise_or(mask1, mask2 )



    contours,_ = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

    for c in contours:
        area = cv2.contourArea(c)
        print(area)
        if(area>100):
            x,y,w,h = cv2.boundingRect(c)
            cur_frame = cv2.rectangle(cur_frame,(x,y),(x+w,x+h),(0,0,255),5)
    cv2.imshow('red_detect',cur_frame)
    cv2.waitKey(1)

def col_subs():
    rospy.init_node('col_img_listener',anonymous=True)
    rospy.Subscriber('/webcam/image_raw',Image,col_img_call)
    rospy.spin()
    cv2.destroyAllWindows()
    
    
if __name__=='__main__':
    col_subs()
    

