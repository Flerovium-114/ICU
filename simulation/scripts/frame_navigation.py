#!/usr/bin/env python

# -*- coding: utf-8 -*-

import rospy
import time
from py_gnc import *
import cv2
import cv2.aruco as aruco
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from term_colors import *
from geometry_msgs.msg import TwistStamped

times=0
drone = gnc_api()
frame_centre = [320.,240.]
pixel_threshold = 10

aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_250)

def aruco_centre(corners):
    x = (corners[0][0][0][0] + corners[0][0][2][0])/2
    y = (corners[0][0][0][1] + corners[0][0][2][1])/2
    return [x,y]
    
def reqd_velo(corners,k):
    lst=aruco_centre(corners)
    # 
    
    v_x=lst[0]-frame_centre[0]
    v_y=lst[1]-frame_centre[1]
    p=(v_x**2+v_y**2)**(0.5)
    v_x,v_y=v_x/p,v_y/p
    return k*v_x, k*v_y

def thres(corners):
    lst1=aruco_centre(corners)
    
    mag=((lst1[0]-frame_centre[0])**2+(lst1[1]-frame_centre[1])**2)**(0.5)
    if(mag>20):
        return mag
    return 0
    
def img_callback(data):
    br=CvBridge()
    cur_frame=br.imgmsg_to_cv2(data)
    gray = cv2.cvtColor(cur_frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = aruco.detectMarkers(gray, aruco_dict)
    while ids!=None and corners!=None:
        gray = cv2.cvtColor(cur_frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, aruco_dict)
        rospy.loginfo(corners)
        velocity_pub = rospy.Publisher(
            name="/mavros/setpoint_velocity/cmd_vel",
            data_class=TwistStamped,
            queue_size=10,
        )
        vel = TwistStamped()
        if corners==[]:
            break
        if not (thres(corners)):
            vel.twist.linear.x = 0
            vel.twist.linear.y = 0
            velocity_pub.publish(vel)
            rospy.loginfo("Velocity in x is "+str(vel.twist.linear.x)+ " and y is "+str(vel.twist.linear.x))
            break
        rospy.loginfo("Centering code starts")
        aruco.drawDetectedMarkers(cur_frame, corners)
        # rospy.loginfo(corners)
        v_x,v_y=reqd_velo(corners,0.1)


        vel.twist.linear.x = -v_y
        vel.twist.linear.y = -v_x
        rospy.loginfo("Velocity in x is "+str(vel.twist.linear.x)+ " and y is "+str(vel.twist.linear.x))
        velocity_pub.publish(vel)

        rospy.loginfo("Landing code starts")
        # func(cur_frame, corners)
        rospy.loginfo("We reached here")
        # exit
    else :
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

