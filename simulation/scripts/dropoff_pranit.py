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

mp = {}

aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_250)

def aruco_centre(corners):
    x = (corners[0][0][0][0] + corners[0][0][2][0])/2
    y = (corners[0][0][0][1] + corners[0][0][2][1])/2
    return [x,y]

def reqd_velo(corners,param):
    lst=aruco_centre(corners)
    v_x=lst[0]-frame_centre[0]
    v_y=lst[1]-frame_centre[1]
    p=(v_x**2+v_y**2)**(0.5)
    v_x,v_y=v_x/p,v_y/p
    return param*v_x, param*v_y

def thres(corners):
    lst1=aruco_centre(corners)
    
    mag=((lst1[0]-frame_centre[0])**2+(lst1[1]-frame_centre[1])**2)**(0.5)
    if(mag>20):
        return mag
    return 0

def get_aruco_info(corners,ids):
    no=0
    pairs=[]
    for i in range(len(corners[0])):
        no+=1
        pairs.append([corners[0][i],ids[i][0]])
    return pairs,no

def get_dict(a):
    corners,ids,_=a
    dict={}
    for i in range(len(ids)):
        dict[ids[i][0]]=corners[i][0]
    return dict

def img_callback(data):
    br=CvBridge()
    cur_frame=br.imgmsg_to_cv2(data)
    gray = cv2.cvtColor(cur_frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = aruco.detectMarkers(gray, aruco_dict)

    ################################################################################################ 

    while len(ids) and len(corners) and times == 0: ##### marker ka id is 0
        gray = cv2.cvtColor(cur_frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, aruco_dict)
        aruco.drawDetectedMarkers(cur_frame, corners)
        velocity_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel",TwistStamped,queue_size=10)
        vel = TwistStamped()
        curpos_z = drone.current_pose_g.pose.pose.position.z

        global mp
        mp.clear()
        for i in range(len(ids)):
            mp[ids[i][0]]=[[corners[i][0]]]
        rospy.loginfo(mp)


        if(abs(curpos_z - 1) < 0.2) or (curpos_z < 0.5):
            end_phase1()
            break
        rospy.loginfo(thres(mp[0]))
        if not (thres(mp[0])):
            rospy.loginfo(thres(mp[0]))
            vel.twist.linear.x = 0
            vel.twist.linear.y = 0
            vel.twist.linear.z = -0.2
            
            velocity_pub.publish(vel)
            break
        aruco.drawDetectedMarkers(cur_frame, corners)
        
        
        v_x,v_y = reqd_velo(mp[0],0.1)

        vel.twist.linear.x = -v_y
        vel.twist.linear.y = -v_x
    
        velocity_pub.publish(vel)


    while len(ids) and len(corners)  and times == 1: ##### marker ka id is 4
        gray = cv2.cvtColor(cur_frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, aruco_dict)
        aruco.drawDetectedMarkers(cur_frame, corners)
        velocity_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel",TwistStamped,queue_size=10)
        vel = TwistStamped()
        curpos_z = drone.current_pose_g.pose.pose.position.z

        global mp
        mp.clear()
        for i in range(len(ids)):
            mp[ids[i][0]]=[[corners[i][0]]]
        rospy.loginfo(mp)


        if(abs(curpos_z - 1) < 0.2) or (curpos_z < 0.5):
            end_phase1()
            break
        rospy.loginfo(thres(mp[4]))
        if not (thres(mp[4])):
            rospy.loginfo(thres(mp[4]))
            vel.twist.linear.x = 0
            vel.twist.linear.y = 0
            vel.twist.linear.z = -0.2
            
            velocity_pub.publish(vel)
            break
        aruco.drawDetectedMarkers(cur_frame, corners)
        
        
        v_x,v_y = reqd_velo(mp[4],0.1)

        vel.twist.linear.x = -v_y
        vel.twist.linear.y = -v_x
    
        velocity_pub.publish(vel)        

 
def end_phase1 ():
    rospy.loginfo("reached here")
    drone.land()
    rospy.loginfo("sleeping starts")
    time.sleep(20)
    rospy.loginfo("sleeping end")
    takeoff_again(3)
    global times
    times += 1
    return

def recv():
    rospy.Subscriber('/webcam/image_raw',Image,img_callback)
    rospy.spin()
    cv2.destroyAllWindows()

def takeoff_again(height):
    drone.set_mode("GUIDED")
    drone.takeoff(height)
    rate = rospy.Rate(3)
    rospy.loginfo(CGREEN2 + "Takeoff Completed" + CEND)
     
def takeoff_drone(): 
    drone.wait4connect()
    drone.wait4start()
    drone.initialize_local_frame()
    drone.takeoff(3)
    rate = rospy.Rate(3)
    rospy.loginfo(CGREEN2 + "Takeoff Completed" + CEND)


if __name__ == '__main__':
    # print(k)
    rospy.init_node("drone_controller", anonymous=True)
    takeoff_drone()
    recv()

