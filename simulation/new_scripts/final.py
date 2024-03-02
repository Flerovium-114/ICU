#!/usr/bin/env python
# -*- coding: utf-8 -*-

from term_colors import *
import rospy
import cv2
import cv2.aruco as aruco
from cv_bridge import CvBridge
from math import atan2, pow, degrees, radians, sin, cos
from geometry_msgs.msg import PoseStamped, Point, TwistStamped
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandTOL, CommandTOLRequest, CommandLong, CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from sensor_msgs.msg import Image

class drone_api:
    def __init__(self):
 
        self.current_state_g = State()
        self.current_pose_g = Odometry()
        self.local_offset_pose_g = Point()
        self.waypoint_g = PoseStamped()

        self.current_heading_g = 0.0
        self.local_offset_g = 0.0
        self.correction_heading_g = 0.0

        self.ns = rospy.get_namespace()
        if self.ns == "/":
            rospy.loginfo(CBLUE2 + "Using default namespace" + CEND)
        else:
            rospy.loginfo(CBLUE2 + "Using {} namespace".format(self.ns) + CEND)

        self.local_pos_pub = rospy.Publisher(
            name="{}mavros/setpoint_position/local".format(self.ns),
            data_class=PoseStamped,
            queue_size=10,
        )

        self.currentPos = rospy.Subscriber(
            name="{}mavros/global_position/local".format(self.ns),
            data_class=Odometry,
            queue_size=10,
            callback=self.pose_cb,
        )

        self.state_sub = rospy.Subscriber(
            name="{}mavros/state".format(self.ns),
            data_class=State,
            queue_size=10,
            callback=self.state_cb,
        )

        rospy.wait_for_service("{}mavros/cmd/arming".format(self.ns))

        self.arming_client = rospy.ServiceProxy(
            name="{}mavros/cmd/arming".format(self.ns), service_class=CommandBool
        )

        rospy.wait_for_service("{}mavros/cmd/land".format(self.ns))

        self.land_client = rospy.ServiceProxy(
            name="{}mavros/cmd/land".format(self.ns), service_class=CommandTOL
        )

        rospy.wait_for_service("{}mavros/cmd/takeoff".format(self.ns))

        self.takeoff_client = rospy.ServiceProxy(
            name="{}mavros/cmd/takeoff".format(self.ns), service_class=CommandTOL
        )

        rospy.wait_for_service("{}mavros/set_mode".format(self.ns))

        self.set_mode_client = rospy.ServiceProxy(
            name="{}mavros/set_mode".format(self.ns), service_class=SetMode
        )

        rospy.wait_for_service("{}mavros/cmd/command".format(self.ns))

        self.command_client = rospy.ServiceProxy(
            name="{}mavros/cmd/command".format(self.ns), service_class=CommandLong
        )
        rospy.loginfo(CBOLD + CGREEN2 + "Initialization Complete." + CEND)

    def state_cb(self, message):
        self.current_state_g = message

    def pose_cb(self, msg):
        self.current_pose_g = msg
        self.enu_2_local()

        q0, q1, q2, q3 = (
            self.current_pose_g.pose.pose.orientation.w,
            self.current_pose_g.pose.pose.orientation.x,
            self.current_pose_g.pose.pose.orientation.y,
            self.current_pose_g.pose.pose.orientation.z,
        )
        psi = atan2((2 * (q0 * q3 + q1 * q2)),(1 - 2 * (pow(q2, 2) + pow(q3, 2))))
        self.current_heading_g = degrees(psi) - self.local_offset_g

    def enu_2_local(self):
        x, y, z = (
            self.current_pose_g.pose.pose.position.x,
            self.current_pose_g.pose.pose.position.y,
            self.current_pose_g.pose.pose.position.z,
        )

        current_pos_local = Point()

        current_pos_local.x = x * cos(radians((self.local_offset_g - 90))) - y * sin(
            radians((self.local_offset_g - 90))
        )

        current_pos_local.y = x * sin(radians((self.local_offset_g - 90))) + y * cos(
            radians((self.local_offset_g - 90))
        )

        current_pos_local.z = z

        return current_pos_local

    def wait4connect(self):
        rospy.loginfo(CYELLOW2 + "Waiting for FCU connection" + CEND)
        while not rospy.is_shutdown() and not self.current_state_g.connected:
            rospy.sleep(0.01)
        else:
            if self.current_state_g.connected:
                rospy.loginfo(CGREEN2 + "FCU connected" + CEND)
                return 0
            else:
                rospy.logerr(CRED2 + "Error connecting to drone's FCU" + CEND)
                return -1

    def wait4start(self):
        rospy.loginfo(CYELLOW2 + CBLINK +
                      "Waiting for user to set mode to GUIDED" + CEND)
        while not rospy.is_shutdown() and self.current_state_g.mode != "GUIDED":
            rospy.sleep(0.01)
        else:
            if self.current_state_g.mode == "GUIDED":
                rospy.loginfo(
                    CGREEN2 + "Mode set to GUIDED. Starting Mission..." + CEND)
                return 0
            else:
                rospy.logerr(CRED2 + "Error startting mission" + CEND)
                return -1

    def set_mode(self, mode):
        SetMode_srv = SetModeRequest(0, mode)
        response = self.set_mode_client(SetMode_srv)
        if response.mode_sent:
            # rospy.loginfo(CGREEN2 + "SetMode Was successful" + CEND)
            return 0
        else:
            rospy.logerr(CRED2 + "SetMode has failed" + CEND)
            return -1

    def arm(self):
        rospy.loginfo(CBLUE2 + "Arming Drone" + CEND)

        arm_request = CommandBoolRequest(True)

        while not rospy.is_shutdown() and not self.current_state_g.armed:
            rospy.sleep(0.1)
            response = self.arming_client(arm_request)
            self.local_pos_pub.publish(self.waypoint_g)
        else:
            if response.success:
                rospy.loginfo(CGREEN2 + "Arming successful" + CEND)
                return 0
            else:
                rospy.logerr(CRED2 + "Arming failed" + CEND)
                return -1

    def takeoff(self, takeoff_alt):
        self.arm()
        takeoff_srv = CommandTOLRequest(0, 0, 0, 0, takeoff_alt)
        response = self.takeoff_client(takeoff_srv)
        rospy.sleep(3)

        while ( abs(self.current_pose_g.pose.pose.position.z - takeoff_alt) > 0.25):
            pass

        
        if response.success:
            rospy.loginfo(CGREEN2 + "Takeoff successful" + CEND)
            return 0
        else:
            rospy.logerr(CRED2 + "Takeoff failed" + CEND)
            return -1

    def initialize_local_frame(self):
        """This function will create a local reference frame based on the starting location of the drone. This is typically done right before takeoff. This reference frame is what all of the the set destination commands will be in reference to."""
        self.local_offset_g = 0.0

        for i in range(30):
            rospy.sleep(0.1)

            q0, q1, q2, q3 = (
                self.current_pose_g.pose.pose.orientation.w,
                self.current_pose_g.pose.pose.orientation.x,
                self.current_pose_g.pose.pose.orientation.y,
                self.current_pose_g.pose.pose.orientation.z,
            )

            psi = atan2((2 * (q0 * q3 + q1 * q2)),
                        (1 - 2 * (pow(q2, 2) + pow(q3, 2))))

            self.local_offset_g += degrees(psi)
            self.local_offset_pose_g.x += self.current_pose_g.pose.pose.position.x
            self.local_offset_pose_g.y += self.current_pose_g.pose.pose.position.y
            self.local_offset_pose_g.z += self.current_pose_g.pose.pose.position.z

        self.local_offset_pose_g.x /= 30.0
        self.local_offset_pose_g.y /= 30.0
        self.local_offset_pose_g.z /= 30.0
        self.local_offset_g /= 30.0

        rospy.loginfo(CBLUE2 + "Coordinate offset set" + CEND)
        rospy.loginfo(
            CGREEN2 + "The X-Axis is facing: {}".format(self.local_offset_g) + CEND)



times=0
drone = drone_api()
frame_centre = [320.,240.]
pixel_threshold = 10
reached_marker_0 = 0
reached_marker_4 = 0
reached_color = 0
reached_marker_4_2 = 0
#    Marker Coordinates

#    x: -1.58653998375
#    y: 0.0905971601605

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
    if(mag>10):
        return mag
    return 0

def thres_col(cent_x,cent_y):
    mag_col=((cent_x-frame_centre[0])**2+(cent_y-frame_centre[1])**2)**(0.5)
    if(mag_col>10):
        return mag_col
    return 0


def img_callback(data):
    global times
    global reached_marker_0
    global reached_marker_4
    global reached_color
    global reached_marker_4_2

    br=CvBridge()
    cur_frame=br.imgmsg_to_cv2(data)
    gray = cv2.cvtColor(cur_frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = aruco.detectMarkers(gray, aruco_dict)
    # aruco.drawDetectedMarkers(cur_frame, corners)
    img_hsv = cv2.cvtColor(cur_frame, cv2.COLOR_BGR2HSV)

    mask = cv2.inRange(img_hsv, (5, 50, 50), (15, 255, 255))

    contours,_ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    
    mp.clear()

    if ids is not None:
        for i in range(len(ids)):
            mp[ids[i][0]]=[[corners[i][0]]]
    
    velocity_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=10)
    vel = TwistStamped()


    if (times == 0) and (0 not in mp.keys()):
        reached_marker_0 = 1
        # cv2.imshow('liveimg',cur_frame)
        # cv2.waitKey(1)
        rospy.loginfo_once("Finding First Box")
        vel.twist.linear.y = 0.2
        velocity_pub.publish(vel)
        return


    if (reached_marker_0 == 1):
        vel.twist.linear.y = 0
        times += 1
        velocity_pub.publish(vel)
        reached_marker_0 = 0
        rospy.loginfo_once("Found First Box")


    if (0 in mp.keys()) and (times == 1): ## Phase 1
        if not (thres(mp[0])):
            hovering_phase(1)
            return

        # aruco.drawDetectedMarkers(cur_frame, corners)
        
        v_x,v_y = reqd_velo(mp[0],0.2)
        rospy.loginfo_once("Centering on Box 1")
        vel.twist.linear.x = -v_y
        vel.twist.linear.y = -v_x
        
        velocity_pub.publish(vel)
        return

    if (times == 2) and (4 not in mp.keys()):
        reached_marker_4 = True
        vel.twist.linear.y = -0.2
        vel.twist.linear.x = -0.1
        ##########################################
        rospy.loginfo_once("Searching Dropzone")
        
        velocity_pub.publish(vel)
        return

    if (reached_marker_4 == True):
        vel.twist.linear.y = 0
        times += 1
        velocity_pub.publish(vel)
        reached_marker_4 = False
        rospy.loginfo_once("Found Dropzone")


    if (4 in mp.keys()) and (times == 3): ## Phase 1
        if not (thres(mp[4])):
            hovering_phase(2)
            return

        # aruco.drawDetectedMarkers(cur_frame, corners)
        
        v_x,v_y = reqd_velo(mp[4],0.1)

        vel.twist.linear.x = -v_y
        vel.twist.linear.y = -v_x
        rospy.loginfo_once("Centering on Dropzone")
        velocity_pub.publish(vel)
        return

    if (times == 4):
        rospy.loginfo_once("Finding Second Box")
        vel.twist.linear.y = 0.2
        velocity_pub.publish(vel)
        if len(contours):
            times+=1
        return

    if (times == 5):
        if len(contours):
            for c in contours:
                area = cv2.contourArea(c)
                if area > 60:
                    
                    x,y,w,h = cv2.boundingRect(c)
                    cur_frame = cv2.rectangle(cur_frame, (x,y), (x+w,y+h), (0,0,255), 5)
                    x_c=x+w//2
                    y_c=y+h//2
                    if not thres_col(x_c,y_c):
                        rospy.loginfo_once("Going down now")
                        hovering_phase(1)
                        return
                    rospy.loginfo_once("Centering on box")
                    v_x=x_c-frame_centre[0]
                    v_y=y_c-frame_centre[1]
                    p=(v_x**2+v_y**2)**(0.5)
                    v_x,v_y=0.2*v_x/p,0.2*v_y/p
                    vel.twist.linear.x = -v_y
                    vel.twist.linear.y = -v_x
                    velocity_pub.publish(vel)
                    return

    if (times == 6) and (4 not in mp.keys()):
        reached_marker_4_2 = True
        vel.twist.linear.y = -0.2
        rospy.loginfo_once("Searching Dropzone")
        
        velocity_pub.publish(vel)
        return

    if (reached_marker_4_2 == True):
        vel.twist.linear.y = 0
        times += 1
        velocity_pub.publish(vel)
        reached_marker_4_2 = False
        rospy.loginfo_once("Found Dropzone")

    if (4 in mp.keys()) and (times == 7):
        ## Phase 1
        if not (thres(mp[4])):
            rospy.loginfo("Zoning")
            hovering_phase(2)
            return

        v_x,v_y = reqd_velo(mp[4],0.1)

        vel.twist.linear.x = -v_y
        vel.twist.linear.y = -v_x
        rospy.loginfo_once("Centering on Dropzone")
        velocity_pub.publish(vel)
        
        return

    if times > 7:
        rospy.loginfo_once(CBOLD + CGREEN + "MISSION SUCCESSFUL, RETURNING TO BASE !" + CEND)
        rtl()
        

def hovering_phase(x):
    pose_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
    poser=PoseStamped()
    cur_x = drone.current_pose_g.pose.pose.position.x
    cur_y = drone.current_pose_g.pose.pose.position.y
    if(x==1):
        rospy.loginfo_once("Picking Box ")
    else : rospy.loginfo_once("Dropping Box")
    poser.pose.position.x = cur_x
    poser.pose.position.y = cur_y
    poser.pose.position.z = 0.3
    pose_pub.publish(poser)
    # rospy.loginfo("First pose was given")
    rospy.sleep(10)
    
    # rospy.loginfo("Second pose was given")
    while (drone.current_pose_g.pose.pose.position.z < 1.95):
        poser.pose.position.z = 2
        pose_pub.publish(poser)

    global times
    times += 1
    return

def rtl():
    drone.set_mode("RTL")
    global times
    times += 1

def takeoff_drone(): 
    drone.wait4connect()
    drone.wait4start()
    drone.initialize_local_frame()
    drone.takeoff(2)
    rate = rospy.Rate(3)
    rospy.loginfo(CGREEN2 + "Takeoff Completed" + CEND)

def recv():
    rospy.Subscriber('/webcam/image_raw',Image,img_callback)
    rospy.spin()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    # print(k)
    rospy.init_node("drone_controller", anonymous=True)
    takeoff_drone()
    recv()