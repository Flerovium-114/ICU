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
from sensor_msgs.msg import Image, NavSatFix
import numpy as np
from math import *

#from sensor_msgs import NavSatFix
#from pygeodesy.geoids import GeoidPGM

#_egm96 = GeoidPGM('/usr/share/GeographicLib/geoids/egm96-5.pgm', kind=-3)


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

        self.local_vel_pub = rospy.Publisher(
            name="{}mavros/setpoint_velocity/cmd_vel".format(self.ns),
            data_class=TwistStamped,
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

        '''self.current_global_pos = rospy.Subscriber(
            name="{}mavros/global_position/global".format(self.ns),
            data_class=NavSatFix,
            queue_size=10,
            callback=self.update_gps
        )'''

        '''self.gps_client = rospy.ServiceProxy(
            name="{}mavros/global_position/global".format(self.ns), service_class=CommandLong
        )'''


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
        #self.set_mode(216)
        rospy.loginfo(CBLUE2 + "Arming Drone" + CEND)
        
        arm_request = CommandBoolRequest(True)
        response = self.arming_client(arm_request)

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
    
    '''def update_gps(self,msg):
        self.curr_gps_lat=msg.latitude
        self.curr_gps_long=msg.longitude
        self.curr_gps_h=msg.altitude'''


    
    def coord_conversion(self,lat,lon):
        # r = 1
        r = [[0 for j in range(3)] for i in range(3)]
        r[0][0] = -sin(lat)*cos(lon)
        r[0][1] = -sin(lat)*sin(lon)
        r[0][2] = cos(lat)
        r[1][0] = -sin(lon)
        r[1][1] = cos(lon)
        r[1][2] = 0
        r[2][0] = -cos(lat)*cos(lon)
        r[2][1] = -cos(lat)*sin(lon)
        r[2][2] = -sin(lat)

        r = np.array(r)
        #p_ned = np.array(p_ned)


        
        lat_1 = (current_global_pos.latitude)*pi/180
        lon_1 = (current_global_pos.longitude)*pi/180
        p_ref = [cos(lat_1)*cos(lon_1),cos(lat_1)*sin(lon_1),sin(lat_1)]
        p_ecef = [[cos(lat)*cos(lon),cos(lat)*sin(lon),sin(lat)]]
       
        p_ref = np.array(p_ref)
        p_ecef = np.array(p_ecef)        

        p_ned = np.matmul(r,(p_ecef - p_ref))

       
        return p_ned


    def cruise(self):
       
        vel.twist.linear.x = 10*median_vector_unit[1]
        vel.twist.linear.y = 10*median_vector_unit[0]

        for i in range(20):
           
            self.local_vel_pub.publish(vel)
            rospy.sleep(1)

times=0
drone = drone_api()

'''start_lat_1 = (-35.3632622 + -35.3635750)/2
start_long_1 = (149.1652375 + 149.1643129)/2
start_lat_2 = (-35.3637 + -35.3639)/2
start_long_2 = (149.163 + 149.162)/2

ned1 = drone.coord_conversion(start_lat_1,start_long_1)
ned2 = drone.coord_conversion(start_lat_2,start_long_2)'''

median_vector = [1, 0 , 20]


#median_vector = [drone.coord_conversion(start_lat_2,start_long_2)-drone.coord_conversion(start_lat_1,start_long_1),drone.coord_conversion(drone,start_long_2)-drone.coord_conversion(start_long_1),0]
median_vector_unit = []
median_vector_mod = sqrt(median_vector[0]**2 + median_vector[1]**2 + median_vector[2]**2)

for i in range(3):
    median_vector_unit.append(median_vector[i]/median_vector_mod)
       

def rtl():
    drone.set_mode("RTL")
    global times

def takeoff_drone(): 
    print("Hi")
    drone.wait4connect()
    print("Hiya")
    drone.wait4start()
    print("Hey")
    drone.initialize_local_frame()
    drone.takeoff(2)
    rate = rospy.Rate(3)
    rospy.loginfo(CGREEN2 + "Takeoff Completed" + CEND)

vel = TwistStamped()

def recv():
    rospy.spin()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    # print(k)
    rospy.init_node("drone_controller", anonymous=True)
    takeoff_drone()
    
    drone.cruise()
    #recv()
