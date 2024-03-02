# Import necessary packages
import rospy
from mavros_msgs.msg import PositionTarget
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix
from aruco_msgs.msg import MarkerArray

# Define grid search class
class GridSearch:
    def __init__(self):
        # Set initial state
        self.state = "INIT"
        
        # Set initial position and orientation
        self.current_pose = PoseStamped()
        self.current_pose.pose.position.x = 0
        self.current_pose.pose.position.y = 0
        self.current_pose.pose.position.z = 0
        self.current_pose.pose.orientation.x = 0
        self.current_pose.pose.orientation.y = 0
        self.current_pose.pose.orientation.z = 0
        self.current_pose.pose.orientation.w = 1
        
        # Set initial GPS position
        self.gps_position = NavSatFix()
        self.gps_position.latitude = 0
        self.gps_position.longitude = 0
        self.gps_position.altitude = 0
        
        # Set search grid and Aruco markers
        self.search_grid = [[0, 0, 0, 0, 0],
                            [0, 0, 0, 0, 0],
                            [0, 0, 0, 0, 0],
                            [0, 0, 0, 0, 0],
                            [0, 0, 0, 0, 0]]
        self.markers = []
        
        # Initialize publishers and subscribers
        self.local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
        self.gps_pos_pub = rospy.Publisher("mavros/setpoint_position/global", NavSatFix, queue_size=10)
        self.aruco_sub = rospy.Subscriber("aruco_marker_publisher/markers", MarkerArray, self.aruco_callback)
        self.local_pos_sub = rospy.Subscriber("mavros/local_position/pose", PoseStamped, self.local_pos_callback)
        self.gps_pos_sub = rospy.Subscriber("mavros/global_position/global", NavSatFix, self.gps_pos_callback)
        
        # Set search rate
        self.rate = rospy.Rate(10)
        
    # Define Aruco marker callback function
    def aruco_callback(self, msg):
        self.markers = msg.markers
        
    # Define local position callback function
    def local_pos_callback(self, msg):
        self.current_pose = msg
        
    # Define GPS position callback function
    def gps_pos_callback(self, msg):
        self.gps_position = msg
        
    # Define function to move to next search location
    def move_to_next(self):
        # Set target position and orientation
        target_pos = PoseStamped()
        target_pos.header.stamp = rospy.Time.now()
        target_pos.header.frame_id = "base_footprint"
        target_pos.pose.position.x = 0
        target_pos.pose.position.y = 0
        target_pos.pose.position.z = 2
        target_pos.pose.orientation.x = 0
        target_pos.pose.orientation.y = 0
        target_pos.pose.orientation.z = 0
        target_pos.pose.orientation.w = 1
        
        # Set target GPS position
        target_gps = NavSatFix()
        target_gps.header.stamp = rospy.Time.now()
        target_gps.header.frame_id = "global_frame"
        target_gps.latitude = 0
        target_gps.longitude = 0
        target_gps.altitude = 2
        
        # Find next search location
        for i in range(len(self.search_grid)):
            for j in range(len(self.search_grid[i])):
                if self.search_grid[i][j] == 0:
                    target_pos.pose.position.x = i
                    target_pos.pose.position.y = j
                    target_gps.latitude = i
                    target_gps.longitude = j
                    self.search_grid[i][j] = 1
                    break
        
        # Publish target position and orientation
        self.local_pos_pub.publish(target_pos)
        self.gps_pos_pub.publish(target_gps)
        
        # Update state
        self.state = "MOVING"

    # Define main search loop
    def search(self):
        while not rospy.is_shutdown():
            if self.state == "INIT":
                # Move to initial search location
                self.move_to_next()
                
            elif self.state == "MOVING":
                # Check if drone has reached target position
                if (abs(self.current_pose.pose.position.x - target_pos.pose.position.x) < 0.1 and
                    abs(self.current_pose.pose.position.y - target_pos.pose.position.y) < 0.1 and
                    abs(self.current_pose.pose.position.z - target_pos.pose.position.z) < 0.1):
                    # Update state
                    self.state = "SEARCHING"
                    
            elif self.state == "SEARCHING":
                # Check if any Aruco markers are detected
                if len(self.markers) > 0:
                    # Update state
                    self.state = "FOUND"
                    
                else:
                    # Move to next search location
                    self.move_to_next()
                    
            elif self.state == "FOUND":
                # Land drone
                # TODO: implement landing sequence


