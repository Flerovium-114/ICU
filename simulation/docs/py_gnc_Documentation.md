# Documentation of the py_gnc Library

## Class Definition
```python
class gnc_api():
    self.current_state_g = State()
    self.current_pose_g = Odometry()
    self.correction_vector_g = Pose()
    self.local_offset_pose_g = Point()
    self.waypoint_g = PoseStamped()

    self.current_heading_g = 0.0
    self.local_offset_g = 0.0
    self.correction_heading_g = 0.0
    self.local_desired_heading_g = 0.0

    self.ns #ROS_namespace

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

    # arm, land, takeoff set_mode, command cliens, defined after wait_for_service(service)


```
Using the library

```python
from py_gnc import *

drone = gnc_api()
```
## Summary of useful functions

### get_current_heading

```python
get_current_heading()
```

Returns the current heading of the drone.

**Returns**:

- `Heading` _Float_ - θ in is degrees.

---

### get_current_location

```python
get_current_location()
```

Returns the current position of the drone.

**Returns**:

- `Position` *geometry_msgs.msgs.Point()* - Returns position of type Point().

---
### land

```python
land()
```

The function changes the mode of the drone to LAND.

**Returns**:

- `0` _int_ - LAND successful.
- `-1` _int_ - LAND unsuccessful.

---

### wait4connect

```python
wait4connect()
```

Wait for connect is a function that will hold the program until communication with the FCU is established.

**Returns**:

- `0` _int_ - Connected to FCU.
- `-1` _int_ - Failed to connect to FCU.

---

### wait4start

```python
wait4start()
```

This function will hold the program until the user signals the FCU to mode enter GUIDED mode. This is typically done from a switch on the safety pilot's remote or from the Ground Control Station.

**Returns**:

- `0` _int_ - Mission started successfully.
- `-1` _int_ - Failed to start mission.

---

### set_mode

```python
set_mode(mode)
```

This function changes the mode of the drone to a user specified mode. This takes the mode as a string. \
`Ex. set_mode("GUIDED")`.

**Arguments**:

- `mode` _String_ - Can be set to modes given in https://ardupilot.org/copter/docs/flight-modes.html
  

**Returns**:

- `0` _int_ - Mode Set successful.
- `-1` _int_ - Mode Set unsuccessful.

---

### set_speed

```python
set_speed(speed_mps)
```

This function is used to change the speed of the vehicle in guided mode. It takes the speed in meters per second as a float as the input.

**Arguments**:

- `speed_mps` _Float_ - Speed in m/s.
  

**Returns**:

- `0` _int_ - Speed set successful.
- `-1` _int_ - Speed set unsuccessful.

---

### set_heading

```python
set_heading(heading)
```

This function is used to specify the drone's heading in the local reference frame. Psi is a counter clockwise rotation following the drone's reference frame defined by the x axis through the right side of the drone with the y axis through the front of the drone.

**Arguments**:

- `heading` _Float_ - θ(degree) Heading angle of the drone.

---

### set_destination

```python
set_destination(x, y, z, psi)
```

This function is used to command the drone to fly to a waypoint. These waypoints should be specified in the local reference frame. This is typically defined from the location the drone is launched. Psi is counter clockwise rotation following the drone's reference frame defined by the x axis through the right side of the drone with the y axis through the front of the drone.

**Arguments**:

- `x` _Float_ - x(m) Distance with respect to your local frame.
- `y` _Float_ - y(m) Distance with respect to your local frame.
- `z` _Float_ - z(m) Distance with respect to your local frame.
- `psi` _Float_ - θ(degree) Heading angle of the drone.

---

### arm

```python
arm()
```

Arms the drone for takeoff.

**Returns**:

- `0` _int_ - Arming successful.
- `-1` _int_ - Arming unsuccessful.

---

### takeoff

```python
takeoff(takeoff_alt)
```

The takeoff function will arm the drone and put the drone in a hover above the initial position.

**Arguments**:

- `takeoff_alt` _Float_ - The altitude at which the drone should hover.
  

**Returns**:

- `0` _int_ - Takeoff successful.
- `-1` _int_ - Takeoff unsuccessful.

---

### initialize_local_frame

```python
initialize_local_frame()
```

This function will create a local reference frame based on the starting location of the drone. This is typically done right before takeoff. This reference frame is what all of the the set destination commands will be in reference to.

---

### check_waypoint_reached

```python
check_waypoint_reached(pos_tol=0.3, head_tol=0.01)
```

This function checks if the waypoint is reached within given tolerance and returns an int of 1 or 0. This function can be used to check when to request the next waypoint in the mission.

**Arguments**:

- `pos_tol` _float, optional_ - Position tolerance under which the drone must be with respect to its position in space. Defaults to 0.3.
- `head_tol` _float, optional_ - Heading or angle tolerance under which the drone must be with respect to its orientation in space. Defaults to 0.01.
  

**Returns**:

- `1` _int_ - Waypoint reached successfully.
- `0` _int_ - Failed to reach Waypoint.
