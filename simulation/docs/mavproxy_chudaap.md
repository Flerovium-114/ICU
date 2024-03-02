# MAVProxy Chudaap

1. `sudo mavproxy.py <args>` - don't do (because u cannot roslaunch with root permission)

2. instead, if usb connection, do the following
```bash
sudo chmod a+rw /dev/ttyACM0
mavproxy.py --master=/dev/ttyACM0 --out=udp:<ip>:<port>
```
and below command in other terminal
```bash
roslaunch mavros apm.launch fcu_url="/dev/ttyACM0:57600"
```

3. Fuck off
