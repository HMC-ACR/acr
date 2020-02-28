# Autonomous Campus Robot
## Teleop:
Plug joystick into Jetson and run:
```
roslaunch low_level teleop.launch  # ignore error msg about force feedback
rosrun low_level dyn_model
rosrun low_level motor_controller
```
Hold down joystick LB and use right wheel to control robot!

Untested: to use a remote laptop running Ubuntu 18.04 and ROS Melodic, ensure ROS connectivity between remote laptop and Jetson, plug in joystick and run:
```
source acr/catkin_ws/devel/setup.bash
sudo apt install ros-melodic-joy ros-melodic-teleop-twist-joy
roslaunch low_level teleop.launch  # may need to change launchfile if joystick does not enumerate as /dev/input/js0
```
## Running Odometry Publisher:
(X=0,Y=0) corresponds to (Lat=,Long=)
Yaw=0 degrees corresponds to east.
X is directly true east, Y is directly true north. Yaw is CCW about Z-axis.
"Adafruit-Blinka" Python module must be installed before running.
```
roscore # makes sure that a roscore is up and running
cd ~/acr/catkin_ws
catkin_make
sudo -s # run as root
rosrun low_level gps_imu.py
```

## SSH into Jetson from Remote Computer
Run `ifconfig` while connected to display and Claremont-ETC. Hope IP address stays the same once we take vehicle outside. Then connect to Claremont-ETC (password cityoftrees87) and `ssh acr@<IPAddr>` from remote computer.
