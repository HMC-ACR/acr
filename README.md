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
apt install ros-melodic-joy
apt install ros-melodic-teleop-twist-joy
roslaunch low_level teleop.launch  # may need to change launchfile if joystick does not enumerate as /dev/input/js0
```
