# Autonomous Campus Robot
Teleop:
On remote laptop running Ubuntu 18.04 and ROS Melodic:
```
source acr/catkin_ws/devel/setup.bash
apt install ros-melodic-joy
apt install ros-melodic-teleop-twist-joy
roslaunch low_level teleop.launch  # ignore error msg about force feedback
```
Ssh into Jetson and run:
```
rosrun low_level dyn_model
rosrun low_level motor_controller
```
Hold down joystick LB and use right wheel to control robot!
