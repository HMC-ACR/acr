#!/usr/bin/env python3

import serial
import math
import time
import pynmea2
import board
import busio
import adafruit_bno055
import rospy
import rospkg
from geometry_msgs.msg import TwistWithCovariance, PoseWithCovariance
from nav_msgs.msg import Odometry

EARTH_RADIUS_M = 6.371E6
RADS_PER_DEG = math.pi/180.0
# Origin located in middel of Sprague Courtyard
ORIGIN_LAT_DEG = 34.106103
ORIGIN_LON_DEG = -117.711969
ORIGIN_LAT_RAD = ORIGIN_LAT_DEG*RADS_PER_DEG
ORIGIN_LON_RAD = ORIGIN_LON_DEG*RADS_PER_DEG
COS_ORIGIN_LAT = math.cos(ORIGIN_LAT_RAD)

class GPSIMU():

    def __init__(self):

        # ROS
        self.odom = Odometry()
        rospy.init_node('Localization', anonymous=True)
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        self.rate = rospy.Rate(10)  # 10 Hz

        # i2c device must be on Jetson's i2c bus 1
        i2c = busio.I2C(board.SCL, board.SDA)
        self.sensor = adafruit_bno055.BNO055(i2c)
        print("connected to IMU")

        # set up serial connection to GPS
        self.ser = serial.Serial("/dev/ttyACM0", 9600)
        print("connected to GPS")

        while not rospy.is_shutdown():
            self.update()
            self.odom_pub.publish(self.odom)
            self.rate.sleep()

    def update(self):
        # Read GPS
        msg = self.ser.readline().decode().strip('\r\n')
        if (msg[0:6] == '$GNGGA'):
            parsed_msg = pynmea2.parse(msg)
            print(type(parsed_msg.latitude))
            print(parsed_msg.latitude)

            # Latitude in x, Longitude in y
            #if parsed_msg.longitude == "" or parsed_msg.latitude == "":
            #    self.odom.pose.pose.position.x = 0
            #    self.odom.pose.pose.position.y = 0
            #else:
            self.odom.pose.pose.position.x = EARTH_RADIUS_M*(parsed_msg.longitude*RADS_PER_DEG -
                    ORIGIN_LON_RAD)*COS_ORIGIN_LAT
            self.odom.pose.pose.position.y = EARTH_RADIUS_M*(parsed_msg.latitude*RADS_PER_DEG -
                    ORIGIN_LAT_RAD)

        # Read IMU
        # linear acc
        # note that we are putting acc. values in the velocity msg
        self.odom.twist.twist.linear.x = self.sensor.linear_acceleration[0]
        self.odom.twist.twist.linear.y = self.sensor.linear_acceleration[1]
        self.odom.twist.twist.linear.z = self.sensor.linear_acceleration[2]
        # quaternion
        self.odom.pose.pose.orientation.x = self.sensor.quaternion[0]
        self.odom.pose.pose.orientation.y = self.sensor.quaternion[1]
        self.odom.pose.pose.orientation.z = self.sensor.quaternion[2]
        self.odom.pose.pose.orientation.w = self.sensor.quaternion[3]

if __name__ == '__main__':
    try:
        gpsimu = GPSIMU()
    except rospy.ROSInterruptException:
        pass
