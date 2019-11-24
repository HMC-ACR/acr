#!/usr/bin/env python

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
from nag_msgs.msgs import Odometry

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
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        self.rate = rospy.Rate(10)  # 10 Hz
        rospy.init_node('Localization', anonymous=True)

        while not rospy.is_shutdown():
            self.odom_pub(self.odom)
            self.rate.sleep()

    def init_imu():
        # i2c device must be on Jetson's i2c bus 1
        i2c = busio.I2C(board.SCL, board.SDA)
        sensor = adafruit_bno055.BNO055(i2c)

        print("connected to IMU")

    def init_gps():
        # set up serial connection to GPS
        ser = serial.Serial("/dev/ttyACM0", 9600)
        print("connected to GPS")

    def update():
        # Read GPS
        msg = ser.readline().decode().strip('\r\n')
        if (msg[0:6] == '$GNGGA'):
            parsed_msg = pynmea2.parse(msg)
            # Latitude in x, Longitude in y
            self.odom.pose.pose.position.x = EARTH_RADIUS_M*(parsed_msg.lon*RADS_PER_DEG -
                    ORIGIN_LON_RAD)*COS_ORIGIN_LAT
            self.odom.pose.pose.position.y = EARTH_RADIUS_M*(pasred_msg.lat*RADS_PER_DEG -
                    ORIGIN_LAT_RAD)

        # Read IMU
        # linear acc
        # note that we are putting acc. values in the velocity msg
        self.odom.twist.twist.linear.x = sensor.linear_acceleration[0]
        self.odom.twist.twist.linear.y = sensor.linear_acceleration[1]
        self.odom.twist.twist.linear.z = sensor.linear_acceleration[2]
        # quaternion
        self.odom.pose.pose.orientation.x = quaternion[0]
        self.odom.pose.pose.orientation.y = quaternion[1]
        self.odom.pose.pose.orientation.z = quaternion[2]
        self.odom.pose.pose.orientation.w = quaternion[3]

if __name__ == '__main__':
    try:
        gpsimu = GPSIMU()
    except rospy.ROSInterruptException:
        pass
