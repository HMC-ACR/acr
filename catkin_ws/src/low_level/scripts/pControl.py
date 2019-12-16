#!/usr/bin/env python
import rospy
import rospkg
import serial
import tf
import numpy as np
from math import cos, sin, pi, exp, sqrt, atan2

from geometry_msgs.msg import Twist, Pose, PoseStamped
from nav_msgs.msg import Odometry, Path
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class PointTracker:
    def __init__(self):

        # Initialize member fields
        self.current_target = Pose()
        self.path_distance = 0.0
        self.alpha = 0.0
        self.beta = 0.0
        self.rho = 0.0
        self.k_alpha = 1.0/4.0 #6.0/8.0  #untuned, just working from strong stability conditions
        self.k_beta = -2.0/8.0    
        self.k_rho = 1.0/4.1
        #print("Kalpha is" self.k_alpha)

        rospy.init_node('pointTracking', anonymous=True)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.odom_sub = rospy.Subscriber('/decawave/odom', Odometry, self.odom_callback)
        self.target_sub = rospy.Subscriber('/astar_planner/current_target', PoseStamped, self.target_callback)
        #self.path_sub = rospy.Subscriber('/astar_planner/global_path', Path, self.path_callback)
        self.rate = rospy.Rate(10)    #10 Hz

        
        while not rospy.is_shutdown():
            self.cmd_vel_pub() #publish /cmd_vel at 10 Hz
            self.rate.sleep()
        
    def target_callback(self, target):
        self.current_target = target.pose
        rospy.loginfo('Updating current target' )
        #print("hi")

    def path_callback(self, path):
        #estimate for error in rho
        self.path_distance = len(path.poses)*0.5 #grid cells are 0.5 m
        
    
    def odom_callback(self, odom):
        if (self.current_target.position.x == 0 and self.current_target.position.y == 0):
            #Do nothing since target has not been initialized
            print('Target needs to be initilized to someting that is not origin')
        else: #WE have a target, and we have moved, so recalc errors
            delta_x = self.current_target.position.x - odom.pose.pose.position.x 
            delta_y = self.current_target.position.y - odom.pose.pose.position.y 
            self.rho = sqrt(delta_x ** 2 + delta_y ** 2)
            poseQuat = odom.pose.pose.orientation
            poseQuat_arr = np.array([poseQuat.x, poseQuat.y, poseQuat.z, poseQuat.w])
            eulerAngles = euler_from_quaternion(poseQuat_arr, 'sxyz')
            theta = eulerAngles[2] #We want yaw
            self.alpha = -theta + atan2(delta_y, delta_x)
            self.beta = -theta - self.alpha
            print("alpha:"+str(self.alpha) + "beta" + str(self.beta) + "rho:" + str(self.rho)+ "theta" + str(theta))

    def cmd_vel_pub(self):
        # Compute desired velocities based on control laws
        v = self.k_rho * self.rho

        w = self.k_alpha * self.alpha #+ self.k_beta * self.beta
        if(self.rho < 0.5): #ONce close enough to point, stop moving
            v = 0
            w = 0
        
        # Create message and publish
        twist_lin = Twist()
        twist_ang = Twist()        #message object is a Twist
        if (abs(self.alpha) > 0.52): #if angle is bigger tha 30 degrees, then we need to turn 
            twist_ang.angular.z = w
            self.pub.publish(twist_ang)
        elif (w ==0):
            twist_lin.linear.x = v
            self.pub.publish(twist_lin)
        else: #both components
            twist_ang.angular.z = w
            twist_lin.linear.x = v
            self.pub.publish(twist_lin)
            self.pub.publish(twist_ang)

        
        
        


if __name__ == '__main__':
    try:
        tracker = PointTracker()
    except rospy.ROSInterruptException:
        pass
