#!/usr/bin/env python
import rospy
import rospkg
import math
import random
import numpy as np
import time

from geometry_msgs.msg import Twist, Pose, PoseStamped
from nav_msgs.msg import Odometry, Path

class ParticleFilter:


    def __init__(self):
        self.particles = []
        self.numParticles = 700

        # Initial state estimate
        self.x0 = 0 
        self.y0 = 0 
        self.theta0 = 0
    
        self.init_particles()

        rospy.init_node('pfLocalization', anonymous=True)
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)

        #TODO(apham): Peter fill in this with the point cloud 2 msg from the Ouster
        self.ouster_sub = rospy.Subscriber('/point_cloud', PoseStamped, self.lidar_callback)

        #TODO(apham): Fill out with acclereramtor callback or cmd_vel
        self.ouster_sub = rospy.Subscriber('/point_cloud', PoseStamped, self.imu_callback)

        self.rate = rospy.Rate(10)    #10 Hz

        while not rospy.is_shutdown():
            self.state_est_pub() #publish /cmd_vel at 10 Hz
            self.rate.sleep()


    def lidar_callback(self, point_cloud):
        ''' Run PF to update state est '''
        self.propagate()
        meas_state_est = self.run_icp(point_cloud)
        self.calculate_weights(meas_state_est)
        self.resample()

    #TODO(apham): Implement to update internal variable
    def imu_callback(self, meas):
        pass

    def state_est_pub():
        ''' Publish state est'''
        odom_msg = self.get_state_est()
        self.odom_pub.publish(odom_msg)


    def init_particles(self):
        ''' At known location '''
        weight = 1/self.numParticles
        for i in range(0, self.numParticles):
            # TODO(apham): add some white noise to the particles init to increase robustness to poor
            # placement
            self.particles.insert(i, self.Particle(self.x0, self.y0, self.theta0, weight))
            

    def set_rand_pos(self, particle):
        # TODO(apham): implement random state generation
        rand_x = 0
        rand_y= 0
        rand_theta = 0

        particle.x = rand_x
        particle.y = rand_y
        particle.theta = rand_theta


    def propagate(self):
        ''' Propogate particles based on cmd_vel/acceleramator'''
        for p in self.particles:
            x = p.x
            y = p.y
            theta = p.theta

            #TODO(apham): implement propogation with motion model
            delta_theta = 0
            delta_x = 0
            delta_y = 0

            x += delta_x
            y += delta_y
            theta += delta_theta
            theta = self.angleDiff(theta)

            p.x = x
            p.y = y
            p.theta = theta

        
    def calculate_weight(self, meas_state_est):
        ''' Calculate weights '''
        for p in self.particles:
            #TODO(apham): implement weight calculation (multivariate gaussian?)
            p.weight = p.weight


    def resample(self):
        ''' Approx. resampling '''
        # Create sampling pool
        w_max = max(map(lambda x: x.weight, self.particles))
        particle_pool = []
        for p in self.particles:
            w_i = p.weight/w_max
            if w_i < 0.2:
                    particle_pool += [p]
            if w_i < 0.4:
                    particle_pool += 2*[p]
            elif w_i < 0.6:
                    particle_pool += 4*[p]
            elif w_i < 0.8:
                    particle_pool += 6*[p]
            elif w_i < 0.9:
                    particle_pool += 8*[p]
            elif w_i <= 1:
                    particle_pool += 10*[p]

        # Sample from pool
        for i in range(self.numParticles):
            r = int(random.uniform(0, len(particle_pool)-1))

            self.particles[i].x = particle_pool[r].x
            self.particles[i].y = particle_pool[r].y
            self.particles[i].theta = particle_pool[r].theta
            self.particles[i].weight = particle_pool[r].weight

        # Introdue new particles for robustness
        for i in range(self.num_randomize):
            random_particle = int(random.uniform(0, self.numParticles-1))
            self.SetRandomStartPos(random_particle)


    def run_icp(self, point_cloud):
        # TODO(apham): implement ICP to return a translation/rotation/pose estimate of the robot
        pass


    def get_state_est(self):
        ''' Get state est from particles and pack into ros msg'''
        # TODO(apham): update if we choose different states
        x_sum = 0
        y_sum = 0
        theta_x_sum = 0
        theta_y_sum = 0
        self.particle_weight_sum = 0

        # We add angles in cartesian space to average
        for particle in self.particles:
            self.particle_weight_sum += particle.weight
            x_sum += particle.weight*particle.x
            y_sum += particle.weight*particle.y
            theta_x_sum += particle.weight*math.cos(particle.heading)
            theta_y_sum += particle.weight*math.sin(particle.heading)

        xavg = x_num/self.particle_weight_sum
        yavg = y_num/self.particle_weight_sum
        thetaavg = self.angleDiff(math.atan2(theta_y, theta_x))

        # TODO(apham): pack state into odom msg
        
        return odom_msg


    def angleDiff(self, ang):
        ''' Wrap angles between -pi and pi'''
        while ang < -math.pi:
            ang = ang + 2 * math.pi
        while ang > math.pi:
            ang = ang - 2 * math.pi
        return ang


    class Particle:
        def __init__(self, x, y, theta, weight):
            self.x = x
            self.y = y
            self.theta = theta
            self.weight = weight

        def __str__(self):
            return str(self.x) + " " + str(self.y) + " " + str(self.heading) + " " + str(self.weight)
