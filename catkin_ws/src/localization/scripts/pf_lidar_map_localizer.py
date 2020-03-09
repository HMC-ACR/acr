#!/usr/bin/env python
import rospy
import rospkg
import math
import random
import numpy as np
import time
from datetime import datetime
import board
import busio
import adafruit_bno055

from geometry_msgs.msg import Twist, Pose, PoseStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import PointCloud2

class ParticleFilter:
    DT = 0.1


    def __init__(self):
        np.random.seed()
        random.seed(datetime.now())


        self.particles = []
        self.numParticles = 700
        self.percent_randomize = 0.01
        self.num_randomize = self.numParticles*self.percent_randomize

        # i2c device must be on Jetson's i2c bus 1
        i2c = busio.I2C(board.SCL, board.SDA)
        self.sensor = adafruit_bno055.BNO055(i2c)

        # Initial state estimate
        self.x0 = 0 
        self.y0 = 0 
        self.x_dot0 = 0 
        self.y_dot0 = 0 
        self.theta0 = 0

        mean = [0.0, 0.0, 0.0]
        cov = [[0.2, 0.0, 0.0], [0.0, 0.2, 0.0], [0.0, 0.0, 0.09]]
        self.rv = ss.multivariate_normal(mean, cov)

        self.map_minX = 0.0
        self.map_maxX = 28.1  # meters
        self.map_minY = 0.0
        self.map_maxY = 29.9  # meters
    
        self.init_particles()

        rospy.init_node('pfLocalization', anonymous=True)
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)

        self.ouster_sub = rospy.Subscriber('/os1_node/points', PointCloud2, self.lidar_callback)

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

    def state_est_pub():
        ''' Publish state est'''
        odom_msg = self.get_state_est()
        self.odom_pub.publish(odom_msg)


    def init_particles(self):
        ''' At known location '''
        weight = 1/self.numParticles
        for i in range(0, self.numParticles):
            # Add noise to be more robust to poor placement
            x0 = self.x0 + np.random.normal(0.0, 0.1)
            y0 = self.y0 + np.random.normal(0.0, 0.1)
            theta0 = self.theta0 + np.random.normal(0.0, 0.02)
            self.particles.insert(i, self.Particle(x0, y0, self.x_dot0, self.y_dot0, theta0, weight))
            

    def set_rand_pos(self, particle):
        rand_x = random.uniform(self.map_minX, self.map_maxX)
        rand_y= random.uniform(self.map_minY, self.map_maxY)
        rand_theta = random.uniform(-math.pi, math.pi)

        particle.x = rand_x
        particle.y = rand_y
        particle.theta = rand_theta

    def get_imu_data(self):
        x_accel = self.sensor.linear_acceleartion[0]
        z_rot_vel = self.sensor.gyro[2]

        return x_accel, z_rot_vel


    def propagate(self):
        ''' Propogate particles based on cmd_vel/acceleramator'''
        x_accel, z_rot_vel = self.get_imu_data()

        for p in self.particles:
            x_dot = p.x_dot
            y_dot = p.y_dot
            x = p.x
            y = p.y
            theta = p.theta

            delta_x_dot = math.cos(theta)*x_accel*DT
            delta_y_dot = math.sin(theta)*x_accel*DT
            deleta_x = x_dot*DT + (0.5)*math.cos(theta)*x_accel*DT*DT
            deleta_y = y_dot*DT + (0.5)*math.sin(theta)*x_accel*DT*DT
            delta_theta = z_rot_vel*DT

            x_dot += delta_x_dot
            y_dot += delta_y_dot
            x += delta_x
            y += delta_y
            theta += delta_theta
            theta = self.angleDiff(theta)

            p.x = x
            p.y = y
            p.x_dot = x_dot
            p.y_dot = y_dot
            p.theta = theta

        
    def calculate_weight(self, meas_state_est):
        ''' Calculate weights '''
        for p in self.particles:
            delta = [p.x - meas_state_est[0], p.y - meas_state_est[1], self.angleDiff(p.theta - meas_state_est[2])]
            p.weight = self.rv.pdf(delta)


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
        x_sum = 0
        y_sum = 0
        x_dot_sum = 0
        y_dot_sum = 0
        theta_x_sum = 0
        theta_y_sum = 0
        self.particle_weight_sum = 0

        # We add angles in cartesian space to average
        for particle in self.particles:
            self.particle_weight_sum += particle.weight
            x_sum += particle.weight*particle.x
            y_sum += particle.weight*particle.y
            x_dot_sum += particle.weight*particle.x_dot
            y_dot_sum += particle.weight*particle.y_dot
            theta_x_sum += particle.weight*math.cos(particle.theta)
            theta_y_sum += particle.weight*math.sin(particle.theta)

        xavg = x_num/self.particle_weight_sum
        yavg = y_num/self.particle_weight_sum
        x_dotavg = x_num/self.particle_weight_sum
        y_dotavg = y_num/self.particle_weight_sum
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
        def __init__(self, x, y, x_dot, y_dot theta, weight):
            self.x = x
            self.y = y
            self.x_dot = x_dot
            self.y_dot = y_dot
            self.theta = theta
            self.weight = weight

        def __str__(self):
            return str(self.x) + " " + str(self.y) + " " + str(self.heading) + " " + str(self.weight)
