#!/usr/bin/env python
import rospy
import rospkg

import tf

import random
import math
import numpy as np

from road_map_node import PRM_Node 

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap

from e190_bot.srv import *


from tf.transformations import euler_from_quaternion, quaternion_from_euler

class prm_planning:

	def __init__(self):

		rospy.init_node('prm_planning', anonymous=True)

		self.path_init()
		self.map_init()# call map_server using service, other methods possible

		self.start_i = 0
		self.start_j = 0
		self.start_id = 0 #not important most of time

		self.goal = PoseStamped()
		self.goal_i = 0
		self.goal_j = 0
		self.goal_id = 0 #not important most of time


		self.current_x = .0
		self.current_y = .0
		quat = quaternion_from_euler(.0, .0, .0)
		self.current_o = Quaternion()
		self.current_o.x = quat[0]
		self.current_o.y = quat[1]
		self.current_o.z = quat[2]
		self.current_o.w = quat[3]

		self.nodes = [] #an array of all nodes in our tree

		s = rospy.Service('path_Service', path_Service, self.path_Service_callback)

		# subscribe to /goal topic, you can use "2D Nav Goal" tab on RViz to set a goal by mouse
		# config 2D Nav Goal using panels->tool properties
		rospy.Subscriber("/goal", PoseStamped, self.goal_callback)
		#rospy.Subscriber("/odom", Odometry, self.odom_callback)

		self.pubPlan = rospy.Publisher('/plan', Path, queue_size=10)

		rospy.spin()
		#self.rate = rospy.Rate(2)
		#while not rospy.is_shutdown():
		#	self.rate.sleep();


	def path_init(self):
		self.prm_plan = Path()
		self.prm_plan.header.frame_id = "map"
		self.roadmap = []
		self.start_node = PRM_Node()
		self.goal_node = PRM_Node()

	def map_init(self):
		rospy.wait_for_service('static_map')
		try:
			map_Service = rospy.ServiceProxy('static_map', GetMap)
			self.map = map_Service().map
			self.map_width = self.map.info.width
			self.map_height = self.map.info.height
			self.map_res = self.map.info.resolution

			# print(self.map.data)
			# print("Map size: " + str(self.map_res*self.map_width) +" , " + str(self.map_res*self.map_height))
		except rospy.ServiceException, e:
			print "Map service call failed: %s"%e


	def goal_callback(self, Goal):
		self.goal = Goal

	def path_Service_callback(self,req):
		Goal = self.goal
		start = req.current
		self.goal_x = min(Goal.pose.position.x, self.map_width*self.map_res*0.99)
		self.goal_x = max(self.goal_x, 0)
		self.goal_y = min(Goal.pose.position.y, self.map_height*self.map_res*0.99)
		self.goal_y = max(self.goal_y, 0)
		self.goal_o = Goal.pose.orientation	
		
		self.start_x = min(start.pose.position.x, self.map_width*self.map_res*0.99)
		self.start_x = max(self.start_x, 0)
		self.start_y = min(start.pose.position.y, self.map_height*self.map_res*0.99)
		self.start_y = max(self.start_y, 0)
		self.start_o = start.pose.orientation	

		#print("goal_clicked: " + str(self.goal_x) + " , " + str(self.goal_y))

		self.goal_i, self.goal_j, self.goal_id = self.pos_to_grid(self.goal_x, self.goal_y)
		self.start_i, self.start_j, self.start_id = self.pos_to_grid(self.start_x,self.start_y)


		self.plan_path()

		#self.pubPlan.publish(self.prm_plan) #plan is published here!
		return path_ServiceResponse(self.prm_plan.poses[1])

	def odom_callback(self,Odom):
		# When you are using your actual robot, you need to update this
		# Let's assume it starts at 0

		self.current_x = .0
		self.current_y = .0

		quat = quaternion_from_euler(.0, .0, .0)
		self.current_o = Quaternion()
		self.current_o.x = quat[0]
		self.current_o.y = quat[1]
		self.current_o.z = quat[2]
		self.current_o.w = quat[3]


	def plan_path(self):
		# Core function! modify as you wish! Here is only a demo that yield definitely wrong thing
		# Here is an example how do you deal with ROS nav_msgs/Path
		# start_pose = PoseStamped()
		# start_pose.header.frame_id = "map"
		# start_pose.pose.position.x = self.start_x
		# start_pose.pose.position.y = self.start_y
		# start_pose.pose.position.z = 0

		# start_pose.pose.orientation = self.start_o

		# goal_pose = PoseStamped()
		# goal_pose.header.frame_id = "map"
		# goal_pose.pose.position.x = self.goal_x
		# goal_pose.pose.position.y = self.goal_y
		# goal_pose.pose.position.z = 0

		# goal_pose.pose.orientation = self.goal_o


		# self.prm_plan.poses.append(start_pose)
		# self.prm_plan.poses.append(goal_pose)
		
		# print("I have: " + str(len(self.prm_plan.poses)) + " poses in path planned")

		#Rapidly - Exploring Tree PRM
		# Here is a hint that how you deal "topological" prm_node

		#create goal node
		self.goal_node.x = self.goal_x
		self.goal_node.y = self.goal_y
		self.goal_node.index = 1

		# make sure start node is free
		for i in range(1, 10):
			grid_i1, grid_j1, grid_id1 = self.pos_to_grid(self.start_x, self.start_y)

			if self.map.data[grid_id1]==0:
				break
			
			print("Jiggling start node")
			# otherwise randomly displace start_pose 
			self.start_x += -0.01
			self.start_y += -0.01

		#create start node
		self.start_node.x = self.start_x
		self.start_node.y = self.start_y
		self.start_node.index = 0

		# list to keep track of nodes
		self.nodes = [self.start_node]

		#clear path
		self.prm_plan.poses = []
		#Loop 
		last_node = self.start_node
		numiter =0
		#Stop loop if no collision between node and goal
		# by checking each new node for a path to goal here, we take a greedy approach
		# so we stop as soon as we find any path
		while not self.collisionDetect(last_node.x, last_node.y, self.goal_node.x, self.goal_node.y):
			numiter +=1
			#Generate random target location for node
			last_node_coordinates = self.generate_Node()
			#print(len(self.nodes))
			#print(last_node_coordinates)

			#FInd nearest existing node to target using nearest vertex
			#Loop through array, and calculate distances from each of the nodes
			minDistance = 2*(self.map_width * self.map_res + self.map_height * self.map_res)
			minDistance_node = None
			for node in self.nodes:
				current_distance = math.sqrt((last_node_coordinates[0] - node.x)**2 + (last_node_coordinates[1] - node.y)**2)
				if current_distance < minDistance:
					minDistance_node = node
					minDistance = current_distance
			#print(str(minDistance))

			#Add a node in the direction of the target at a random radius from nearest node
			new_node_x_vector = last_node_coordinates[0] - minDistance_node.x  
			new_node_y_vector = last_node_coordinates[1] - minDistance_node.y
			new_node_scaling = random.uniform(0.01,0.2)
			new_node_x = new_node_x_vector*new_node_scaling + minDistance_node.x
			new_node_x = min( 2.99, new_node_x)
			new_node_y = new_node_y_vector*new_node_scaling + minDistance_node.y
			new_node_y = min( 2.99, new_node_y)

			#print("new node: ")
			#print(new_node_x, new_node_y)

			#Then check that their is a collision free path to target
			if self.collisionDetect(minDistance_node.x, minDistance_node.y, new_node_x, new_node_y):
				#Add to tree and to array
				new_node = PRM_Node(new_node_x, new_node_y, minDistance_node)
				minDistance_node.addChild(new_node)
				self.nodes.append(new_node)
				last_node = new_node
			
			#print("last node: ")
			#print(last_node.x, last_node.y)

			
			#Now the loop will break, and we can calculate the path to the node
			#Just go up the tree looking at parents from new_node all the way to start_node
		
		path = self.path_backtrack(last_node)
		path.append(self.goal_node)

		#Path smoothing
 		for n in range(1,40*len(path)):
			index1 = random.randint(0,len(path)-1)
			index2 = random.randint(0,len(path)-1)
			node1 = path[index1]
			node2 = path[index2]
			dist = self.distance(node1.x, node1.y, node2.x, node2.y)
			if (self.collisionDetect(node1.x, node1.y, node2.x, node2.y)) and dist < 0.4 :
				if(index1 < index2):
					path = path[:index1+1]+path[index2:]
				elif (index1 > index2):
					path = path[:index2+1]+path[index1:] 
		#Otherwise, indexes equal, do nothing
					


		#Convert nodes to poses
		for node in path :
			node_pose = PoseStamped()
			node_pose.header.frame_id = "map"
			node_pose.pose.position.x = node.x
			node_pose.pose.position.y = node.y
			node_pose.pose.position.z = 0
			#add poses to PRM path
			self.prm_plan.poses.append(node_pose)


	def distance(self, x1, y1, x2, y2):
		dist = np.sqrt((x2-x1)**2 + (y2-y1)**2)
		return dist

	#want list of nodes-first element is start node, last node is new node
	def path_backtrack(self, new_node):
		if (new_node.parent is not None):
			return self.path_backtrack(new_node.parent) + [new_node] 
		else:
			#no parent
			return [new_node]

	def generate_Node(self):
		x = random.uniform(0,self.map_width * self.map_res*0.99)
		y = random.uniform(0, self.map_height * self.map_res*0.99)
		#x or y can be within 3 meters

		return (x, y)

	#convert position in meter to map grid id, return grid_x, grid_y and their 1d grid_id
	def pos_to_grid(self,poseX,poseY):
		grid_i = int((poseX - self.map.info.origin.position.x) / self.map_res)
		grid_j = int((poseY - self.map.info.origin.position.y) / self.map_res)

		grid_id = grid_j * self.map_width + grid_i

		return grid_i, grid_j, grid_id

	#straight line collision detection, all inputs unit are in meter
	def collisionDetect(self,x1,y1,x2,y2):
		grid_i1, grid_j1, grid_id1 = self.pos_to_grid(x1, y1)
		grid_i2, grid_j2, grid_id2 = self.pos_to_grid(x2, y2)

		#print ("Check line between: (%d, %d) and (%d, %d)" % (grid_i1, grid_j1, grid_i2, grid_j2))
		line = get_line(grid_i1, grid_j1, grid_i2, grid_j2)
		#print(line)

		for k in range(0,len(line)):
			#print("Check map gird: " + str(line[k][0]) + " " + str(line[k][1]))
			#print("Sise of map:" + str(len(self.map.data)))
			#print(self.map.data[line[k][1] * self.map_width + line[k][0]])
			
			#Map value 0 - 100
			if(self.map.data[line[k][1] * self.map_width + line[k][0]]!=0):
				return False

		return True

	

#naive line drawing function to use instead of bresenham
def naiveDrawLine(x1, y1, x2, y2):
	line=[]
	deltax = x2-x1
	deltay = y2-y1
	if deltay >0:
		stepy = 1
	else: 
		stepy = -1

	if deltax >0:
		stepx = 1
	else:
		 stepx = -1
	if x2-x1 == 0:
		for y in range(y1, y2, stepy):
			line.append([x1,y])
	else:
		m = float(y2 - y1)/float(x2 - x1)
		#print("m =", str(m))
		for x in range(x1, x2, stepx):    
			# Assuming that the round function finds
			# losest integer to a given float.
			y = int(round(m*x - m*x1)+y1)    
			line.append([x,y])
	return line

#Bresenham Line alogirthm from http://www.roguebasin.com/index.php?title=Bresenham%27s_Line_Algorithm#Python
def get_line(x1, y1, x2, y2):
    """Bresenham's Line Algorithm
    Produces a list of tuples from start and end
 
    >>> points1 = get_line((0, 0), (3, 4))
    >>> points2 = get_line((3, 4), (0, 0))
    >>> assert(set(points1) == set(points2))
    >>> print points1
    [(0, 0), (1, 1), (1, 2), (2, 3), (3, 4)]
    >>> print points2
    [(3, 4), (2, 3), (1, 2), (1, 1), (0, 0)]
    """
    # Setup initial conditions
    dx = x2 - x1
    dy = y2 - y1
 
    # Determine how steep the line is
    is_steep = abs(dy) > abs(dx)
 
    # Rotate line
    if is_steep:
        x1, y1 = y1, x1
        x2, y2 = y2, x2
 
    # Swap start and end points if necessary and store swap state
    swapped = False
    if x1 > x2:
        x1, x2 = x2, x1
        y1, y2 = y2, y1
        swapped = True
 
    # Recalculate differentials
    dx = x2 - x1
    dy = y2 - y1
 
    # Calculate error
    error = int(dx / 2.0)
    ystep = 1 if y1 < y2 else -1
 
    # Iterate over bounding box generating points between start and end
    y = y1
    points = []
    for x in range(x1, x2 + 1):
        coord = (y, x) if is_steep else (x, y)
        points.append(coord)
        error -= abs(dy)
        if error < 0:
            y += ystep
            error += dx
 
    # Reverse the list if the coordinates were swapped
    if swapped:
        points.reverse()
    return points  


# bresenham alg for line generation, adapted from https://www.geeksforgeeks.org/bresenhams-line-generation-algorithm/  
def bresenham(x1,y1,x2,y2):
	line=[]

	m_new = 2 * (y2 - y1)  
	slope_error_new = m_new - (x2 - x1) 

	y=y1

	deltax = x2-x1
	deltay = y2-y1
	if deltay >0:
		stepy = 1
	else: 
		stepy = -1

	if deltax >0:
		stepx = 1
	else:
		 stepx = -1

	for x in range(x1,x2+1, stepx):
		line.append([x,y])
		#print("(",x ,",",y ,")\n")  
		# Add slope to increment angle formed  
		slope_error_new =slope_error_new + m_new  

		# Slope error reached limit, time to  
		# increment y and update slope error.  
		if (slope_error_new >= 0.5):  
			y=y+stepy
			slope_error_new =slope_error_new - 1 # 2 * (x2 - x1)

	return line


if __name__ == '__main__':
	try:
		planning = prm_planning()
		

	except rospy.ROSInterruptException:
		pass