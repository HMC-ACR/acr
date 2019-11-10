import rospy
import rospkg
import serial
import tf
import numpy as np
from geometry_msgs.msg import Pose 
from nav_msgs.msgs import OccupancyGrid, Odometry, Path


class PathPlanner:

    def __init__(self):

        # initialize goal, odometry and map
        self.goal = Pose()
        self.odom = Odometry()
        self.map = OccupancyGrid()

        # 
        self.width = 0
        self.height = 0
        self.resolution = 0

        # Initialize path.
        self.start_pose = Pose()
        self.prm_plan = Path()
        self.roadmap = []

        # ROS
        rospy.init_node('pathPlanning', anonymous=True)
        # Assuming we get map data from a map class
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.map_sub = rospy.Subscriber(
            '/map', OccupancyGrid, self.map_callback)
        self.goal_sub = rospy.Subscriber('/goal', Pose, self.goal_callback)
        self.path_pub = rospy.Publisher('/path', Path, queue_size=10) 
        
        self.rate = rospy.Rate(10) #10 Hz

        while not rospy.is_shutdown():
            self.path_pub()
            self.rate.sleep()
    
    def goal_callback(self, Goal):
        self.goal = Goal

        # Make sure goal and start_pose are in bounds of map.
        self.goal.position.x = min(self.goal.position.x, self.width*self.resolution*0.99)
        self.goal.position.x = max(
            self.goal.position.x, 0)
        self.goal.position.y = min(
            self.goal.position.y, self.height*self.resolution*0.99)
        self.goal.position.y = max(
            self.goal.position.y, 0)
    
        self.start_pose.position.x = min(
            self.start_pose.position.x, self.width*self.resolution*0.99)
        self.start_pose.position.x = max(
            self.start_pose.position.x, 0)
        self.start_pose.position.y = min(
            self.start_pose.position.y, self.height*self.resolution*0.99)
        self.start_pose.position.y = max(
            self.start_pose.position.y, 0)

        # Convert positions to grid indices.
        self.goal_i, self.goal_j, self.goal_index = self.pos_to_grid(
            self.goal.position.x, self.goal.position.y)
        self.start_i, self.start_j, self.start_index = self.pos_to_grid(
            self.start.position.x, self.start.position.y)
        
        self.plan_path()

    def odom_callback(self, Odom):
        self.odom = Odom
        self.start_pose = odom
    
    def map_callback(self, Map):
        self.map = Map
        self.width = self.map.info.width
        self.height = self.map.info.height
        self.resolution = self.map.info.resolution

    def pos_to_grid(self, x, y):
        """ Convert a position to a point on the occupancy grid.
        """
        grid_i = int(x/self.map_res)
        grid_j = int(y/self.map_res)
        
        # Due to row-major ordering of cells.
        grid_index = (grid_j * self.width) + grid_i

        return grid_i, grid_j, grid_index

    def plan_path(self):
        """ Updates the path plan based on goal / current positions.
            Uses RRT.
        """
        #create goal node
		self.goal_node.x = self.goal_x
		self.goal_node.y = self.goal_y
		self.goal_node.index = 1
    # 3. Randomly Select New Node c to expand
    # 4. Randomly Generate new Node c’ from c
    # 5. If edge e from c to c’ is collision-free
    # 6.    Add(c’, e) to R
    # 7. If c’ belongs to endgame region, return path
    # 8. Return if stopping criteria is met

        while not self.no_collision(last_node.x, last_node.y, self.goal):
            numiter += 1
            # Generate random target location for node.
            new_node_coordinates = self.generate_node()

            #Find nearest existing node to target using nearest vertex
            #Loop through array, and calculate distances from each of the nodes
            minDistance = 2*(self.width * self.resolution +
                             self.height * self.resolution)
            minDistance_node = None
            for node in self.nodes:
                            current_distance = math.sqrt(
                            (last_node_coordinates[0] - node.x)**2 + (last_node_coordinates[1] - node.y)**2)
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

            if self.collisionDetect(minDistance_node.x, minDistance_node.y, new_node_x, new_node_y):
    				#Add to tree and to array
				new_node = PRM_Node(new_node_x, new_node_y, minDistance_node)
				minDistance_node.addChild(new_node)
				self.nodes.append(new_node)
				last_node = new_node        

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
				if index1 < index2:
					path = path[:index1+1]+path[index2:]
				elif index1 > index2:
					path = path[:index2+1]+path[index1:] 
		#Otherwise, indexes equal, do nothing
					
		#Convert nodes to poses
		for node in path :
			pose = Pose()
			pose.position.x = node.x
			pose.position.y = node.y
			#add node to PRM path
			self.prm_plan.poses.append(node_pose)

    def distance(self, x1, y1, x2, y2):
    		dist = np.sqrt((x2-x1)**2 + (y2-y1)**2)
		return dist

    def generate_node(self):
        """ Generate a random new node to expand.
        """
        x = random.uniform(0,self.width * self.resolution*0.99)
        y = random.uniform(0, self.height * self.resolution*0.99)

        return x, y
    
    def no_collision(self,x1,y1,x2,y2):
        """ Detect if travelling on a straight line between 2 points will cause a collision.
            Returns: True if there is no collision
        """
    	grid_i1, grid_j1, grid_id1 = self.pos_to_grid(x1, y1)
		grid_i2, grid_j2, grid_id2 = self.pos_to_grid(x2, y2)
        line = get_line(grid_i1, grid_j1, grid_i2, grid_j2)

		for k in range(0,len(line)):
			# Check if any points on the line collide with an obstacle
			if (self.map.data[line[k][1] * self.map_width + line[k][0]] != 0):
				return False
		return True

if __name__ == '__main__':
    try:
        planner = PathPlanner()
    except rospy.ROSInterruptException:
        pass
