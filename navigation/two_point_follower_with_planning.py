#! /usr/bin/env python
import sys
import rospy
import tf
import math
import numpy
from geometry_msgs.msg import *
from std_msgs.msg import *
from nav_msgs.msg import *
import heapq
param = {
		'start':'',
		'kp':'100',
		'vel_cmd':''
		}

default = {
			'start':'True',
			'kp':'100',
			'vel_cmd':'0.2'
			}
def initialize_parameters():
	rospy.set_param('/trajectory_follower_start', default['start'])
	param['start'] = default['start']

	if rospy.has_param('/trajectory_follower_kp'):
		param['kp'] = rospy.get_param('/trajectory_follower_kp')
	else:
		rospy.set_param('/trajectory_follower_kp', default['kp'])
		param['kp'] = default['kp']

	if rospy.has_param('/trajectory_follower_vel_cmd'):
		param['vel_cmd'] = rospy.get_param('/trajectory_follower_vel_cmd')
	else:
		rospy.set_param('/trajectory_follower_vel_cmd', default['vel_cmd'])
		param['vel_cmd'] = default['vel_cmd']

class TrajectoryFollower():
	"""docstring for TrajectoryFollower"""
	def __init__(self):
		self.flag=0
		self.z = 'a'
		self.y = 'a'
		self.x = 'a'
		self.path1 = []
		self.target_point=[]
		self.robot_x=0.0
		self.robot_y=0.0
		self.robot_yaw = 0.0
		self.initialize_variables()
		self.initialize_pubs_subs()

		rospy.Timer(rospy.Duration(0.05), self.controller)

	def update_params(self):
		if rospy.has_param('/trajectory_follower_start'):
			param['start'] = rospy.get_param('/trajectory_follower_start')

		if rospy.has_param('/trajectory_follower_kp'):
			param['kp'] = rospy.get_param('/trajectory_follower_kp')
		
		if rospy.has_param('/trajectory_follower_vel_cmd'):
			param['vel_cmd'] = rospy.get_param('/trajectory_follower_vel_cmd')
		

	def initialize_variables(self):
		coordinates = { 'a':{'x':0,'y':0}, 'b':{'x':0,'y':1}, 'c':{'x':0,'y':2}, 'd':{'x':1,'y':2}, 'e':{'x':1,'y':1}, 'f':{'x':2,'y':2}, 'g':{'x':2,'y':4}, 'h':{'x':0,'y':3}, 'i':{'x':1,'y':4}}
		self.robot_x = self.robot_x
		self.robot_y = self.robot_y
		self.robot_yaw = self.robot_yaw
		if(self.flag == 0):
			self.z = self.y
			self.x = raw_input("enter the pickup point: ")
			self.y = raw_input("enter dropping point: ")
			self.flag = 1
			self.path1 = path_plan(self.x, self.y)
			c = len(self.path1)
			if((self.robot_x >= (coordinates[self.path1[c-1]]['x']-0.1) and self.robot_x <= (coordinates[self.path1[c-1]]['x']+0.1)) and (self.robot_y >= coordinates[self.path1[c-1]]['y']-0.1 and self.robot_y <= coordinates[self.path1[c-1]]['y']+0.1)):
				self.path3 = self.path1
			else:
				self.path2 = path_plan(self.z, self.x)
				print "path2", self.path2
				del (self.path2)[0]
				self.path3 = (self.path1)+(self.path2)
		print "path3", self.path3
		b = len(self.path3)
		a=b-1
		for i in range(b):
			if((self.robot_x >= (coordinates[self.path3[0]]['x']-0.1) and self.robot_x <= (coordinates[self.path3[0]]['x']+0.1)) and (self.robot_y >= coordinates[self.path3[0]]['y']-0.1 and self.robot_y <= coordinates[self.path3[0]]['y']+0.1)):
				print "Final Target Reached"
				self.flag=0
				break
			elif((self.robot_x >= (coordinates[self.path3[a]]['x']-0.1) and self.robot_x <= (coordinates[self.path3[a]]['x']+0.1)) and (self.robot_y >= coordinates[self.path3[a]]['y']-0.1 and self.robot_y <= coordinates[self.path3[a]]['y']+0.1)):
				self.target_point = [coordinates[self.path3[a-1]]['x'],coordinates[self.path3[a-1]]['y'],0.0] #[x,y,yaw]
				break
			a=a-1
		print self.target_point


	def odom_data_callback(self, data):
		self.robot_x = data.pose.pose.position.x
		self.robot_y = data.pose.pose.position.y

		self.robot_orientation = tf.transformations.euler_from_quaternion([data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w])
		self.robot_yaw = self.robot_orientation[2]

	def initialize_pubs_subs(self):
		self.odom_subscriber = rospy.Subscriber('/odom', Odometry, self.odom_data_callback)
		self.vel_cmd_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
		
	def controller(self, event):
		# print "in controller"
		self.update_params()
		# print param['start']
		
		if param['start'] == 'True':
			# print "controller started"
			target_point_rel_x = self.target_point[0] - self.robot_x
			target_point_rel_y = self.target_point[1] - self.robot_y
			self.target_dir = (numpy.arctan2(target_point_rel_y, target_point_rel_x))
			self.delta_angle = self.target_dir - self.robot_yaw
			self.target_distance = math.sqrt((self.target_point[0] - self.robot_x)**2 + (self.target_point[1] - self.robot_y)**2)

			self.kp = float(param['kp'])

			if(((self.target_dir - self.robot_yaw)<=0.1 or (self.target_dir - self.robot_yaw)>=-0.1) and (self.target_distance)<=0.03):
					self.omega = 0.0
					self.vel_cmd = 0.0
					self.sp = Twist()
					self.sp.linear.x = self.vel_cmd
					self.sp.angular.z = self.omega
					self.vel_cmd_publisher.publish(self.sp)
					print("Target Reached")
					self.initialize_variables()
			elif((self.target_dir - self.robot_yaw)>=0.2 or (self.target_dir - self.robot_yaw)<=-0.2 and (self.target_distance)>=0.05):
				self.omega = 0.5*(self.target_dir - self.robot_yaw)
				self.vel_cmd = 0.0

			elif((self.target_dir - self.robot_yaw)<=0.2 or (self.target_dir - self.robot_yaw)>=-0.2 and (self.target_distance)>=0.05):
				self.omega = 0.4*(self.target_dir - self.robot_yaw)
				self.vel_cmd = 0.1*(self.target_distance)

			elif(self.target_distance>=0.2):
				self.omega = 0.0
				self.vel_cmd = 2*(self.target_distance)
			elif(self.target_distance<=0.2):
				while(self.target_distance>=0.02):
					self.omega = 0.0
					self.vel_cmd = 75*(self.target_distance)
					self.sp = Twist()
					self.sp.linear.x = self.vel_cmd
					self.sp.angular.z = self.omega
					self.vel_cmd_publisher.publish(self.sp)
					print "yo"
					print "vel, omega, dtheta,target_distance:", self.vel_cmd, self.omega, self.delta_angle, self.target_distance
					print "yaw, dir:", self.robot_yaw, self.target_dir
					print "x, y:", self.robot_x, self.robot_y
					print "bf:", target_point_rel_x, target_point_rel_y
					target_point_rel_x = self.target_point[0] - self.robot_x
					target_point_rel_y = self.target_point[1] - self.robot_y
					self.target_dir = (numpy.arctan2(target_point_rel_y, target_point_rel_x))
					self.delta_angle = self.target_dir - self.robot_yaw
					self.target_distance = math.sqrt((self.target_point[0] - self.robot_x)**2 + (self.target_point[1] - self.robot_y)**2)
			
			

			self.sp = Twist()
			self.sp.linear.x = self.vel_cmd
			self.sp.angular.z = self.omega

			self.vel_cmd_publisher.publish(self.sp)
			print "yo"
			print "vel, omega, dtheta,target_distance:", self.vel_cmd, self.omega, self.delta_angle, self.target_distance
			print "yaw, dir:", self.robot_yaw, self.target_dir
			print "x, y:", self.robot_x, self.robot_y
			print "bf:", target_point_rel_x, target_point_rel_y

class Vertex:
    def __init__(self, node):
        self.id = node
        self.adjacent = {}
        # Set distance to infinity for all nodes
        self.distance = sys.maxint
        # Mark all nodes unvisited        
        self.visited = False  
        # Predecessor
        self.previous = None

    def add_neighbor(self, neighbor, weight=0):
        self.adjacent[neighbor] = weight

    def get_connections(self):
        return self.adjacent.keys()  

    def get_id(self):
        return self.id

    def get_weight(self, neighbor):
        return self.adjacent[neighbor]

    def set_distance(self, dist):
        self.distance = dist

    def get_distance(self):
        return self.distance

    def set_previous(self, prev):
        self.previous = prev

    def set_visited(self):
        self.visited = True

    def __str__(self):
        return str(self.id) + ' adjacent: ' + str([x.id for x in self.adjacent])

class Graph:
    def __init__(self):
        self.vert_dict = {}
        self.num_vertices = 0

    def __iter__(self):
        return iter(self.vert_dict.values())

    def add_vertex(self, node):
        self.num_vertices = self.num_vertices + 1
        new_vertex = Vertex(node)
        self.vert_dict[node] = new_vertex
        return new_vertex

    def get_vertex(self, n):
        if n in self.vert_dict:
            return self.vert_dict[n]
        else:
            return None

    def add_edge(self, frm, to, cost = 0):
        if frm not in self.vert_dict:
            self.add_vertex(frm)
        if to not in self.vert_dict:
            self.add_vertex(to)

        self.vert_dict[frm].add_neighbor(self.vert_dict[to], cost)
        self.vert_dict[to].add_neighbor(self.vert_dict[frm], cost)

    def get_vertices(self):
        return self.vert_dict.keys()

    def set_previous(self, current):
        self.previous = current

    def get_previous(self, current):
        return self.previous

def shortest(v, path):
    ''' make shortest path from v.previous'''
    if v.previous:
        path.append(v.previous.get_id())
        shortest(v.previous, path)
    return

def dijkstra(aGraph, start, target):
    print '''Dijkstra's shortest path'''
    # Set the distance for the start node to zero 
    start.set_distance(0)

    # Put tuple pair into the priority queue
    unvisited_queue = [(v.get_distance(),v) for v in aGraph]
    heapq.heapify(unvisited_queue)

    while len(unvisited_queue):
        # Pops a vertex with the smallest distance 
        uv = heapq.heappop(unvisited_queue)
        current = uv[1]
        current.set_visited()

        #for next in v.adjacent:
        for next in current.adjacent:
            # if visited, skip
            if next.visited:
                continue
            new_dist = current.get_distance() + current.get_weight(next)
            
            if new_dist < next.get_distance():
                next.set_distance(new_dist)
                next.set_previous(current)
                print 'updated : current = %s next = %s new_dist = %s' \
                        %(current.get_id(), next.get_id(), next.get_distance())
            else:
                print 'not updated : current = %s next = %s new_dist = %s' \
                        %(current.get_id(), next.get_id(), next.get_distance())

        # Rebuild heap
        # 1. Pop every item
        while len(unvisited_queue):
            heapq.heappop(unvisited_queue)
        # 2. Put all vertices not visited into the queue
        unvisited_queue = [(v.get_distance(),v) for v in aGraph if not v.visited]
        heapq.heapify(unvisited_queue)
def path_plan(a,b):
	g = Graph()
	g.add_vertex('a')
	g.add_vertex('b')
	g.add_vertex('c')
	g.add_vertex('d')
	g.add_vertex('e')
	g.add_vertex('f')
	g.add_vertex('g')
	g.add_vertex('h')
	g.add_vertex('i')

	g.add_edge('a', 'b', 1)  
	g.add_edge('b', 'c', 1)
	g.add_edge('c', 'h', 1)
	g.add_edge('h', 'i', 1.4142135)
	g.add_edge('b', 'd', 1.4142135)
	g.add_edge('c', 'd', 1)
	g.add_edge('h', 'd', 1.4142135)
	g.add_edge('d', 'i', 2)
	g.add_edge('d', 'f', 1)
	g.add_edge('i', 'f', 2.236068)
	g.add_edge('f', 'g', 2)
	g.add_edge('d', 'e', 1)
	g.add_edge('f', 'e', 1.4142135)

	dijkstra(g, g.get_vertex(a), g.get_vertex(b)) 
	target = g.get_vertex(b)
	path = [target.get_id()]
	shortest(target, path)
	return path

def main():
	rospy.init_node('trajectory_follower', anonymous=True)
	
	print "[TrajectoryFollower: ] Started trajectory_follower node"
	followe_obj = TrajectoryFollower()
	print "[TrajectoryFollower: ] obj created"
	initialize_parameters()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print "[TrajectoryFollower: ] Shutting down trajectory_follower node"


if __name__ == "__main__":
	main()