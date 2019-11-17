#! /usr/bin/env python
"""
Path planning Code of RRT* with
author: Ojit Mehta(@ojitmehta123)
"""

import math
import os
import sys

import matplotlib.pyplot as plt

sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../rrt_for_scan/")

import random
from shapely.geometry import Polygon, Point, LineString
from descartes import PolygonPatch

from utils import adjustable_random_sampler as sampler

from utils_scan import scan_obstacle_checker , make_obstacles_scan , check_intersection_scan


try:
    from rrt import RRT
except ImportError:
    raise

show_animation = True


def find_near_nodes(node_list, new_node):
	nnode = len(node_list) + 1
	r = 50 * math.sqrt((math.log(nnode) / nnode))
	dist_list = [(node.x - new_node.x) ** 2 +
					(node.y - new_node.y) ** 2 for node in node_list]
	near_inds = [dist_list.index(i) for i in dist_list if i <= r ** 2]
	return near_inds


class Node(object):
	"""
	Coordinate representation in node form.
	x,y --> Coordinates
	Parent node is the node connected to the present node
	"""
	def __init__(self, x, y):
            self.x = x
            self.y = y
            self.path_x = []
            self.path_y = []
            self.parent = None
			self.cost = 0.0

class RRTStar(object):
	"""
	RRT star algorithm
	"""

	def __init__(self, sample_area,
					expand_dis=0.5,
					path_resolution=1.0,
					goal_sample_rate=20,
					max_iter=300,
					connect_circle_dist=50.0
					):
		"""
		start: Start Point. in our case remains(0 , 0) unless specified
		goal: Next goal to be reached
		scan_list = LaserScan polar distances to Obstacles [r1,r2,r3...] initially assuming every scan occurs at 1 rad interval
		randomArea:Random Sampling Area
		"""

		self.start = Node(start[0],start[1])
		self.goal = Node(goal[0],goal[1])
		self.sample_area = sample_area
        self.sampler = sampler
        self.expand_dis = expand_dis
        self.goal_sample_rate = goal_sample_rate
		

	def __call__(self, start_point=[0,0], goal_point, scan , animation=False):
        """Plans path from start to goal avoiding obstacles.

        Args:
            start_point: tuple with start point coordinates.
            end_point: tuple with end point coordinates.
            scan_list: list of obstacles which themselves are list of points
            animation: flag for showing planning visualization (default False)

        Returns:
            A list of points representing the path determined from
            start to goal while avoiding obstacles.
            An list containing just the start point means path could not be planned.
        """

		#Make line obstacles and scan in x,y from scan_list
        line_obstacles , pts = make_obstacles_scan(scan)

		#Setting Start and End
		self.start = Node(start[0],start[1])
		self.goal = Node(goal[0],goal[1])

		#Initialize node with Starting Position
		self.node_list = [self.start]

		#Loop for maximum iterations to get the best possible path
		for i in range(self.max_iter):
			#Sample a Random point in the sample area
			rnd_point = sampler(sample_area , self.goal , self.goal_sample_rate)

			# Find nearest node to the sampled point
			distance_list = [(node.x - rnd_point[0])**2 + (node.y - rnd_point[1])**2 for node in node_list]
			nearest_node = self.node_list[distance_list.index(min(distance_list))]

			#Creating a new Point in the Direction of sampled point
			theta = math.atan2(rnd_point[1] - nearest_node.y, rnd_point[0] - nearest_node.x)  
			new_point = nearest_node.x + self.expand_dis*math.cos(theta), \
						nearest_node.y + self.expand_dis*math.sin(theta)

			#Check obstacle collision
			new_point = scan_obstacle_checker(scan_list , new_point)
			
			if not math.isnan(new_point[0]):
				nearest_indexes = find_near_nodes(self.node_list , new_point , self.cir)

