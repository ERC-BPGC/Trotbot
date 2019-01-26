__author__ = 'Atman'
#This code is to integrate
#dynamic checker along with
#RRT path finder

#The dynamic_1.py code will
#subscribe to the obstacle polygon data

#import dynamic_1
import rrt_st_path_atamn as rrtst
import time

from shapely.geometry import Point
from shapely.geometry import Polygon
from shapely.geometry import LineString

from descartes import PolygonPatch
import matplotlib.pyplot as plt
import numpy as np
import random

# Function to check
# if the path between
# any two path_free points
# intersects an obstacle or not
def path_obstacle_free(points , obstacles):
	path = [Point(p[0] , p[1]) for p in points]
	for point_id in range(1 , len(path)):
		dPath = LineString([(path[point_id].x , path[point_id].y) , (path[point_id - 1].x , path[point_id - 1].y)])
		for o in obstacles:
			if dPath.intersects(o):
				return False
	return True

#points = dynamic_1.pointPath

# start, end -> tuples
#path -> list of tuples
#obstacle_list -> list of tuples
def dynamic_rrt(start , end , path , obstacle_list):
#	path = [(p[0] , p[1]) for p in path]
	if not path_obstacle_free(path , obstacle_list):
		print "obstacle"
		path = rrtst.do_RRT(show_animation = False , start_point_coors = list(start) , end_point_coors = list(end) , obstacleList2 = obstacle_list)
	return path
