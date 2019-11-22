#! /usr/bin/env python

import random , numpy as np
import math
import cmath
from shapely.geometry import Polygon, Point, LineString
from descartes import PolygonPatch
import matplotlib.pyplot as plt

PI = np.pi
THRESHOLD = 0.25
ALPHA = 10

def make_obstacles_scan(scan_list):
	""" 
		Make List of Lines from scan representing different obstacles 

        Args:
            scan_list : List of scan points

        Return:
            line_obstacles : List of obstacles in LineString Format
			pts : Cartesian form (x,y) of scan. List of [(x,y)]
	"""
	#Convert the scan to a numoy arra of points
	pt_ang = np.arange(0,2*np.pi,np.pi/180)
	pt_scan = np.array(scan_list)
	pts = [] #To be returned cartesian form of scan
	pt_x = np.multiply(pt_scan,np.cos(pt_ang))
	pt_y = np.multiply(pt_scan,np.sin(pt_ang))

	for a,b in zip(pt_x,pt_y):
		pts.append((a,b))

	pt_scan = np.array(scan_list)
	#Shifting the scan values
	pt_scan_prev = np.append(pt_scan[1:],pt_scan[0])
	# Taking the absolute difference and comparing with Threshold
	line_obst = abs(pt_scan_prev - pt_scan)>2*THRESHOLD
	ind=np.argwhere(line_obst==True)
	ind  = np.append(0 , ind)
	ind  = np.append(ind, len(scan_list))

	line_obstacles = []
	pt_scan_enum = list(enumerate(scan_list))
	for i in range(len(ind)-1):
		line = [(pt[0] , pt[1]) for pt in pts[ind[i]+1:ind[i+1]+1]]
		line_obstacles.append(line)

	return (line_obstacles , pts)


def adjustable_random_sampler(sample_area, goal, goal_sample_rate):
    """Randomly sample point in area while sampling goal point 
        at a specified rate.

        Args:
            sample_area: area to sample point in (min and max)
            goal: tuple containing goal point coordinates.
            goal_sample_rate: number between 0 and 1 specifying how often 
                                to sample the goal point.
    
        Return:
            Randomly selected point as a tuple.
    """

    if random.random() > goal_sample_rate:
        return (random.uniform(sample_area[0], sample_area[1]), 
                random.uniform(sample_area[0], sample_area[1]))
    else:
        return goal


def scan_obstacle_checker(scan_list , point):
	"""
	Checking Whether the sampled point lies beyond THRESHOLD distance of every point. Experimental

		Args:
			scan_list --> List of scan 
			point --> sampled point

		Returns --> ('nan','nan') if lies in obstacle else point
	"""
	#Point in polar coordinates (rho , phi)
	phi = math.atan2(point[1] , point[0])
	
	if phi<0:
		phi = 2*PI + phi

	rho = math.sqrt(point[0]**2 + point[1]**2)

	# Phi in degrees, used for indexing in LaserScan
	phi_deg = int(math.floor(phi*180/PI))

	#enumerating the list 
	scan_list_enum = list(enumerate(scan_list))

	for obstacle in scan_list_enum:
		# Checking the absolute of vector difference from each coordinate to be greater than THRESHOLD
		if abs(complex(cmath.rect(obstacle[1],obstacle[0]*PI/180) - complex(cmath.rect(rho,phi))))<THRESHOLD:
			return float('nan'),float('nan')
	return point

#############################################################
##################EXPERIMENTAL###############################
	# try:
	# 	ALPHA = int(math.ceil(math.asin(THRESHOLD/rho)))
	# except:
	# 	ALPHA=10
	# 	print(rho)
	# 	# return float('nan'),float('nan')
	# if (ALPHA<=phi_deg<=360 - ALPHA):
	# 	for obstacle in scan_list_enum[phi_deg-ALPHA:phi_deg+ALPHA+1]:
	# 		# print(abs(complex(cmath.rect(obstacle[1],obstacle[0]) - complex(cmath.rect(rho,phi)))))
	# 		if abs(complex(cmath.rect(obstacle[1],obstacle[0]*PI/180) - complex(cmath.rect(rho,phi))))<THRESHOLD:
	# 			# print("nan")
	# 			return float('nan'),float('nan')
	# 	return point

	# elif (360 - ALPHA<=phi_deg):
	# 	for obstacle in scan_list_enum[phi_deg-ALPHA:360]:
	# 		if abs(complex(cmath.rect(obstacle[1],obstacle[0]*PI/180) - complex(cmath.rect(rho,phi))))<THRESHOLD:
	# 			return float('nan'),float('nan')
		
	# 	for obstacle in scan_list_enum[0: ALPHA+2 - (360-phi_deg)]:
	# 		if abs(complex(cmath.rect(obstacle[1],obstacle[0]*PI/180) - complex(cmath.rect(rho,phi))))<THRESHOLD:
	# 			return float('nan'),float('nan')
	# 	return point

	# else:
	# 	# print(phi_deg)
	# 	for obstacle in scan_list_enum[0:phi_deg+ALPHA+2]:
	# 		if abs(complex(cmath.rect(obstacle[1],obstacle[0]*PI/180) - complex(cmath.rect(rho,phi))))<THRESHOLD:				
	# 			return float('nan'),float('nan')

	# 	for obstacle in scan_list_enum[360-phi_deg:]:
	# 		if abs(complex(cmath.rect(obstacle[1],obstacle[0]*PI/180) - complex(cmath.rect(rho,phi))))<THRESHOLD:
	# 			return float('nan'),float('nan')
	# 	return point
		

def check_intersection_scan(point_list , line_obstacles):
	"""Check whether line passes through any Scan obstacle.

	Args:
		point_list: list of points in the line.
		line_obstacles: list of obstacles as LineString.

	Returns:
		boolean specifying whether or not the line intersects
		any of the obstacles. 
	"""
	#Direct path of pointlist
	direct_line = LineString(point_list)
	#Check if the direct_line is at least THRESHOLD distance away from every LineString Obstacles
	for obstacle in line_obstacles:
		if direct_line.distance(LineString(obstacle))<THRESHOLD:
			return True
	return False

def visualize_scan(path, scan_list):
	"""Draw the path along with environment obstacles.

		Args:
			path: list of points in the path as tuples.
			scan_list: LaserScan obstacle

		Returns:
			Nothing. Function is used to visualize path.
	"""
	
	
	pt_ang = np.arange(0,2*np.pi,np.pi/180)
	pt_scan = np.array(scan_list)
	pts = []
	pt_x = np.multiply(pt_scan,np.cos(pt_ang))
	pt_y = np.multiply(pt_scan,np.sin(pt_ang))

	for a,b in zip(pt_x,pt_y):
		pts.append((a,b))

    # Clear the figure
	plt.clf()

    # Plot each point in the path
	plt.plot([x for (x, _) in path], [y for (_, y) in path],'b')
	plt.plot([x for (x, _) in pts], [y for (_, y) in pts],'r.')
	plt.axis((-5,5,-5,5))
    
	plt.show()

def los_optimizer_scan(path , line_obstacles):
	"""Line of Sight Path Optimizer for Scan.

        For each point in the path, it checks if there is a direct
        connection to procceeding points which does not pass through
        any obstacles. By joining such points, number of uneccessary
        points in the path are reduced.

        Args:
            path: list of tuples containing coordinates for a point in path..
            obstacle_list: list of obstacles.

        Returns:
            Optimized path as a list of tuples containing coordinates.
            If path is found to be intersecting with any obstacle and
            there is no lookahead optimization which avoids this, then
            only the path uptill the intersection is returned.
	"""

	# Init optimized path with the start as first point in path.
	optimized_path = [path[0]]

	# Loop through all points in path, checking for LOS shortening
	current_index = 0
	while current_index < len(path) - 1:

		# Keep track of whether index has been updated or not
		index_updated = False

		# Loop from last point in path to the current one, checking if 
		# any direct connection exists.
		for lookahead_index in range(len(path) - 1 , current_index, -1):
			if not check_intersection_scan([path[current_index], path[lookahead_index]], line_obstacles):
				# If direct connection exists then add this lookahead point to optimized
				# path directly and skip to it for next iteration of while loop
				optimized_path.append(path[lookahead_index])
				current_index = lookahead_index
				index_updated = True
				break

		# If index hasnt been updated means that there was no LOS shortening
		# and the edge between current and next point passes through an obstacle.  
		if not index_updated:    
		# In this case we return the path so far  
			return optimized_path

	return optimized_path