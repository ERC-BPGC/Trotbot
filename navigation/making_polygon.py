#!/usr/bin/env python

import rospy
import numpy
import math
from geometry_msgs.msg import Polygon, Point32, PolygonStamped
from sensor_msgs.msg import LaserScan

from laser_geometry import LaserProjection
from sensor_msgs.msg import PointCloud2 as pcl                 #for point cloud conversion

from sensor_msgs import point_cloud2

# from shapely.geometry import Point, Polygon                       #what we can do is we can use one node for rrt, one for
																#obstacle expansion, and one for making obstacles


def callback1(data):
	# print("in callback1")
	global laser1
	laser1 = data

	global array1
	array1 = list(data.ranges)

	# print(array1)

def callback2(data):
	# print("in callback2")
	global laser2
	laser2 = data

	global array2
	array2 = list(data.ranges)
	# print(array1)
	compare_val(array1,array2)

def compare_val(arr_1,arr_2):
	# arr_1 = array1[:]                             
	# arr_2 = array2[:]

	obs_list = []                             # contains list of points which are same objects
	
	for i in range(len(arr_1)-1):
		empty_list = []                               # make a circular array
		
		#s = arr_1[i] - arr_2[i]                       # compare 2 - 1
		# print(s)              
		increment = 0.400000095367
		j = 0
		# print("arr2",arr_2)
		# print("arr1",arr_1)
		if arr_1[i] == "nan" or arr_2[i] == "nan":
			continue
		# print(len(arr_1) == len(arr_2))
		# print(str(arr_2[i]-arr_1[i+1])[2])
		# print(str(arr_2[i+1]-arr_1[i])[2])

		# laser1 = LaserScan()
		laser1.header = "laser1"
		# laser2 = LaserScan()
		laser2.header = "laser2"
		x1 = []
		x2 = []

		for l1 in range(len(arr_2)):
			x1.append(0)  
			x2.append(0)                               #making range entries equal to 0
		laser1.ranges = x1[:]
		laser2.ranges = x1[:]

		while (str(arr_2[i]-arr_1[i+1])[2]) == '3' and (str(arr_2[i+1]-arr_1[i])[2]) == '4':
			# print("in loop") 
			
			# empty_list.append(arr_1[i])       #this just makes list
			# empty_list.append(arr_2[i])

			x1[i] = arr_1[i]
			x2[i] = arr_2[i]
			

			i = i + 1

			if i-2 > range(len(arr_1)):
				break
			if arr_1[i] == "nan" or arr_2[i] == 'nan':
				i = i + 1

		# if len(empty_list) != 0:
		# 	obs_list.append(empty_list)
		
		laser1.ranges = x1[:]
		laser2.ranges = x2[:]

		# print(x1)
		# print(x2)

		# print(laser1)
		# print(laser2)

		laserproj1 = LaserProjection()
		laserproj2 = LaserProjection()
		cloud1 = laserproj1.projectLaser(laser1)
		cloud2 = laserproj2.projectLaser(laser2)

		# print(cloud1)
		# print(cloud2)

		xyz_array1 = list(point_cloud2.read_points(cloud1, skip_nans=True, field_names = ("x", "y", "z")))
		xyz_array2 = list(point_cloud2.read_points(cloud2, skip_nans=True, field_names = ("x", "y", "z")))

		if len(xyz_array1) != 0:
			print("1 ")
			print(xyz_array1)

##	
	# print("obs_list")
	# print(obs_list)

	rate = rospy.Rate(1000)
	rate.sleep()

	


def main():
	# global array1
	# array1 = []
	# global array2
	# array2 = []

	rospy.init_node("obstacles_make")
	sub1 = rospy.Subscriber("/tp1", LaserScan, callback1)
	sub2 = rospy.Subscriber("/tp3", LaserScan, callback2)
	# compare_val(array1,array2)
	rospy.spin()


if __name__ == '__main__':
	main()