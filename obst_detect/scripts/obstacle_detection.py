#!/usr/bin/env python

import rospy
import math
import numpy
import tf, tf2_ros, tf2_geometry_msgs

from geometry_msgs.msg import Polygon, Point32, PointStamped, TransformStamped, Vector3, Point
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, Header, ColorRGBA
from visualization_msgs.msg import Marker

from planner_msgs.msg import PointArray, PolyArray


def callback_laserscan(msg):
	"""
		scan callback function
		Args:
			msg: LaserScan data
	"""
	global scanLen 

	#Split obstacles
	obstacles_1D = split_array(msg.ranges, 0.25)	#Tuning required
	scanLen = len(msg.ranges)

	#Expand Obstacles
	obstacle_list = expand(obstacles_1D, 0.35)	#Tuning required

	#Visualise
	visualizeObst(obstacle_list)

	#Publish
	obstacles_pub.publish(obstacle_list)
	


def split_array(ranges, threshold):
	"""
	Args: 
		ranges: Output from laserscan
		threshold: Threshold for deciding when to start a new obstacle

	Returns: 
		obstacles_1D: List of list of ranges
	"""
	obstacles_1D = [[ranges[0]]]
	current_obstacle_index = 0
	for i in range(len(ranges)-1):
		if abs(ranges[i] - ranges[i+1]) > threshold:
			obstacles_1D.append([])
			current_obstacle_index +=1
		obstacles_1D[current_obstacle_index].append(ranges[i+1])


	return obstacles_1D



def expand(obstacles_1D, expand_dis, angle_deviation=0):
	"""
	Args:
		obstacles_1D: List of lists of ranges in every obstacle
		expand_dis: Expansion magnitude
		angle_deviation: Deviation of the zero ofthe lidar from zero of the bot
		(in radians)

	Calls: rtheta_to_xy()

	Returns:
		obstacles: Array of geometry_msgs.Polygon

	"""
	global scanLen
	obstacle_list = []
	obstacles = PolyArray()

	#Create tuples of ranges with their angles
	# least_angle = 2*math.pi/len(obstacles_1D)
	least_angle = 2*math.pi/scanLen
	pt_count = 0

	for i in range(len(obstacles_1D)):
		# obstacles_1D[i] = (obstacles_1D[i], angle_deviation + i*least_angle)
		for j in range(len(obstacles_1D[i])):
			obstacles_1D[i][j] = (obstacles_1D[i][j], angle_deviation + pt_count*least_angle)
			pt_count = pt_count + 1

	# print(obstacles_1D[5])
	# print(obstacles_1D)

	#For each 1D obstacle, create a new list
	#Fill the new list by expanding on one side
	#Append the list by expanding on the other side in reverse order

	for obstacle in obstacles_1D:
		point_list = []

		for point in obstacle:
			point_list.append((max(0,point[0] - expand_dis),point[1]))
		point_list.append((obstacle[-1][0] , obstacle[-1][1]+2*least_angle))

		for point in reversed(obstacle):
			point_list.append(((point[0] + expand_dis), point[1]))
		point_list.append((obstacle[0][0] , obstacle[0][1]-3*least_angle))

		new_point_list = []
		for pt in point_list:
			if not math.isinf(pt[0]):
				new_point_list.append(pt)

		point_list = [rtheta_to_xy(p) for p in new_point_list]

		#Convert new list to geometry_msgs Polygons
		polygon = PointArray()
		polygon.points = point_list

		obstacle_list.append(polygon)

	obstacles.polygons = obstacle_list

	return obstacles


def rtheta_to_xy(point_rtheta):
	"""
	Args:
		point_rtheta: point in R-Theta form(range, angle)

	Returns:
		point: geometry_msgs.Point32
	"""
	

	point = Point32()
	point.x = point_rtheta[0]*math.cos(point_rtheta[1])
	point.y = point_rtheta[0]*math.sin(point_rtheta[1])
	point.z = 0
	
	return point

def visualizeObst(obstacles):
	"""
        Helper function to visualise scan points in rviz
        http://wiki.ros.org/rviz/Tutorials/Markers%3A%20Points%20and%20Lines
        https://github.com/cse481sp17/cse481c/wiki/Lab-12:-Creating-Custom-Visualizations-in-RViz-using-Markers
    """
	
	marker = Marker(type=Marker.POINTS)
	marker.scale = Vector3(0.05, 0.05, 0.05)
	marker.lifetime = rospy.Duration(5)
	marker.header = Header(frame_id="base_link", stamp=rospy.Time.now())
	marker.color = ColorRGBA(0, 0, 1, 0.75)

	for poly in obstacles.polygons:
		for point in poly.points:
			marker.points.append(Point(point.x, point.y, point.z))
	
	marker_pub.publish(marker)


#-------------------------------

if __name__ == "__main__":

	rospy.init_node("obstacle_detection")

	scanLen = 0

	obstacles_pub = rospy.Publisher("obstacles", PolyArray ,queue_size=5)
	lidar_sub = rospy.Subscriber("/scan", LaserScan, callback_laserscan)
	marker_pub = rospy.Publisher("markers", Marker, queue_size=10)

	rospy.loginfo("Obstacle detection initiated")
	rospy.spin()