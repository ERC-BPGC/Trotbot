import rospy
import math
import numpy

from geometry_msgs import Polygon, Point32
from sensor_msgs.msg import LaserScan
from stad_msgs import float32

#IMPORT CUSTOM MESSAGES HERE (PointArray and PolygonArray)

def callback_laserscan(msg):
	"""

	"""
	#Split obstacles
	obstacles_1D = split_array(msg.ranges, 0.5)	#Tuning required

	#Expand Obstacles
	obstacle_list = expand(obstacles_1D, 0.35)	#Tuning required

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
	current_obstacle_index = 0;
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

	Calls: rtheta_to_xy()

	Returns:
		obstacles: Array of geometry_msgs.Polygon

	"""
	obstacle_list = []
	obstacles = PolygonArray()

	#Create tuples of ranges with their angles
	angle_inc = 


	#For each 1D obstacle, create a new list
	#Fill the new list by expanding on one side
	#Append the list by expanding on the other side in reverse order
	

	for obstacle in obstacles_1D:
		point_list = []
		for p in obstacle:
			point_list.append(max(0, p - expand_dis))
		point_list.append(obstacle[-1])
		for p in reversed(obstacle):
			point_list.append(p + expand_dis)
		point_list.append(obstacle[0])

		#Call rtheta_to_xy for each point in the list
		point_list = [rtheta_to_xy(p) for p in point_list]

		#Convert new list to geometry_msgs Polygons
		polygon = PointArray()
		polygon.points = []
		for p in point_list:
			vertex = Point32()
			vertex.x=p[0]
			vertex.y=p[1]
			vertex.z=0
			polygon.points.append(vertex)

		obstacle_list.append(polygon)

	obstacles.polygons = obstacle_list

	return obstacles


def rtheta_to_xy(ranges):
	"""
	Args:
		ranges: list of ranges

	Returns:
		points: list of geometry_msgs.Point32's
	"""
	return points

#-------------------------------

if __name__ == "__main__":

	rospy.init_node("obstacle_detection", anonymous=True)

	
	obstacles_pub = rospy.Publisher("obstacles", PolygonArray ,queue_size=5)
	lidar_sub = rospy.Subscriber("scan", LaserScan, callback_laserscan)

	rospy.spin()