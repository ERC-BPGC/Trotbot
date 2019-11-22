import rospy
import math
import numpy

from geometry_msgs import Polygon, Point32
from sensor_msgs.msg import LaserScan
from std_msgs.msg import float32

#IMPORT CUSTOM MESSAGES HERE (PointArray and PolygonArray)
from navigation.msg import PointArray, PolygonArray

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
	obstacle_list = []
	obstacles = PolygonArray()

	#Create tuples of ranges with their angles
	least_angle = 2*math.pi/len(obstacles_1D)

	for i in range(len(obstacles_1D)):
		obstacles_1D[i] = (obstacles_1D[i], angle_deviation + i*least_angle)


	#For each 1D obstacle, create a new list
	#Fill the new list by expanding on one side
	#Append the list by expanding on the other side in reverse order
	

	for obstacle in obstacles_1D:
		point_list = []
		for p in obstacle:
			point_list.append(max(0, p[0] - expand_dis),p[1])
		point_list.append((obstacle[-1][0] , obstacle[-1][1]+3*least_angle))
		for p in reversed(obstacle):
			point_list.append((p[0] + expand_dis), p[1])
		point_list.append((obstacle[0][0] , obstacle[0][1]-3*least_angle))

		#Call rtheta_to_xy for each point in the list
		point_list = [rtheta_to_xy(p) for p in point_list]

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
	point.z=0

	return point

#-------------------------------

if __name__ == "__main__":

	rospy.init_node("obstacle_detection", anonymous=True)

	
	obstacles_pub = rospy.Publisher("obstacles", PolygonArray ,queue_size=5)
	lidar_sub = rospy.Subscriber("scan", LaserScan, callback_laserscan)

	rospy.spin()