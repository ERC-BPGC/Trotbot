#! /usr/bin/env python

import rospy
import actionlib
from tf import transformations
import shapely
from shapely.geometry import Point, LineString
import math
import utils
import collections
from navigation.msg import *

# Define some usefull named tuples
Orientation = collections.namedtuple('Orientation', ['roll', 'pitch', 'yaw'])

class Manager():

	def __init__(self):
		self.position = None 
		self.orientation = None
		self.obstacles = []
		self.path = collections.deque()

		# self.odometry_sub = rospy.Subscriber("odom", Odometry, self.odom_update)
		
		self.obstacle_sub = rospy.Subscriber("obstacles", PolyArray, self._obstacle_update)
		
		self.controller_client = actionlib.SimpleActionClient('move_bot' MoveBotAction)
		self.controller_client.wait_for_server()
		
		self.plan_path = rospy.ServiceProxy('rrt_planner_service', Planner)

	def go_to(self, goal):
		self._call_path_planner()
		while len(self.path) is not 0:
			self._move_to_next_point()

		
	def _move_to_next_point(self):
		next_point = self.path[0]
		goal_for_controller = MoveBotGoal()
		goal_for_controller.goal.point = list(next_point)
		self.controller_client.send_goal(goal_for_controller, done_cb=self._next_point_reached)	


	def _next_point_reached(self, done):
		if done:
			self.path.popleft()

	def _odom_update(self, data): 
		self.position = Point(data.pose.pose.position.x, data.pose.pose.position.y)
		self.orientation = Orientation(
			transformations.euler_from_quaternion([
				data.pose.pose.orientation.x, data.pose.pose.orientation.y, 
				data.pose.pose.orientation.z, data.pose.pose.orientation.w
		]))

		self._tranform_path()


	def _tranform_path(self):
		p = LineString(self.path)
		p = shapely.affinity.translate(p, -self.position.x, -self.position.y)
		p = shapely.affinity.rotate(p, angle=math.degrees(self.orientation.yaw), origin=(0, 0))
		self.path = collections.deque(p.coords)

	def _obstacle_update(self, data): 
		self.obstacles = [ [ (point.x, point.y) for point in polygon.points ] for polygon in data.polygons ] 
		if not utils.check_intersection(self.path, self.obstacles):
			self._call_path_planner()

	def _call_path_planner(self, first_plan=False):
		rospy.logwarn("Waiting for Service")
		rospy.wait_for_service("rrt_planner_service")
		
		try:
			response = PlannerResponse()
			request = PlannerRequest()
			
			request.start.point=#[ , ] Set initial points here
			request.goal.point=#[ , ]
			
			#Obstacle to be given in theis format:
			# request.obstacle_list.polygons=[
			# PointArray([ Point_xy([8, 5]), Point_xy([7,8]), Point_xy([2,9]), Point_xy([3,5]) ]),
			# PointArray([ Point_xy([3,3]), Point_xy([3,5]), Point_xy([5,5]), Point_xy([5,3]) ])
			# ]

			response = self.plan_path(request)
			try response.ack:
				self.path = collections.deque([(pt.point[0],pt.point[1]) for pt in response.path.points])
			except Exception as e:
				print("Failed to compute path!")
				print(e)


	    except rospy.ServiceException, e:
	        print "Service call failed: %s"%e


def main():
	rospy.init_node("manager", anonymous=True)