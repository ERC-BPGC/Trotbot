#! /usr/bin/env python

import rospy
import actionlib
from tf import transformations
import shapely
from shapely.geometry import Point, LineString
import math
import utils
import collections
from navigation.msg import PolyArray, PointArray, Point_xy, MoveBotAction, MoveBotGoal, Odometry
from navigation.srv import Planner, PlannerRequest, PlannerResponse


Orientation = collections.namedtuple('Orientation', ['roll', 'pitch', 'yaw'])

class Manager():

	def __init__(self):
		self.position = Point(0, 0) 
		self.orientation = Orientation(0, 0, 0)
		self.current_goal_point = Point(0, 0)
		self.obstacles = []
		self.path = collections.deque()

		self.odometry_sub = rospy.Subscriber("odom", Odometry, self._odom_update)
		self.obstacle_sub = rospy.Subscriber("obstacles", PolyArray, self._obstacle_update)
		
		self.controller_client = actionlib.SimpleActionClient('move_bot', MoveBotAction)
		self.controller_client.wait_for_server()
		
		self.plan_path = rospy.ServiceProxy('rrt_planner_service', Planner)

	def go_to(self, goal_point):
		self.current_goal_point = goal_point
		self._call_path_planner(self.current_goal_point)
		while len(self.path) is not 0:
			self._move_to_next_point()

		
	def _move_to_next_point(self):
		next_point = self.path[0]
		goal_for_controller = MoveBotGoal()
		goal_for_controller.goal.point = list(next_point)
		self.controller_client.send_goal(goal_for_controller, done_cb=self._next_point_reached)
		controller_client.wait_for_result()

	def _next_point_reached(self, _, done):
		if done.ack:
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
			self._call_path_planner(self.current_goal_point)

	def _call_path_planner(self, goal_point):
		rospy.logwarn("Waiting for Service")
		rospy.wait_for_service("rrt_planner_service")
		
		try:
			response = PlannerResponse()
			request = PlannerRequest()
			request.start.point = [0, 0]
			request.goal.point = list(list(goal_point.coords)[0])
			request.obstacle_list.polygons= [ PointArray([ Point_xy(list(p)) for p in o ]) for o in self.obstacles ]
		
			response = self.plan_path(request)
		
			if response.ack:
				self.path = collections.deque([(pt.point[0],pt.point[1]) for pt in response.path.points])
			else:
				print("Failed to compute path!")

		except rospy.ServiceException as e:
			print("Service call failed: %s", e)


def main():
	rospy.init_node("manager", anonymous=True)
	bot = Manager()
	local_goal = Point(5, 5) # Will come from global planner when complete
	bot.go_to(local_goal)