#! /usr/bin/env python

import rospy
import actionlib
import math
import utils
import collections
import shapely

from tf import transformations
from shapely.geometry import Point, LineString

from utils import Orientation, REACH_DIST
from geometry_msgs.msg import Point32
from nav_msgs.msg import Odometry
from navigation.msg import PolyArray, PointArray, MoveBotAction, MoveBotGoal
from navigation.srv import Planner, PlannerRequest, PlannerResponse


class Manager():

	def __init__(self):

		self.position = Point(0, 0) 
		self.orientation = Orientation(0, 0, 0)
		self.current_goal_point = Point(0, 0)
		self.obstacles = []
		self.path = collections.deque()

		self.odometry_sub = rospy.Subscriber("odom", Odometry, self.__odom_update)
		self.obstacle_sub = rospy.Subscriber("obstacles", PolyArray, self.__obstacle_update)
		
		self.controller_client = actionlib.SimpleActionClient('move_bot', MoveBotAction)
		self.controller_client.wait_for_server()
		
		self.plan_path = rospy.ServiceProxy('rrt_planner_service', Planner)

        rospy.loginfo("...Manager Initialized...")
		

	def go_to(self, goal_point):

		self.current_goal_point = goal_point
		self.__call_path_planner(self.current_goal_point)
		while len(self.path) is not 0:
			self.__move_to_next_point()

		
	def __move_to_next_point(self):

		next_point = self.path[0]
		goal_for_controller = MoveBotGoal()
		goal_for_controller.goal.point = [next_point.x, next_point.y]
		self.controller_client.send_goal(goal_for_controller, feedback_cb=self.__next_point_reached)
		controller_client.wait_for_result()

	def __next_point_reached(self, _, done):

		if done.ack:
			self.path.popleft()

	def __odom_update(self, data): 

		self.position, self.orientation = utils.unwrap_pose(data.pose.pose)
		self.path = collections.deque(utils.transform(LineString(self.path), self.position, self.orientation))
		self.current_goal_point = utils.transform(self.current_goal_point, self.position, self.orientation)

		if self.goal.x < REACH_DIST and self.goal.y < REACH_DIST:
			return # TODO: Add completion mech 


	def __obstacle_update(self, data):

		self.obstacles = [ [ (point.x, point.y) for point in polygon.points ] for polygon in data.polygons ] 
		if not utils.check_intersection(self.path, self.obstacles):
			self.__call_path_planner(self.current_goal_point)

	def __call_path_planner(self, goal_point):

		rospy.logwarn("Waiting for Service")
		rospy.wait_for_service("rrt_planner_service")
		
		try:
			response = PlannerResponse()
			request = PlannerRequest()
			request.start.point = [0, 0]
			request.goal.point = list(list(goal_point.coords)[0])
			request.obstacle_list.polygons= [ PointArray([ Point32(x=p[0], y=p[1]) for p in o ]) for o in self.obstacles ]
		
			response = self.plan_path(request)
		
			if response.ack:
				self.path = collections.deque([ Point(pt.point) for pt in response.path.points ])
			else:
				print("Failed to compute path!")

		except rospy.ServiceException as e:
			print("Service call failed: %s", e)


def main():
	bot = Manager()
	local_goal = Point(5, 5) # Will come from global planner when complete
	bot.go_to(local_goal)


if __name__ == "__main__":
    rospy.init_node("manager", anonymous=True)

    try:
        main()

    except Exception as err:
        rospy.loginfo("%s was thrown",err)
