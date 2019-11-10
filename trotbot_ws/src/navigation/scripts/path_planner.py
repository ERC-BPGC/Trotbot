#! /usr/bin/env python

import rospy
import RRT
from navigation.srv import Planner , PlannerRequest , PlannerResponse


class Planner(object):

	def __init__(self, **args , **kwargs):

		self.obstacle_sub = rospy.Subscriber("obstacles", PolygonArray, self.obstacle_update)
		self.path_pub = rospy.Publisher("path", Float32MultiArray, queue_size = 10)
		self.planning_srv = rospy.Service('planner_service', Planner, self.plan)

	def plan(self , request):
		"""
		Callback of Service
		"""		
		st_pt = request.start
		end_pt = request.goal
		return_resp = PlannerResponse()
		
		self.path_pub.publish(path)
		return true

def main():
	rospy.init_node("path_planner", anonymous=True)
