#! /usr/bin/env python

import rospy

class Manager():

	def __init__(self):
		self.position = []
		self.orientation = []
		self.obstacles = []
		self.path = []

		self.odometry_sub = rospy.Subscriber("odom", Odometry, self.odom_update)
		self.obstacle_sub = rospy.Subscriber("obstacles", PolygonArray, self.obstacle_update)
		self.path_sub = rospy.Subscriber("path", Float32MultiArray, self.path_update)

	def odom_update(self, data): 
		self.postion = data
		self.orientation = data

	def obstacle_update(self, data): 
		self.obstacles = data
		if not self.check_path():
			self.call_path_planner()

	def path_update(self, data):
		self.path = data

	def check_path(self):
		return true

	def call_path_planner(self, first_plan=False):
		rospy.wait_for_service('plan_path')
	    try:
	        plan_path = rospy.ServiceProxy('plan_path', PlanPath)
	        path_planned = plan_path(first_plan)
	    except rospy.ServiceException, e:
	        print "Service call failed: %s"%e


	

def main():
	rospy.init_node("manager", anonymous=True)