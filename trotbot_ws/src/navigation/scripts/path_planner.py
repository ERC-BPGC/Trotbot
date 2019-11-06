#! /usr/bin/env python

import rospy

class Planner():

	def __init__(self):
		self.obstacle_sub = rospy.Subscriber("obstacles", PolygonArray, self.obstacle_update)
		self.path_pub = rospy.Publisher("path", Float32MultiArray, queue_size = 10)
		planning_srv = rospy.Service('plan_path', PlanPath, plan)

	def plan(data):
		
		self.path_pub.publish(path)
		return true

def main():
	rospy.init_node("path_planner", anonymous=True)