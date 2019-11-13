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
		
		self.plan_path = rospy.ServiceProxy('rrt_planner_service', Planner)
			

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
				self.path = [(pt.point[0],pt.point[1]) for pt in response.path.points]
			except Exception as e:
				print("Failed to compute path!")
				print(e)


	    except rospy.ServiceException, e:
	        print "Service call failed: %s"%e


	

def main():
	rospy.init_node("manager", anonymous=True)