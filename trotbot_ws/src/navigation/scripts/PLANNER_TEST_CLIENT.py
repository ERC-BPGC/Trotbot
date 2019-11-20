#! /usr/bin/env python
import rospy
import sys
from navigation.srv import Planner , PlannerRequest , PlannerResponse
from navigation.msg import Point_xy , PolyArray, PointArray

rospy.init_node("client_task2", anonymous=True)

class Client_t2():
    def __init__(self):
		self.client=rospy.ServiceProxy("rrt_planner_service", Planner)
		self.response=PlannerResponse()
		self.request=PlannerRequest()
		self.request.start.point=[1,1]
		self.request.goal.point=[10,10]
		self.request.obstacle_list.polygons=[
		PointArray([ Point_xy([8, 5]), Point_xy([7,8]), Point_xy([2,9]), Point_xy([3,5]) ]),
		PointArray([ Point_xy([3,3]), Point_xy([3,5]), Point_xy([5,5]), Point_xy([5,3]) ])
		]
		print(self.request)
		print("-"*30)

    def get_response(self):
		# print(sys.argv[0])
		print("Sending Request")
		self.response=self.client(self.request)
		print(self.response)

if __name__=="__main__":
	try:
		rospy.logwarn("Waiting for Service")
		rospy.wait_for_service("rrt_planner_service")
	except:
		print("SERVICE UNAVAILABLE")
	
	o=Client_t2()
	o.get_response()