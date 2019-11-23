#! /usr/bin/env python

import rospy
from RRT import RRT

import utils
from utils import adjustable_random_sampler as sampler
from utils import los_optimizer as path_optimizer
from geometry_msgs.msg import Point32
from navigation.srv import Planner , PlannerRequest , PlannerResponse
from navigation.msg import PolyArray, PointArray

class Root():
    """
	Root class for path planning.
	Present Planners->(RRT, )

    Attributes:
		planning_srv: Sevice-Server for planning 
	"""
    def __init__(self, *args , **kwargs):
		"""
			Initialize all the planning algos. Service to be called --> 'algo_name'_planner_service  
		"""
		#RRT Planner Service 
		self.planning_srv = rospy.Service('rrt_planner_service', Planner, self.plan)

    def plan(self , request):
		"""Call Path planner selected(RRT, )
			
			Args:
				request: Request by ros client. Contains start , goal , obstacle_list as:

					geometry_msg/Point32 start
						float32 x
						float32 y
						float32 z

					geometry_msg/Point32 goal
						float32 x
						float32 y
						float32 z

					navigation/PolyArray obstacle_list
						navigation/PointArray[] polygons
							navigation/Point32[] points
								float32 x
								float32 y
								float32 z

			Returns:
				RETURN_RESP: PlannerResponse()
				
				navigation/PointArray path
					navigation/Point32[] points
						float32 x
						float32 y
						float32 z
				
				bool ack

		"""
		rospy.loginfo("Planning Request received")
		
		#Converting request from ros msg format -> basic python data type

		ST_PT = tuple(request.start.point) 
		END_PT = tuple(request.goal.point)
		ROBST = request.obstacle_list.polygons
		OBSTACLE = []
		
		#Extracting Obstacle information from ROBST
		for pt_array in ROBST : 
			tmp = []
			tmp = [tuple(pt.point) for pt in pt_array.points]
			OBSTACLE.append(tmp)
		
		RETURN_RESP = PlannerResponse()

		print("-"*30)
		rospy.loginfo(" Starting to plan from %r -> %r \n Obstacles-> %r"%(ST_PT,END_PT,OBSTACLE))
		print("-"*30)
		
		#Make class instance and get path,optimized_path
		tree = RRT(sample_area=(-5, 15), sampler=sampler, expand_dis=0.1)
		PATH , node_list = tree(ST_PT , END_PT , OBSTACLE)
		OPTIMIZED_PATH = path_optimizer(PATH, OBSTACLE)
		
		#Convert optimized path to Ros Format and Send
		RETURN_RESP.path.points = [ Point32(x=p[0], y=p[1]) for p in OPTIMIZED_PATH ]
		RETURN_RESP.ack = True
		print("-"*30)
		return RETURN_RESP

def main():
	rospy.init_node("Planner_Service", anonymous=True)
	try:
		Planner_Service = Root()
		rospy.loginfo("Setup for planners completed!")
	except Exception as er : 
		rospy.logerr("path_planner error: ")
		print(er)

	rospy.spin()
	
if __name__ == "__main__":
	main()
	rospy.logwarn("Killing!")
	