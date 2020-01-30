#! /usr/bin/env python

import rospy
from RRT import RRT
import utils
from utils import adjustable_random_sampler as sampler
from utils import los_optimizer as path_optimizer
from std_msgs.msg import Header
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Point32, Point, Pose, Quaternion, PoseStamped
from navigation.srv import Planner, PlannerRequest, PlannerResponse
from navigation.msg import PolyArray, PointArray


class subs():
	def __init__(self):
		self._odom = Odometry()
		self.odom_sub = rospy.Subscriber("odom", Odometry, self.odom_update)
		self.rate = rospy.Rate(100)

	def odom_update(self, data):
		self._odom = data
		self.rate.sleep()

	def get_x(self):
		return self._odom.pose.pose.position.x

	def get_y(self):
		return self._odom.pose.pose.position.y


class root_planner():
	"""
	root_planner class for path planning.
	Present Planners->(RRT, )

	Attributes:
	planning_srv: Sevice-Server for planning
	"""

	def __init__(self, *args, **kwargs):
		"""
			Initialize all the planning algos. Service to be called --> 'algo_name'_planner_service
		"""
		# RRT Planner Service
		self.planning_srv = rospy.Service('rrt_planner_service', Planner, self.plan)
		self.planning_pub = rospy.Publisher('path', Path, queue_size=5)
		self.subs = subs()

	def plan(self, request):
		"""Call Path planner selected(RRT)

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
		# Converting request from ros msg format -> basic python data type
		ST_PT = request.start.x, request.start.y
		END_PT = request.goal.x, request.goal.y
		ROBST = request.obstacle_list.polygons
		OBSTACLE = []

		# Extracting Obstacle information from ROBST
		for pt_array in ROBST:
			tmp = []
			tmp = [(pt.x, pt.y) for pt in pt_array.points]
			OBSTACLE.append(tmp)

		RETURN_RESP = PlannerResponse()

		print("-"*30)
		rospy.loginfo(" Starting to plan from %r -> %r \n" % (ST_PT, END_PT))
		print("-"*30)

		# Make class instance and get path,optimized_path
		tree = RRT(sample_area=(-5, 15), sampler=sampler, expand_dis=0.1)
		PATH, node_list = tree(ST_PT, END_PT, OBSTACLE)
		OPTIMIZED_PATH = path_optimizer(PATH, OBSTACLE)
		# print(OPTIMIZED_PATH)
		# Convert optimized path to Ros Format and Send
		path_to_be_published = Path()
		path_to_be_published.header = Header(frame_id='odom')

		path_to_be_published.poses = [PoseStamped(pose=Pose(position=Point(x=p[0]+self.subs.get_x(), y=p[1]+self.subs.get_y(), z=0), orientation=Quaternion(
		    x=0, y=0, z=0, w=0)), header=Header(frame_id="odom")) for p in OPTIMIZED_PATH]

		# print(path_to_be_published)
		self.planning_pub.publish(path_to_be_published)
		RETURN_RESP.path.points = [Point32(x=p[0], y=p[1]) for p in OPTIMIZED_PATH]
		RETURN_RESP.ack= True
		print("-"*30)

		return RETURN_RESP


if __name__ == "__main__":
	rospy.init_node("Planner_Service", anonymous=True)

	try:
		Planner_Service= root_planner()
		rospy.loginfo("Setup for planners completed!")
	except Exception as er:
		rospy.logerr("path_planner error: ")
		print(er)

	rospy.spin()

	rospy.logwarn("Killing Path Planner!")
