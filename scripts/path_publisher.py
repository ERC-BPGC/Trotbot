#!/usr/bin/env python

import rospy
from nav_msgs.msg import Path
from tf.transformations import quaternion_from_euler as efq
from geometry_msgs.msg import PoseStamped, Pose

pub=rospy.Publisher("path_pose",Path,queue_size=1)
#sub_odom=rospy.Subscriber("odom",Odometry, getting_data)

		
	



def path_publishing_func(path):
	"""Publishes PoseArray for visualisation in rviz"""
	rospy.init_node("path_publisher")
	pub_Path=Path()
	pub_Pose=PoseStamped()
	q_x,q_y,q_z,q_w=efq(0,0,0)
	pub_Pose.header.frame_id="base_scan"
	pub_Path.header.frame_id="base_scan"
	
	while not rospy.is_shutdown():
		for points in path:
			
			pub_Pose.pose.position.x=points[0]
			pub_Pose.pose.position.y=points[1]
			
			pub_Path.poses.append(pub_Pose)
#			print(pub_Pose)
		print(pub_Path)
		
		pub.publish(pub_Path)
		pub_Path=Path()
		pub_Path.header.frame_id="base_scan"
	

if __name__=="__main__":
	path_publishing_func([[0,0],[1,1]])
	
		
		






