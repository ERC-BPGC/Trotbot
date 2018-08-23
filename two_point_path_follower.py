#! /usr/bin/env python

import rospy
import tf
import math
import numpy
from geometry_msgs.msg import *
from std_msgs.msg import *
from nav_msgs.msg import *

param = {
		'start':'',
		'kp':'',
		'vel_cmd':''
		}

default = {
			'start':'False',
			'kp':'0.01',
			'vel_cmd':'0.2'
			}
def initialize_parameters():
	rospy.set_param('/trajectory_follower_start', default['start'])
	param['start'] = default['start']

	if rospy.has_param('/trajectory_follower_kp'):
		param['kp'] = rospy.get_param('/trajectory_follower_kp')
	else:
		rospy.set_param('/trajectory_follower_kp', default['kp'])
		param['kp'] = default['kp']

	if rospy.has_param('/trajectory_follower_vel_cmd'):
		param['vel_cmd'] = rospy.get_param('/trajectory_follower_vel_cmd')
	else:
		rospy.set_param('/trajectory_follower_vel_cmd', default['vel_cmd'])
		param['vel_cmd'] = default['vel_cmd']

class TrajectoryFollower():
	"""docstring for TrajectoryFollower"""
	def __init__(self):
		self.initialize_variables()
		self.initialize_pubs_subs()

		rospy.Timer(rospy.Duration(0.05), self.controller)

	def update_params(self):
		if rospy.has_param('/trajectory_follower_start'):
			param['start'] = rospy.get_param('/trajectory_follower_start')

		if rospy.has_param('/trajectory_follower_kp'):
			param['kp'] = rospy.get_param('/trajectory_follower_kp')
		
		if rospy.has_param('/trajectory_follower_vel_cmd'):
			param['vel_cmd'] = rospy.get_param('/trajectory_follower_vel_cmd')
		

	def initialize_variables(self):
		self.target_point = [2.0,1.0,0.0] #[x,y,yaw]
		self.robot_x = 0.
		self.robot_y = 0.
		self.robot_yaw = 0.

	def odom_data_callback(self, data):
		self.robot_x = data.pose.pose.position.x
		self.robot_y = data.pose.pose.position.y

		self.robot_orientation = tf.transformations.euler_from_quaternion([data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w])
		self.robot_yaw = self.robot_orientation[2]

	def initialize_pubs_subs(self):
		self.odom_subscriber = rospy.Subscriber('/odom', Odometry, self.odom_data_callback)
		self.vel_cmd_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=5)


	def controller(self, event):
		# print "in controller"
		self.update_params()
		# print param['start']
		
		if param['start'] == 'True':
			# print "controller started"
			target_point_rel_x = self.target_point[0] - self.robot_x
			target_point_rel_y = self.target_point[1] - self.robot_y
			self.target_dir = (numpy.arctan2(target_point_rel_y, target_point_rel_x))
			self.delta_angle = self.target_dir - self.robot_yaw
			self.target_distance = math.sqrt((self.target_point[0] - self.robot_x)**2 + (self.target_point[1] - self.robot_y)**2)

			self.kp = float(param['kp'])

			
			if self.target_distance < 0.1:
				self.omega = 0.0
				self.vel_cmd = 0.0
			else:
				self.omega = self.kp*(self.target_dir - self.robot_yaw)
				if self.target_distance < 1.0:
					print "target ver close"
					if abs(self.delta_angle) > math.pi/2.0:
						self.vel_cmd = 0
					else:
						self.vel_cmd = float(param['vel_cmd'])

				else:
					"target angle too large"
					if abs(self.delta_angle) > math.pi/2.0:
						self.vel_cmd = 0
					else:
						self.vel_cmd = float(param['vel_cmd'])

			self.sp = Twist()
			self.sp.linear.x = self.vel_cmd
			self.sp.angular.z = self.omega

			self.vel_cmd_publisher.publish(self.sp)
			print "yo"
			print "vel, omega, dtheta, target target_distance:", self.vel_cmd, self.omega, self.delta_angle, self.target_distance
			print "yaw, dir:", self.robot_yaw, self.target_dir
			print "x, y:", self.robot_x, self.robot_y
			print "bf:", target_point_rel_x, target_point_rel_y

def main():
	rospy.init_node('trajectory_follower', anonymous=True)
	initialize_parameters()

	print "[TrajectoryFollower: ] Started trajectory_follower node"

	followe_obj = TrajectoryFollower()
	print "[TrajectoryFollower: ] obj created"

	try:
		rospy.spin()
	except KeyboardInterrupt:
		print "[TrajectoryFollower: ] Shutting down trajectory_follower node"


if __name__ == "__main__":

  main()