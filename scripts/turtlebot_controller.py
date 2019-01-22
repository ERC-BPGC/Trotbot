#! /usr/bin/env python
# import sys
import rospy
# import tf
import math
import numpy
from geometry_msgs.msg import *
from std_msgs.msg import *
from nav_msgs.msg import *
# import heapq
from obstacle_expander.msg import *
from tf.transformations import euler_from_quaternion

ang_vel = 0.6
lin_vel = 0.3
pt_buffer = 0.05
ang_buffer = 0.01

class Controller():

	def __init__(self):
		self.initialize_data()
		odometry_sub = rospy.Subscriber("odom", Odometry, self.odom_update)
		path_sub = rospy.Subscriber("final_path", Exp_msg, self.path_update)
		self.vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size = 1)


	def initialize_data(self):
		"""Initialize data."""
		self.curr_pos = (0, 0)
		self.old_path = []
		self.curr_path = []
		self.path_changed = False
		self.counter = 1
		self.curr_ang=0


	def dist_bw(self,pt_a, pt_b):
		return (pt_a[0] - pt_b[0])**2 + (pt_a[1] - pt_b[1])**2

	def odom_update(self, data):
		"""Update current position of bot."""
		self.curr_pos = (data.pose.pose.position.x, data.pose.pose.position.y)
		roll,pitch,yaw=euler_from_quaternion([data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w])
		self.curr_ang=yaw

	def path_update(self, data):
		"""Update path to be taken by both."""
		self.old_path = self.curr_path[:]
		self.curr_path = []
		for epoint in data.bliss:
			i = (epoint.x, epoint.y)
			self.curr_path.append(i)

		if self.curr_path != self.old_path:
			self.path_changed = True
			self.counter = 1

		self.vel_publisher()


	def vel_publisher(self):
		"""Publish velocity according to conditions."""
		if(self.dist_bw(self.curr_pos, self.curr_path[self.counter]) < pt_buffer):
			self.counter+=1


		next_pt = self.curr_path[self.counter]
		ang_diff = -self.curr_ang + math.atan((next_pt[1] - self.curr_pos[1]) / (next_pt[0] - self.curr_pos[0]))
		vel = Twist()

		while (abs(ang_diff) > ang_buffer):
			ang_diff = -self.curr_ang + math.atan((next_pt[1] - self.curr_pos[1]) / (next_pt[0] - self.curr_pos[0]))
			# print(ang_diff)
			turn_time = abs(ang_diff / ang_vel)
			vel.linear.x = 0.3
			vel.angular.z = ang_diff*ang_vel/abs(ang_diff)
			# print ("vel")
			self.vel_pub.publish(vel)

			# print("vel")
		vel.linear.x = lin_vel
		vel.angular.z = vel.angular.z = ang_diff/abs(ang_diff)*ang_vel
		self.vel_pub.publish(vel)

def main():
	rospy.init_node("controller", anonymous=True)
	curr = Controller()

	rospy.spin()


if __name__ == '__main__':
	main()
