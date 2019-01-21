#!/usr/bin/env python

import rospy
from std_msgs.msg import *

rospy.init_node('gp', anonymous=True)
pub = rospy.Publisher('global_plan', Float32MultiArray, queue_size=10)
rate = rospy.Rate(10)

while not rospy.is_shutdown():
	target = Float32MultiArray()
	target.data = [10.0, 10.0]	
	pub.publish(target)
	rate.sleep()
