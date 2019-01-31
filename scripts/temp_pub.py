#! /usr/bin/env python


# from std_msgs.msg import Float32MultiArray
from obstacle_expander.msg import *
import rospy

rospy.init_node("temp_pub")
pub=rospy.Publisher("global_plan",Exp_msg,queue_size=1)

while not rospy.is_shutdown():
	temp=Exp_msg()
	temp.bliss=[Cordi(3,0,0)]

	pub.publish(temp)
