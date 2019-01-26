#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan 
from obstacle_expander.srv import Exp_srv
rospy.init_node("get_laser")
rospy.wait_for_service("Expanding")


def callback(data):
	laser=rospy.ServiceProxy("Expanding",Exp_srv)
	laser(data)	
sub=rospy.Subscriber("scan",LaserScan,callback)
rospy.spin()
