#! /usr/bin/env python


from std_msgs.msg import Float32MultiArray
import rospy

rospy.init_node("temp_pub")
pub=rospy.Publisher("/global_plan",Float32MultiArray,queue_size=1)

while not rospy.is_shutdown():
	temp=Float32MultiArray()
	temp.data=[3.0,3.0]

	pub.publish(temp)


