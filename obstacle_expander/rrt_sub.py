#! /usr/bin/env python

import rospy
from obstacle_expander.msg import Ipoly

def subscriber():
	rospy.init_node("from_oex",anonymous=True)
	sub=rospy.Subscriber("ol2",Ipoly,to_rrt)
	rospy.spin()
	
def to_rrt(data):
	ret=[]
	tobi=[]
	for i in data.eternal_bliss:
		for j in i.bliss:
			tobi.append((j.x,j.y))
		print(tobi)
		ret.append(tuple(tobi))
		tobi=[]
	print("This is ret-->")
	print(ret)
#	rospy.loginfo(data)
	
if __name__=="__main__":
	subscriber()
	
