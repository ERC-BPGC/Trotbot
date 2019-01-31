#! /usr/bin/env python



import rospy
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Polygon,Point32,PolygonStamped
from std_msgs.msg import Header
from jsk_recognition_msgs.msg import PolygonArray
from obstacle_expander.msg import Ipoly,Exp_msg,Cordi#for polygon conversion


#for decreasing the coordinate val
def decrease_in_direction(val):
	if val!="inf":
		val-=0.2 #change this vaue according to bot size
	return val

#for increasing the coordinate val
def increase_in_direction(val):
	if val!="inf":
		val+=0.2 #-4 because the value is changed in decrease_in_direction
	return val


#for converting the point array to a polygon using jsk_recognition_msgs
def getting_cordi(A,B,shu):
	theta_increment=shu
	re=PolygonArray()
	re.header.frame_id="odom"
	roar=PolygonStamped()
	roar.header.frame_id="odom"
	roar.header.stamp=rospy.Time.now()
	pt=Exp_msg()
	count=0
	baby=Ipoly()
	for i in range(len(A)):
		if str(A[i])!="inf":
			if count==0:
				for t in range(10,1,-1):
					theta1=(i-t)*theta_increment
					x=A[i]*math.cos(theta1)
					y=A[i]*math.sin(theta1)
					roar.polygon.points.append(Point32(x,y,0))
					pt.bliss.append(Cordi(x,y,0))
			count+=1
			theta1=(i)*theta_increment
			x=A[i]*math.cos(theta1)
			y=A[i]*math.sin(theta1)
			roar.polygon.points.append(Point32(x,y,0))
			pt.bliss.append(Cordi(x,y,0))
			if i==len(A)-1 or abs(A[i+1]-A[i])>0.15:
				# ra=A[i]/math.cos(theta_increment)
				for t in range(1,10):
					theta1=(t+i)*theta_increment
					# ra=ra/math.cos(theta_increment)
					x=A[i]*math.cos(theta1)
					y=A[i]*math.sin(theta1)
					roar.polygon.points.append(Point32(x,y,0))
					pt.bliss.append(Cordi(x,y,0))
				for t in range(10,1,-1):
					theta1=(t+i)*theta_increment
					x=(B[i])*math.cos(theta1)
					y=(B[i])*math.sin(theta1)
					roar.polygon.points.append(Point32(x,y,0))
					pt.bliss.append(Cordi(x,y,0))
				for j in range(i,i-count,-1):
					theta2=(j)*theta_increment
					x=B[j]*math.cos(theta2)
					y=B[j]*math.sin(theta2)
					roar.polygon.points.append(Point32(x,y,0))
					pt.bliss.append(Cordi(x,y,0))
				for t in range(-1,-10,-1):
					theta2=(j+t)*theta_increment
					x=B[j]*math.cos(theta2)
					y=B[j]*math.sin(theta2)
					roar.polygon.points.append(Point32(x,y,0))
					pt.bliss.append(Cordi(x,y,0))
				baby.eternal_bliss.append(pt)
				pt=Exp_msg()
				count=0
				re.polygons.append(roar)
				roar=PolygonStamped()
				roar.header.frame_id="odom"
				roar.header.stamp=rospy.Time.now()
	return re

def dothis(data):
	for i in data
	edited1=list(map(decrease_in_direction,data.ranges[:]))
	xy_e1=getting_cordi(edited1,list(map(increase_in_direction,data.ranges[:])),data.angle_increment)
	pub5=rospy.Publisher("tp5xy5",PolygonArray,queue_size=0)
	pub5.publish(xy_e1)


#subscriber. scan topic, takes LaserScan and passes it to callBack functions
def expander_Attempt1():
	rospy.init_node("object_expander",anonymous=True)
	sub1=rospy.Service("scan",LaserScan,dothis)
	rospy.spin()


if __name__=='__main__':
	expander_Attempt1()
