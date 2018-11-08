#! /usr/bin/env python

import rospy
import math
from sensor_msgs.msg import LaserScan 
from geometry_msgs.msg import Polygon,Point32,PolygonStamped 
from std_msgs.msg import Header
from jsk_recognition_msgs.msg import PolygonArray #for polygon conversion

#for decreasing the coordinate val
def decrease_in_direction(val): 
	if val!="inf":
		val-=0.2
	return val

#for increasing the coordinate val
def increase_in_direction(val):
	if val!="inf":
		val+=0.4 #-4 because the value is changed in decrease_in_direction
	return val


#for converting the point array to a polygon using jsk_recognition_msgs 
def getting_cordi(A,B):
	theta_increment=(2)*(math.pi)/360
	re=PolygonArray()
	re.header.frame_id="odom"
	roar=PolygonStamped()
	roar.header.frame_id="odom"
	pt=[]	
	count=0
#	print("This is A:")
#	print(A)
	for i in range(len(A)):
		if str(A[i])!="inf":
			count+=1
			theta1=(i)*theta_increment
#			print(A[i])
			x=A[i]*math.sin(theta1)
			y=A[i]*math.cos(theta1)       
			roar.polygon.points.append(Point32(x,y,0))
			pt.append([x,y,theta1,i])
			if i==len(A)-1 or str(A[i+1])=="inf":
				for j in range(i,i-count,-1):
					theta2=(j)*theta_increment
					x=B[j]*math.sin(theta2)
					y=B[j]*math.cos(theta2)       
					roar.polygon.points.append(Point32(x,y,0))
					pt.append([x,y,theta2,j])			
#				print(pt)
				pt=[]
				count=0
#				print("***************************************************************")
#				print(poly.points)
				re.polygons.append(roar)
				roar=PolygonStamped()
				roar.header.frame_id="odom"
	return re		



#adding additional points at the end of the obstacle
def increasing_array(input_data):
	
	count=len(input_data)
	i=1
	while i<count:
		if (str(input_data[i])=='inf' and str(input_data[i-1])!='inf'):
			insert_element=input_data[i-1]
			for j in range(3):
				if i+j==360:
					break
				input_data[i+j]=insert_element
			i+=4
		elif (str(input_data[i-1])=='inf' and str(input_data[i])!='inf'):
			insert_element=input_data[i]
			for j in range(-1,-4,-1):
				if i+j==0:
					break
				else:
					input_data[i+j]=insert_element
			i+=4
		i+=1
	print(input_data)
	return input_data 




def dothis(data):
#	print("This is data:")
#	print(data)
#	toedit1 = LaserScan()
	toedit1=data
	edited1=list(map(decrease_in_direction,toedit1.ranges[:]))
	toedit1.ranges=edited1[:]
#	print(toedit1.ranges)
	toedit1_extended=increasing_array(toedit1.ranges[:])
	toedit1.ranges=toedit1_extended
#	print(toedit1_extended)
	xy_e1=getting_cordi(edited1,list(map(increase_in_direction,toedit1.ranges[:])))
	pub5=rospy.Publisher("tp5xy5",PolygonArray,queue_size=10)
	pub5.publish(xy_e1)
#	rd=PolygonStamped()
#	rd.header.frame_id="odom"
#	rd.header.stamp=rospy.Time.now()
#	points_thru_polygon1=Polygon()
#	points_thru_polygon2=Polygon()
#	points_thru_polygon1.points=[Point32(1,1,0),Point32(1,-1,0),Point32(-1,-1,0),Point32(-1,1,0)]
#	points_thru_polygon2.points=[Point32(0,0,0),Point32(1,1,0),Point32(2,0,0),Point32(1,-1,0)]	
#	rd.polygon=points_thru_polygon1
#	rd.polygon=points_thru_polygon2
#	pub2=rospy.Publisher("tp2",PolygonStamped,queue_size=10)
	pub1=rospy.Publisher("tp1",LaserScan,queue_size=10)
#	print(rd)
	pub1.publish(toedit1)
#	pub2.publish(rd)
	rate=rospy.Rate(10)
	rate.sleep()




def dothat(datta):
	toedit2 = LaserScan()
	toedit2=datta
	edited2=list(map(increase_in_direction,toedit2.ranges[:]))
	toedit2.ranges=edited2[:]
#	print(edited2)
#	xy_e2=getting_cordi(edited2)
#	print(len(xy_e2))
#	pub4=rospy.Publisher("tp4xy2",PolygonArray,queue_size=10)
	pub3=rospy.Publisher("tp3",LaserScan,queue_size=10)
	pub3.publish(toedit2)
#	pub4.publish(xy_e2)
	rate=rospy.Rate(10)
	rate.sleep()
	
	
#subscriber. scan topic, takes LaserScan and passes it to callBack functions	
def expander_Attempt1():
	rospy.init_node("object_expander",anonymous=True)
	sub1=rospy.Subscriber("scan",LaserScan,dothis)
	sub2=rospy.Subscriber("scan",LaserScan,dothat)
	rospy.spin()
	
#def sending_to_rviz(data):
#	rospy.init_node("sending_to_rviz",anonymous=True)
#	pub=rospy.Publisher("scan",this,queue_size)
	
if __name__=='__main__':
	expander_Attempt1()
