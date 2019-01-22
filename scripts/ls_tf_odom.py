#! /usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
import tf

def callback(data):
    scan_odom=temp.lookupTransform("base_link","odom",rospy.Time(0))
    print(scan_odom)

sub=rospy.Subscriber("scan",LaserScan,callback)
temp=tf.TransformListener()

rospy.spin()
