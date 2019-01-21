#! /usr/bin/env python
__author__ = "Atharv"

import rospy
import dynamic_rrt_integration as dri
from obstacle_expander.msg import *
from jsk_recognition_msgs.msg import *
#from geometry_msgs.msg import *
from shapely.geometry import Point
from shapely.geometry import Polygon
from shapely.geometry import LineString

from std_msgs.msg import *
from nav_msgs.msg import *


class Current():
    """
    Class for current status of bot
    """

    def __init__(self):
        self.initialize_data()
        
        self.path_pub = rospy.Publisher("final_path", Exp_msg, queue_size = 10)
        odometry_sub = rospy.Subscriber("odom", Odometry, self.odom_update)
        obstacle_sub = rospy.Subscriber("tp5xy5", PolygonArray, self.update_obst_list)
        gp_sub = rospy.Subscriber("global_plan", Float32MultiArray, self.main_response)
        
        rospy.Timer(rospy.Duration(0.05), self.dynamic_caller)

    def initialize_data(self):
        """Initialize data."""
        self.curr_pos = (0, 0)
        self.obstacle_list = []
        self.path = []
        self.goal_pos = (0, 0)
        self.curr_target = (0, 0)
        self.target_changed = False

    def path_convert(self):
        """Convert path into Exop_msg message type."""
        pub_path = Exp_msg()
        for i in self.path:
            epoint = Cordi()
            (epoint.x, epoint.y) = i
            pub_path.bliss.append(epoint)
        return(pub_path)

    def odom_update(self, data):
        """Update current position of bot."""
        self.curr_pos = (data.pose.pose.position.x, data.pose.pose.position.y)

    def main_response(self, data):
       	"""Updates goal position and calls rrt."""
        if((data.data[0], data.data[1]) != self.curr_target):
            self.curr_target = (data.data[0], data.data[1])
            print(self.obstacle_list)
            self.path = dri.rrtst.do_RRT(self.obstacle_list, show_animation = False, start_point_coors = (0, 0), end_point_coors = self.curr_target)
            self.path_pub.publish(self.path_convert())
            self.target_changed = True
            print("global update")
        else:
            self.target_changed = False

    
    def update_obst_list(self, data):
        """Create obstacle list."""
#        data = Ipoly()
#        data.eternal_bliss[i].bliss[i].x = 
        self.obstacle_list = []
        for i in data.polygons:
            points_list = [(j.x, j.y) for j in i.polygon.points]
            self.obstacle_list.append(Polygon(points_list))


    def dynamic_caller(self, event):
        """Call dynamic checker while en route"""
        #print("dynamic")
        if not self.target_changed:
            self.path = dri.dynamic_rrt(start = self.curr_pos, end = self.curr_target, path = self.path, obstacle_list = self.obstacle_list)
        self.path_pub.publish(self.path_convert())
		

def main():
    rospy.init_node("commander", anonymous=True)
    curr = Current()

    rospy.spin()


if __name__=="__main__":
	main()
	
