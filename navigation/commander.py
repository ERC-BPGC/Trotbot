#! /usr/bin/env python
__author__ = "Atharv"

import rospy
import dynamic_rrt_integration as dri
from obstacle_expander.msg import Ipoly, bliss
from std_msgs.msg import *


class Current():
    """
    Class for current status of bot
    """

    def __init__(self):
        self.initialize_data()

    def initialize_data(self):
        """Initialize data."""
        self.curr_pos = (0, 0)
        self.obstacle_list = []
        self.path = []
        self.goal_pos = (0, 0)
        self.curr_target = (0, 0)
        self.target_changed = False

    def odom_update(self, data):
        """Update current position of bot."""
        self, curr_pos = (data.pose.pose.position.x, data.pose.pose.position.y)

    def main_response(self, data):
       	"""Updates goal position and calls rrt."""
	if((data.x, data.y) != self.curr_target):
            self.curr_target = (data.x, data.y)
            path = do_RRT(show_animation = False, start_points_coors = (0, 0), end_point_coors = self.curr_target, self.obstacle_list)
            self.target_changed = True
        else:
            self.target_changed = False

    
    def update_obst_list(self, data):
        """Create obstacle list."""
        self.obstacle_list = []
        for i in data.eternal_bliss:
            points_list = [(j.x, j.y) for j in i.bliss]
            self.obstacle_list.append(Polygon(points_list))


    def dynamic_caller(self):
        """Call dynamic checker while en route"""
        if not self.target_changed:
            self.path = self.dynamic_rrt(start = self.curr_pos, end = self.curr_target, path = self.curr_path, obstacle_list = self.obstacle_list) 


def main():
    curr = Current()
    rospy.init_node("commander", anonymous=True)
   
    odometry_sub = rospy.Subscriber("odom", Odom, curr.odom_update)
    obstacle_sub = rospy.Subscriber("ol1", Ipoly, curr.update_obst_list)
    gp_sub = rospy.Subscriber("global_plan", Float32MultiArray, curr.main_response)
    
    rospy.Timer(rospy.Duration(0.05), curr.dynamic_caller())

    path_pub = rospy.Publisher("final_path", bliss, queue_size = 10)
    path_pub.publish(curr.path)

    rospy.spin()


if __name__=="__main__":
	main()
	
