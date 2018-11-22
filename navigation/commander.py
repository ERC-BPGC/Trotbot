#! /usr/bin/env python

__author__ = 'Atharb'
import rospy
from obstacle_expander.msg import Ipoly

class Current():
    """
    Class for current status of bot
    """

    def __init__(self)
        self.initialize_data()

    def initialize_data(self):
        self.curr_pos = [0, 0]
        self.obstacle_list = []
        self.path = []
        self.goal_pos = []
        self.old_data = [0, 0]
        self.target_changed = False

    def odom_update(self, data):
        """Update current position of bot."""
        self, curr_pos[0] = data.pose.pose.position.x
        self.curr_pos[1] = data.pose.pose.position.y


    def main_response(self, data):
       """Updates goal position and calls rrt."""
        if([data.x, data.y] != self.curr_target):
            self.curr_target = [data.x, data.y]
            path = do_RRT(show_animation = False, start_points_coors = [0, 0] , end_point_coors = self.curr_target , self.obstacle_list)
            target_changed = True
        else:
            target_changed = False

    def update_obst_list(self, data):
        """Create obstacle list"""
        tobi=[]
        self.obstacle_list = []
        for i in data.eternal_bliss:
            for j in i.bliss:
                tobi.append((j.x,j.y))
            poly=Polygon(tobi)
            self.obstacle_list.append(poly)
            tobi=[]


def main():
    curr = Current()
    rospy.init_node("commander", anonymous=True)
   
    odometry_sub = rospy.Subscriber("odom", Odom, curr.odom_update)
    obstacle_sub = rospy.Subscriber("ol1", Ipoly, curr.update_obst_list)
    gp_sub = rospy.Subscriber("global_plan", Float32[], curr.main_response)
    
    if not target_changed:
        curr.path = dynamic_rrt(start = curr.curr_pos, end = curr.curr_target, path = curr.path, obstacle_list = curr.obstacle_list)

    path_pub = rospy.Publisher("final_path", Float32[][], queue_size = 10)
    path_pub.publish(curr.path)

    rospy.spin()



if __name__=="__main__":
	main()
	
