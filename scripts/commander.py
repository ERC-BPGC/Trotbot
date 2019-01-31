#! /usr/bin/env python
__author__ = "Atharv"

import rospy
import math
import dynamic_rrt_integration as dri
from obstacle_expander.msg import *
from jsk_recognition_msgs.msg import *

from shapely.geometry import Point
from shapely.geometry import Polygon
from shapely.geometry import LineString

from std_msgs.msg import *
from nav_msgs.msg import *
from geometry_msgs.msg import Twist

from tf.transformations import euler_from_quaternion


global ang_buffer,ang_vel,lin_vel

ang_buffer=0.01
ang_vel=0.4
lin_vel=0.5

class Current():
    """
    Class for current status of bot
    """

    def __init__(self):
        self.initialize_data()

        self.path_pub = rospy.Publisher("final_path", Exp_msg, queue_size = 1)
        odometry_sub = rospy.Subscriber("odom", Odometry, self.odom_update)
        obstacle_sub = rospy.Subscriber("tp5xy5", PolygonArray, self.update_obst_list)
        gp_sub = rospy.Subscriber("global_plan", Exp_msg, self.gb_path)
        self.vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size = 10)
        # rospy.Timer(rospy.Duration(0.05), self.dynamic_caller)

    def initialize_data(self):
        """Initialize data."""
        self.curr_pos = (0, 0)
        self.obstacle_list = []
        self.rel_path=[]
        self.curr_target = (0, 0)
        self.target_changed = False
        self.gp_counter=0
        self.next_pt_count=0
        self.next_path_point=(0,0)
        self.next_pp_ang=0
        self.prev_path_point=(0,0)

    def path_convert(self):
        """Convert path into Exp_msg message type."""
        pub_path = Exp_msg()
        for i in self.rel_path[:]:
            epoint = Cordi()
            (epoint.x, epoint.y) = i
            pub_path.bliss.append(epoint)
        return(pub_path)

    def odom_update(self, data):
        """Update current position of bot."""
        self.curr_pos = (data.pose.pose.position.x, data.pose.pose.position.y)
#        roll,pitch,yaw=euler_from_quaternion([data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w])
#        self.curr_ang=yaw

    def target_reached(self):
        '''checks if bot reached target'''
        if((self.curr_pos[0]-self.curr_target[0])**2 + (self.curr_pos[1]-self.curr_target[1])**2)<0.25:
            print("reached--> nxt_gb_pt"+str(self.curr_pos[0])+","+str(self.curr_pos[1]))
            self.gp_counter+=1
        if ((self.curr_pos[0]-(self.next_path_point[0]+self.prev_path_point[0])**2 + (self.curr_pos[1]-(self.next_path_point[1]+self.prev_path_point[1]))**2))<0.0000005:
            print("reached--> nxt_pt"+str(self.curr_pos[0])+","+str(self.curr_pos[1]))
            print(self.next_path_point[0]+self.prev_path_point[0])
            self.prev_path_point=self.curr_pos[0],self.curr_pos[1]
            self.next_pt_count+=1
            if self.next_pt_count >= len(self.rel_path):
                self.next_pt_count -= 1

			
    def gb_path(self,data):
        '''Plans locally between global path.'''
        while not rospy.is_shutdown():
            if self.gp_counter==len(data.bliss):
                print(len(data.bliss),self.gp_counter)
                print("reached final goal!")
                #v=Twist()
                rospy.signal_shutdown("done")
            next_pt=Float32MultiArray()
            next_pt.data=[data.bliss[self.gp_counter].x,data.bliss[self.gp_counter].y]
        # print(next_pt)        
            self.main_response(next_pt)
            self.target_reached()


    def main_response(self, data):
       	"""Updates goal position and calls rrt."""
        if((data.data[0], data.data[1]) != self.curr_target):
            self.curr_target = (data.data[0], data.data[1])
            #print(self.obstacle_list)
            self.rel_path = dri.rrtst.do_RRT(obstacleList2=self.obstacle_list, show_animation = False, start_point_coors = (0,0), end_point_coors = (self.curr_target[0]-self.curr_pos[0],self.curr_target[1]-self.curr_pos[1]))
            self.next_path_point=self.rel_path[1]
            #print(self.next_path_point)
            try:
            	self.next_pp_ang=math.atan((self.next_path_point[1]-self.curr_pos[1])/(self.next_path_point[0]-self.curr_pos[0]))
            except ZeroDivisionError:
            	self.next_pp_ang=1.57
#            print(self.rel_path)
            # self.path_pub.publish(self.path_convert())
            self.target_changed = True
            ang_buffer=0.01
            print("global update")
        else:
            self.target_changed = False
        self.dynamic_caller()


    def update_obst_list(self, data):
        """Create obstacle list."""
#        data = Ipoly()
#        data.eternal_bliss[i].bliss[i].x =
        self.obstacle_list = []
        for i in data.polygons:
            points_list = [(j.x, j.y) for j in i.polygon.points]
            self.obstacle_list.append(Polygon(points_list))


    def dynamic_caller(self):
        """Call dynamic checker while en route"""
        # next_pt_path=self.rel_path[:]
        # print("dynamic")

        if not self.target_changed:
            curr_path=self.rel_path[:]
            self.rel_path= dri.dynamic_rrt(start = (0,0), end =(self.curr_target[0]-self.curr_pos[0],self.curr_target[1]-self.curr_pos[1]), path = self.rel_path, obstacle_list = self.obstacle_list)
            if self.rel_path != curr_path:
            	self.next_pt_count=0
            print(self.next_pt_count)
                # print("path not same")
            print(self.rel_path)
            self.next_path_point=self.rel_path[self.next_pt_count]
            
            try:
            	self.next_pp_ang=math.atan((self.next_path_point[1]-self.curr_pos[1])/(self.next_path_point[0]-self.curr_pos[0]))#        print(self.obstacle_list)
            except ZeroDivisionError:
                self.next_pp_ang=1.57

        self.path_pub.publish(self.path_convert())
        self.vel_command()


    def vel_command(self):
        '''cmd_vel publisher'''
        vel=Twist()
        vel.linear.x=math.cos(self.next_pp_ang)
        vel.linear.y=math.sin(self.next_pp_ang)
        self.vel_pub.publish(vel)




def main():
    rospy.init_node("commander", anonymous=True)
    curr = Current()

    rospy.spin()


if __name__=="__main__":
	main()
