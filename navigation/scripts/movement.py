#!/usr/bin/env python

import rospy
import math
import numpy as np 
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist, Pose, Point32
from shapely.geometry import Point, LineString, Polygon
from collections import namedtuple
from navigation.msg import PolyArray, PointArray
from navigation.srv import Planner, PlannerRequest, PlannerResponse
import tf
import utils
import collections
import shapely


Orientation = namedtuple('Orientation', ['roll', 'pitch', 'yaw'])
DISTMIN = 0.1

class Bot():

    def __init__(self):
        self.final_goal = Point()
        self.path = []
        self.path_use = []
        self.vel_pub = rospy.Publisher('robot0/cmd_vel', Twist, queue_size=10)
        self.odom_sub = rospy.Subscriber('robot0/odom', Odometry, self.odom_update)
        self.path_sub = rospy.Subscriber('/path', Path, self._update_path)
        self.obstacle_sub = rospy.Subscriber('obstacles', PolyArray, self.obstacle_update)
        self.obstacles = PolyArray()
        self.obstacle_list = []
        self.path_planner = rospy.ServiceProxy('rrt_planner_service', Planner)
        self.position = Point(0, 0, 0)
        self.orientation = Orientation(0, 0, 0)
        self.velocity = Twist()
        self.move_to_goal = False
        self.goal = Point()
        self.goal_reached = False
        self.rate = rospy.Rate(10)
        self.position0 = Point(0, 0, 0)
        self.orientation0 = Orientation(0, 0, 0)
        
    def odom_update(self, msg):
        ## callback for odom sub
        self.position = msg.pose.pose.position
        self.orientation = Orientation(*tf.transformations.euler_from_quaternion([
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ]))
        self.position.x -= self.position0.x
        self.position.y -= self.position0.y
        if len(self.path) > 1:
            self.path_use = [Point(p[0], p[1]) for p in utils.transform(
                    LineString([(p.x, p.y) for p in self.path]), self.position, self.orientation).coords]
        print([(p.x, p.y) for p in self.path_use][1])        
        self._set_goal()
        self.vel_update()
        self.rate.sleep()


    def vel_update(self):
        ##function for setting the velocity
        
        if self.move_to_goal:
            self.velocity.linear.x = 0.05*(self.goal.x) 
            self.velocity.linear.y = 0.05*(self.goal.y) 
        else:
            self.velocity.linear.x = 0
            self.velocity.linear.y = 0

        self.vel_pub.publish(self.velocity)
        print(self.final_goal.x, self.final_goal.y)
        #print(self.velocity.linear.x, self.velocity.linear.y)
        #print([(p.x, p.y) for p in self.path])

    def obstacle_update(self, data):
        ## call back for odom sub
        self.obstacles = data
        if len(self.path) < 1 or utils.check_intersection([(point.x, point.y) for point in self.path], [[(point.x, point.y) for point in polygon.points] for polygon in self.obstacles.polygons]):
            self.get_path()

    def _set_goal(self):
        if abs(self.final_goal.x) < DISTMIN and (self.final_goal.y) < DISTMIN:
            self.goal_reached = True  

        if len(self.path_use) != 0 and not self.goal_reached:
            self.next_loc = self.path_use[0]
            print((self.next_loc.x, self.next_loc.y), len(self.path))
            if abs(self.next_loc.x) < DISTMIN and abs(self.next_loc.y) < DISTMIN:
                self.move_to_goal = False
                self.path.pop(0)
            else :
                self.goal = self.next_loc
                self.move_to_goal = True
        elif self.goal_reached:
            self.velocity.linear.x, self.velocity.linear.y = 0, 0
            self.vel_pub.publish(self.velocity)
            print('-------Completed Path-------')      
         

    def get_path(self):
        
        ## callback for the planner request
        rospy.wait_for_service('rrt_planner_service')
        try:
            self.final_goal = utils.transform(self.final_goal, self.position, self.orientation)
            self.position0 = Point(self.position.x, self.position.y)
            self.orientation0 = Orientation(self.orientation.roll, self.orientation.pitch, self.orientation.yaw)            
            response = PlannerResponse()
            request = PlannerRequest()
            request.start.x = 0
            request.start.y = 0
            request.goal.x = self.final_goal.x
            request.goal.y = self.final_goal.y
            request.obstacle_list = self.obstacles

            response = self.path_planner(request)
            if not response.ack:
                print('----Path not planned----')             


        except rospy.ServiceException :
            print('Error')

    def _update_path(self, data):
        # callback for the path sub (if implemented)
        self.path_raw = data.poses
        for pose in self.path_raw:
            pose = Pose()
            point = Point(pose.position.x, pose.position.y)
            self.path.append(point)
        
        

        
        


def main():
    rospy.init_node('BOT')
    bot = Bot()
    bot.final_goal = Point(5, 5)
    rospy.spin()

if __name__ == '__main__':
    main()