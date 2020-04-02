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


Orientation = namedtuple('Orientation', ['roll', 'pitch', 'yaw'])
DISTMIN = 0.1

class Bot():

    def __init__(self):
        self.final_goal = Point(10, 10)
        self.path = []
        self.vel_pub = rospy.Publisher('robot0/cmd_vel', Twist, queue_size=10)
        self.odom_sub = rospy.Subscriber('robot0/odom', Odometry, self.odom_update)
        #self.path_sub = rospy.Subscriber('/path', Path, self._update_path)
        self.obstacle_sub = rospy.Subscriber('obstacles', PolyArray, self.obstacle_update)
        self.obstacles = PolyArray()
        self.obstacle_list = []
        self.path_planner = rospy.ServiceProxy('rrt_planner_service', Planner)
        self.position = Point(0, 0, 0)
        self.orientation = Orientation(0, 0, 0)
        self.velocity = Twist()
        self.move_to_goal = False
        self.goal = Point()
        self.rotation_complete = False
        self.rate = rospy.Rate(10)
        
    def odom_update(self, msg):
        ## callback for odom sub
        self.position = msg.pose.pose.position
        self.orientation = Orientation(*tf.transformations.euler_from_quaternion([
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ]))
        self._set_goal()
        self.vel_update()
        self.rate.sleep()


    def vel_update(self):
        ##function for setting the velocity
        
        if self.move_to_goal:
            self.velocity.linear.x = 0.25*(self.goal.x - self.position.x) 
            self.velocity.linear.y = 0.25*(self.goal.y - self.position.y) 
        else:
            self.velocity.linear.x = 0
            self.velocity.linear.y = 0

        self.vel_pub.publish(self.velocity)
        print(self.velocity.linear.x, self.velocity.linear.y)
        #print([(p.x, p.y) for p in self.path])

    def obstacle_update(self, data):
        ## call back for odom sub
        self.obstacles = data.polygons
        for obstacle in self.obstacles:
            obstacle = Polygon([(p.x, p.y) for p in obstacle.points])
            self.obstacle_list.append(obstacle)
        if len(self.path) <= 2 or self.collision_check() == True:
            self.get_path(self.final_goal)

    def _set_goal(self):
        self.path_use = self.path        
        if len(self.path_use) != 0:
            self.next_loc = self.path_use[0]
            print((self.next_loc.x, self.next_loc.y), len(self.path))
            if abs(self.next_loc.x - self.position.x) < DISTMIN and abs(self.next_loc.y - self.position.y) < DISTMIN:
                self.move_to_goal = False
                self.path.pop(0)
            else :
                self.goal = self.next_loc
                self.move_to_goal = True
        elif len(self.path_use) == 0:
            self.get_path(self.final_goal)       
        elif (self.final_goal.x == self.position.x) and (self.final_goal.y == self.position.y):
            print('----Completed Path----')            

    def collision_check(self):
        ## checking collsions in the path
        intersection = 0
        directline = LineString([(p.x, p.y) for p in self.path])
        for obstacle in self.obstacle_list:
            if directline.intersects(obstacle):
                intersection += 1
        if intersection > 0:
            collision = True
        else:
            collision = False
        return collision
        

    def get_path(self, goal):
        ## callback for the planner request
        rospy.wait_for_service('rrt_planner_service')
        try:
            response = PlannerResponse()
            request = PlannerRequest()
            request.start.x = self.position.x
            request.start.y = self.position.y
            request.goal.x = self.final_goal.x
            request.goal.y = self.final_goal.y
            request.obstacle_list = self.obstacles

            response = self.path_planner(request)
            self.path = [Point(p.x, p.y) for p in response.path.points]          
                


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
    bot.final_goal = Point(10, 10)
    rospy.spin()

if __name__ == '__main__':
    main()