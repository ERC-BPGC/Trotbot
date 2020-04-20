#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Path, Odometry
from shapely.geometry import LineString, Polygon
from navigation.msg import PointArray, PolyArray
import utils
import math
import numpy as np 

DISTMIN = 0.01 # parameter for tuning

class Manager():
    
    def __init__(self):

        #Init messages
        self.position = Point()
        self.goal = Point()
        self.final_goal = Point()
        self.path = Path()
        self.path_points = []
        self.odometry = Odometry()
        self.orientation = utils.Orientation(0, 0, 0)
        self.obstacles = PolyArray()

        #Init publishers and subscribers

        self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self._odom_callback)
        self.path_pub = rospy.Publisher('/path', Path, queue_size=10)

        #init booleans

        self.goal_reached = False
        self.move_to_goal = False


    def _odom_callback(self, msg):
        self.position, self.orientation = utils.unwrap_pose(msg.pose.pose)

        self.get_obst()
        self._check_path()
        self._set_goal()

    def get_obst(self):
        ##run this fucntion to update obstacles
        '''
            1. Return obstacle in the form of Polgons?
                or PolyArray??
            2. update self.obstacles
        '''
        pass


    def __path_planner(self):
        ##run this function to get path from 
        ##current position

        '''
            1. Store position and orientation at point
                where path planner is called
            2.  Publish Transform from Odom frame to
                position0 and orientation0
            3. Return a list of coordinates
        '''
        pass 
    
    def _check_path(self):

        '''
            Path must not contain the present location
            i.e assume path to be starting from 0, 0
        '''

        if (len(self.path_points) < 1 or not self.check_collision()) and not self.goal_reached:
            self.__path_planner()
        
        if len(self.path_points) >= 1:

            '''
                1. Transform path wrt to Bot frame
                2. Check for path optimisation
                3. Publish transfomed and optimised path message
                4. Convert the path to coordinate tuples or Point
                    for further manipulation
            '''
            self.path_pub.publish(self.path)

    def _set_goal(self):
        '''
            Returns the goal coordinate wrt to bot frame
            which can be published to the controller
        '''

        #Condition for final goal 

        if abs(self.final_goal.x) < DISTMIN and abs(self.final_goal.y) < DISTMIN:
            self.goal_reached = True
        
        if len(self.path_points) !=0 and not self.goal_reached:
            self.next_loc = self.path_points[0]
            if abs(self.next_loc.x) < DISTMIN and abs(self.next_loc.y) < DISTMIN:
                self.move_to_goal = False
                self.path_points.pop(0)
                ## remove next_loc from self.path message
            else:
                self.goal = self.next_loc
                self.move_to_goal = True

        elif self.goal_reached:
            ##publish a message to convey to the controller
            ## to publish zero vel
            print('----------Completed Path----------')


    def check_collision(self):
        ''' 
            Function to check collision with dynamic obstacles
        '''
        pass