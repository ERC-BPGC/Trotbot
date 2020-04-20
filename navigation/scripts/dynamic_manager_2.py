#!/usr/bin/env python

import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import Point, Twist, Pose, PoseStamped, Quaternion, TransformStamped
from nav_msgs.msg import Path, Odometry
from shapely.geometry import LineString, Polygon
from navigation.msg import PointArray, PolyArray
import utils
import math
import numpy as np 
import tf_conversions, tf2_ros

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
        self.obstacle_message = PolyArray()
        self.obstacles = []
        self.position0 = Point(0, 0, 0)
        self.orientation0 = utils.Orientation(0, 0, 0)
    
        

        #Decide upon the msg type of self.obstacles!


        #Init publishers and subscribers

        self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self._odom_callback)
        self.path_pub = rospy.Publisher('/path', Path, queue_size=10)

        #init booleans

        self.goal_reached = False
        self.move_to_goal = False

        #init broadcasters
        self.br_planner = tf2_ros.TransformBroadcaster()
        self.br_bot = tf2_ros.TransformBroadcaster()


    def _odom_callback(self, msg):
        '''
            Callback for odometry
        '''
        self.position, self.orientation = utils.unwrap_pose(msg.pose.pose)

        self.get_obst()
        self._check_path()
        self._set_goal()
        self.broadcast_transforms()


    def get_obst(self):
        ##run this fucntion to update obstacles
        '''
            1. Return obstacle in the form of Polgons?
                or PolyArray??
            2. update self.obstacles
        '''

        self.obstacle_message = output from obst_detect algos

        #Assuming message is PolyArray

        self.obstacles= [[(point.x, point.y) for point in polygon.points] for polygon in self.obstacle_message.polygons]

    def __path_planner(self):

        ##run this function to get path from 
        ##current position

        '''
            1. Store position and orientation at point
                where path planner is called
            2.  Publish Transform from Odom frame to
                position0 and orientation0 (check self.broadcast_transforms)
            3. Return a list of coordinates and store in self.path_points
        '''
        self.position0 = self.position
        self.orientation0 = self.orientation

        self.path_points = output from path_planner

    
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
            '''

            #optimise path
            self.path_points = utils.los_optimizer(self.path_points, self.obstacles)

            #publish path message wrt bot frame (Check this part)
            self.path.header = Header(frame_id='planner')
            self.path.poses = [PoseStamped(pose=Pose(position=Point(p[0], p[1]), orientation = Quaternion(x=0, y=0, z=0, w=1))
                , header = Header(frame_id = 'bot')) for p in self.path_points]
            
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
            ##publish a message to convey to the controller to publish zero vel
            
            print('----------Completed Path----------')


    def check_collision(self):
        """
            Check whether path passes through any obstacle.
        """
        direct_line = LineString(self.path_points)
        for obstacle in self.obstacles:
            if direct_line.intersects(Polygon(obstacle)):
                return True

        return False


    def broadcast_transforms(self):
        
        #init transfroms for planner

        t_planner = TransformStamped()

        t_planner.header.stamp = rospy.Time.now()
        t_planner.header.frame_id = 'odom'
        t_planner.child_frame_id = 'planner'

        t_planner.transform.translation.x = self.position0.x
        t_planner.transform.translation.y = self.position0.y
        t_planner.transform.translation.z = self.position0.z

        qnion_planner = tf_conversions.transformations.quaternion_from_euler(self.orientation0.roll, self.orientation0.pitch, self.orientation0.yaw)

        t_planner.transform.rotation.x = qnion_planner[0]
        t_planner.transform.rotation.y = qnion_planner[1]
        t_planner.transform.rotation.z = qnion_planner[2]
        t_planner.transform.rotation.w = qnion_planner[3]
        
        #init transforms for bot

        t_bot = TransformStamped()

        t_bot.header.stamp = rospy.Time.now()
        t_bot.header.frame_id = 'odom'
        t_bot.child_frame_id = 'bot'

        t_bot.transform.translation.x = self.position.x
        t_bot.transform.translation.y = self.position.y
        t_bot.transform.translation.z = self.position.z

        qnion_bot = tf_conversions.transformations.quaternion_from_euler(self.orientation.roll, self.orientation.pitch, self.orientation.yaw)
        
        t_bot.transform.rotation.x = qnion_bot[0]
        t_bot.transform.rotation.y = qnion_bot[1]
        t_bot.transform.rotation.z = qnion_bot[2]
        t_bot.transform.rotation.w = qnion_bot[3]

        self.br_planner.sendTransform(t_planner)
        self.br_bot.sendTransform(t_bot)

if __name__ == '__main__':
    rospy.init_node('dynamic_manager_2.0')
    rate = rospy.Rate(10)

    manager = Manager()
    manager.final_goal = Point(3, 3, 0) # final goal (get from global planner)
    rospy.spin()