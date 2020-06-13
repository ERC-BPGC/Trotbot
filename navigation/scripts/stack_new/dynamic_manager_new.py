#!/usr/bin/env python

try :
    import rospy 
    from nav_msgs.msg import Odometry, Path
    from geometry_msgs.msg import Point, PoseStamped, Quaternion, Pose, Point32, PointStamped
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import Header
    import tf2_ros
    import tf2_geometry_msgs
    from navigation.msg import PointArray, PolyArray
    
except:
    raise ImportError

import numpy as np
from RRT import RRT, Node
from utils import adjustable_random_sampler as sampler
from utils import los_optimizer as path_optimiser
from shapely.geometry import LineString, Polygon
import shapely
import time

OBST_THRES = 0.25
BOTFRAME = 'base_footprint'
ODOMFRAME = 'odom'

class Manager():
    '''
        Main manager class that takes care of calling the path planner
        Also takes care of updating the nextWay point for the bot to move to in the path
    '''

    def __init__(self):

        #Initialising messages and variables
        self.position = Point()
        self.path = Path()
        self.obstacles = []
        self.start = Point()
        self.goal = Point()
        self.scan = LaserScan()
        self.goal_recieved = False
        

        #Publishers and Subscribers
        self.path_pub = rospy.Publisher('/path', Path, queue_size=10)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.__odom_update)      
        #self.scan_pub = rospy.Subscriber('/scan', LaserScan, self.__laser_sub)
        self.goal_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.__goal_sub)  
        self.obst_sub = rospy.Subscriber('/obstacles', PolyArray, self.__obst_sub)

        #Flag to check if final goal is reached
        self.not_reached = True

        #Initialising path planner
        self.path_planner = RRT(sample_area=(-5, 15), sampler=sampler, expand_dis=0.1)
        self.path_points = []


        #Initialise transform listener
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer) 
        rospy.sleep(1)       
        self.rate = rospy.Rate(10)
        
    def __odom_update(self, msg):
        '''
            Callback function for odometry
        '''
        self.position = Point(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z)

        # Obtain Transform
        try:
            self.transform = self.tfBuffer.lookup_transform(ODOMFRAME, BOTFRAME, rospy.Time())
            self.transform_odom_to_base = self.tfBuffer.lookup_transform(BOTFRAME, ODOMFRAME, rospy.Time())
        except:
            rospy.logwarn('Waiting for Transforms')
        # call get path function
        self.get_path()
        #print('path points:' , self.path_points)
    
    def __goal_sub(self, msg):
        '''
            Callback to recieve goal from RVIZ
        '''
        self.goal = Point(msg.pose.position.x, msg.pose.position.y, 0)
        self.goal_recieved = True
        print(self.goal)
        try:
            rospy.loginfo('Path Planner Called from %f, %f to %f, %f', self.position.x, self.position.y, self.goal.x, self.goal.y)
            self.plan_path()
            rospy.loginfo('Path planned from %f, %f to %f, %f', self.position.x, self.position.y, self.goal.x, self.goal.y)
            self.publish_path()
                    
        except Exception as err:
            rospy.logerr(err)

###---------------------------------------Navigation Functions----------------------------------------###       
    def get_path(self):
        '''
            Funtions that checks necessary conditions and calls path
        '''
        if self.goal_recieved:
            try:
                if abs(self.position.x - self.goal.x) < 0.1 and abs(self.position.y - self.goal.y) < 0.1:
                    self.not_reached = False
            except:
                pass
            if (len(self.path_points) == 0 and self.not_reached):
                try:
                    rospy.loginfo('Path Planner Called from %f, %f to %f, %f', self.position.x, self.position.y, self.goal.x, self.goal.y)
                    self.plan_path()
                    #rospy.loginfo('Path planned from %f, %f to %f, %f', self.position.x, self.position.y, self.goal.x, self.goal.y)
                    self.publish_path()
                    
                except Exception as err:
                    rospy.logerr(err)
            elif self.collision():
                try:
                    rospy.logwarn('Path Planner Called due to expected collision')
                    self.plan_path()
                    rospy.loginfo('Path planned from %f, %f to %f, %f', self.position.x, self.position.y, self.goal.x, self.goal.y)
                    self.publish_path()
                    
                except Exception as err:
                    print(err)
            
            elif self.not_reached:
                self.publish_path()
                
            else :
                self.publish_path()
                rospy.loginfo("-"*10 + 'End-Reached' + '-'*10)
        

    def publish_path(self):
        '''
            Publishes the path given out byt the path planner
        '''
        poses = []
        self.path = Path()
        self.path.header = Header(frame_id = 'odom')
        for point in self.path_points:
            pose = PoseStamped(header = Header(frame_id = 'odom', stamp = rospy.Time.now()), pose = Pose(position = Point(point[0], point[1], 0), orientation = Quaternion(0, 0, 0, 1)))
            poses.append(pose)
        self.path.poses = poses

        self.path_pub.publish(self.path)
    
    def plan_path(self):
        path, _ = self.path_planner((self.position.x, self.position.y), (self.goal.x, self.goal.y), self.obstacles)
        optimised_path = path_optimiser(path, self.obstacles)
        self.path_points = optimised_path
        print(self.path_points)

    ### ----------------------------- Functions for LaserScan Processing-----------------------------###

    def __obst_sub(self, msg):
        # get obstacles in base_footprint frame and convert them to odom frame
        self.obstacles = []
        obstacles = msg.polygons 
        for obst in obstacles:
            self.obstacles.append([(self.transformPoint(p).x, self.transformPoint(p).y) for p in obst.points])
           


    ###--------------------------------------Dynamic Checking--------------------------------------------------###
    def collision(self):
        #print('checking collision')
        for obst in self.obstacles:
            if len(self.path_points) >=2:
                point1 = self.path_points[0]
                point2 = self.path_points[1]
                line = LineString([point1, point2])
                if line.intersects(Polygon(obst)):
                    return True
        else:
            return False  

    ###--------------------------------------Transforms---------------------------------------------------------###
    def transformPoint(self, point):
        pointStamped = PointStamped(header = Header(frame_id='base_footprint'), point = Point(point.x, point.y, point.z))
        pointTransformed = tf2_geometry_msgs.do_transform_point(pointStamped, self.transform)
        return pointTransformed.point

if __name__ == '__main__':
    # Initialise node
    rospy.init_node('dynManager')
    rate = rospy.Rate(10)
    
    # Manager Instance
    manager = Manager()

    rospy.loginfo('Manager Initiated')
    rospy.spin()


    