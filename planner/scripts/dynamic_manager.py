#!/usr/bin/env python

try :
    import rospy 
    from nav_msgs.msg import Odometry, Path
    from geometry_msgs.msg import Point, PoseStamped, Quaternion, Pose, Point32, PointStamped, Twist
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import Header
    import tf2_ros
    import tf2_geometry_msgs
    from navigation.msg import PointArray, PolyArray
    
except:
    raise ImportError

try:
    import gennav
    from gennav.planners.rrt import RRT, Node
    from gennav.planners.samplers import uniform_adjustable_random_sampler as sampler
    from gennav.utils.planner import los_optimizer as path_optimiser
    
except ImportError:
    from local.RRT import RRT, Node
    from local.utils import adjustable_random_sampler as sampler
    from local.utils import los_optimizer as path_optimiser

    rospy.logwarn("Cannot import from GENNAV")
    rospy.logwarn("Importing locally")

import numpy as np
from shapely.geometry import LineString, Polygon
import shapely
import time

OBST_THRES = 0.25
BOTFRAME = 'base_link'
WORLDFRAME = 'world'

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
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        #Flag to check if final goal is reached
        self.not_reached = True

        #Initialising path planner
        self.path_planner = RRT(sample_area=(-5, 15), sampler=sampler, expand_dis=0.1)
        self.path_points = []


        #Initialise transform listener
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer) 


        self.rate = rospy.Rate(10)
        
    def __odom_update(self, msg):
        '''
            Callback function for odometry
        '''
        self.position = Point(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z)

        # Obtain Transform
        try:
            self.transform = self.tfBuffer.lookup_transform(WORLDFRAME, BOTFRAME, rospy.Time())
            self.transform_odom_to_base = self.tfBuffer.lookup_transform(BOTFRAME, WORLDFRAME, rospy.Time())
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
        self.not_reached = True
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
        if self.goal_recieved and not self.goalNotReachable():
            try:
                if abs(self.position.x - self.goal.x) < 0.1 and abs(self.position.y - self.goal.y) < 0.1:

                    # if final goal is reached and a new goal is not published, mark final goal reached
                    self.not_reached = False
            except Exception as err:
                rospy.logfatal(err)
                rospy.logfatal('Condition reached for final goal')

            if (len(self.path_points) == 0 and self.not_reached):

                # if no path is planned and the goal is not reached call path planner
                try:
                    rospy.loginfo('Path Planner Called from %f, %f to %f, %f', self.position.x, self.position.y, self.goal.x, self.goal.y)
                    self.plan_path()
                    rospy.loginfo('Path planned from %f, %f to %f, %f', self.position.x, self.position.y, self.goal.x, self.goal.y)
                    self.publish_path()
                    
                except Exception as err:
                    rospy.logerr(err)

            elif self.collision():

                # if there is any collision detected, replan the path accordingly from current position
                # to the goal position
                try:
                    rospy.logwarn('Path Planner Called due to expected collision')
                    self.plan_path()
                    rospy.loginfo('Path planned from %f, %f to %f, %f', self.position.x, self.position.y, self.goal.x, self.goal.y)
                    self.publish_path()
                    
                except Exception as err:
                    print(err)

        elif self.goalNotReachable():

            # Stop random replanning of path and stop the bot and make it wait till it cleared
            rospy.logerr("Goal not reachable. Stopped publishing path")
            self.vel_pub.publish(Twist())            
            self.path_points = []

        

    def publish_path(self):
        '''
            Publishes the path given out by the path planner
        '''
        poses = []
        self.path = Path()
        self.path.header = Header(frame_id = WORLDFRAME)
        for point in self.path_points:
            pose = PoseStamped(header = Header(frame_id = WORLDFRAME, stamp = rospy.Time.now()), pose = Pose(position = Point(point[0], point[1], 0), orientation = Quaternion(0, 0, 0, 1)))
            poses.append(pose)
        self.path.poses = poses

        self.path_pub.publish(self.path)
    
    def plan_path(self):
        '''
            A function to take care of the call to path planner
        '''
        path, _ = self.path_planner.plan((self.position.x, self.position.y), (self.goal.x, self.goal.y), self.obstacles)
        optimised_path = path_optimiser(path, self.obstacles)
        self.path_points = optimised_path

        # for debugging
        print(self.path_points)

    ### ----------------------------- Functions for Obstacle Detection-----------------------------###

    def __obst_sub(self, msg):
        # get obstacles in bot frame and convert them to odom frame
        self.obstacles = []
        obstacles = msg.polygons 
        for obst in obstacles:
            self.obstacles.append([(self.transformPoint(p).x, self.transformPoint(p).y) for p in obst.points])
           


    ###--------------------------------------Dynamic Collision Checking--------------------------------------------------###

    # check by using shapely module
    def collision(self):
        '''
            Checks for collision in between keeping into robots bodyframe into account
            Args:  
                Safety Parameter: (float) adds an extra safety distance between the robot and the obstacles
            Returns:
                Collision : (bool) Whether a collision is detected or not
        '''
        path_modified = []
        for i in range(len(self.path_points)):
            if i == len(self.path_points) - 1:
                p1, p2 = self.expand_points(self.path_points[i], self.path_points[i-1], 0)
            else:
                p1, p2 = self.expand_points(self.path_points[i], self.path_points[i+1], 0)

            path_modified.insert(i, p2)
            path_modified.insert(-i-1, p1)

        for obst in self.obstacles:
            if len(self.path_points) >=2:
                if LineString(path_modified).intersects(Polygon(obst)):
                    return True
        else:
            return False  
    
    def goalNotReachable(self):
        '''
            Function returns if goal is not reachable due to obstacle blockage
            Note : Further work needed
        '''
        for obst in self.obstacles:
            if shapely.geometry.Point(self.goal.x, self.goal.y).within(Polygon(obst)):
                True
        else:
            return False
    
    def expand_points(self, point1, point2, r):
        """
            Returns two points which are perpendicular to point1 with a distance of r
            Args:
                point1 = (tuple) point1
                point2 = (tuple) point2 
                r = (float) safety distance
        """

        theta = np.deg2rad(np.arctan2(point2[1] - point1[1], point2[0] - point1[0]))
        p1 = point1[0] + (r * np.cos(90 +  theta)), point1[1] + (r * np.sin(90 + theta))
        p2 = point1[0] + (-r * np.cos(90 + theta)), point1[1] + (-r * np.sin(90 + theta))
        return p1, p2

    ###--------------------------------------Transforms---------------------------------------------------------###

    # for now everything is being transformed into world frame
    # also consider converting everything to the bot frame

    def transformPoint(self, point):
        pointStamped = PointStamped(header = Header(frame_id='base_footprint'), point = Point(point.x, point.y, point.z))
        pointTransformed = tf2_geometry_msgs.do_transform_point(pointStamped, self.transform)
        return pointTransformed.point

if __name__ == '__main__':

    # Initialise node
    rospy.init_node('dynamicManager')
    rate = rospy.Rate(10)
    
    # Manager Instance
    manager = Manager()

    rospy.loginfo('Manager Initiated')
    rospy.spin()


    