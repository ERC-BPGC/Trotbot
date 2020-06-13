#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist, Point, Quaternion, PoseStamped
from tf.transformations import euler_from_quaternion as efq 

import numpy as np 
from collections import namedtuple

Orientation = namedtuple('Orientation', ['roll', 'pitch', 'yaw'])
Gains = namedtuple('Gains', ['kp', 'kd', 'ki'])

DISTMIN = 0.1
MAXX = 0.22
MAXANG = 2
DIST_THRES = 0.1


class Controller():
    '''
        Main controller class
    '''

    def __init__(self):
        
        # Define the messages
        self.velocity = Twist()
        self.position = Point()
        self.orientation = Orientation(0, 0, 0)
        self.path = Path()
        self.path_points = []
        self.current_index = 0
        self.nextWay = None

        self.goal = None
        self.goal_reached = False
        self.new_goal_recieved = False
        
        # Initialize subs and pubs
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.path_sub = rospy.Subscriber('/path', Path, self.__path_sub)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.__odom_sub)
        

        # Parameters for PID tuning
        self.x_gain = Gains(1, 0, 0)
        self.ang_gain = Gains(0.3, 0, 0)
        self.xdiff, self.angdiff, self.xintegral, self.angintegral = 0, 0, 0, 0

    def __odom_sub(self, msg):

        # Calbacck function for odomentry
        self.position = msg.pose.pose.position
        quaternion = msg.pose.pose.orientation
        orientatioN = efq([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        self.orientation = Orientation(orientatioN[0], orientatioN[1], orientatioN[2])
        
          
        
    def __path_sub(self, msg):

        #Callback for path 
        self.path = msg
        poses = self.path.poses
        self.path_points = [Point(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z) for pose in poses]

        # Run the set goal function
        self._set_goal()

    def _set_goal(self):
        '''
            Function that sets goal for controller to move to 
            i.e. the next way point 
        '''
        try:
            self.final_goal = self.path_points[-1]
            if np.sqrt((self.final_goal.x - self.position.x)**2 + (self.position.y - self.final_goal.y)**2) < 0.1:
                rospy.loginfo('final_goal_reached')
                self.goal_reached = True
                self.move_to_goal = False
                self.current_index = 0  

            if len(self.path_points) != 0 and not self.goal_reached:
                self.nextWay = self.path_points[self.current_index]
                print((self.nextWay.x, self.nextWay.y), self.current_index)
                if abs(self.nextWay.x - self.position.x) < DISTMIN and abs(self.nextWay.y - self.position.y) < DISTMIN:
                    self.move_to_goal = False                    
                    self.current_index += 1
                    print('popped')
                else :
                    self.goal = self.nextWay
                    self.move_to_goal = True
                    self._move_bot()
                    print('next goal set')
            elif self.goal_reached:
                self.velocity.linear.x, self.velocity.angular.z = 0, 0
                self.vel_pub.publish(self.velocity)
                print('-------Completed Path-------')

        except Exception as err:
            rospy.logwarn(err)
        
        

    def _move_bot(self):
        '''
            handles all the velocity commands to be published
        '''
        # Calculate all errors
        ang_goal = np.arctan2(self.goal.y - self.position.y, self.goal.x - self.position.x)
        x_error = np.sqrt((self.goal.x - self.position.x) ** 2 + (self.goal.y - self.position.y) ** 2)
        ang_error = ang_goal - self.orientation.yaw

        # A simple PID Controller, currently tuned by using just Proportional gain
            
        if self.move_to_goal:
            
            x_error = np.sqrt((self.goal.x - self.position.x) ** 2 + (self.goal.y - self.position.y) ** 2)
            ang_error = ang_goal - self.orientation.yaw
    
            if abs(ang_error) < DIST_THRES:
                ang_vel = self.ang_gain.kp * ang_error + self.ang_gain.kd * self.angdiff + self.ang_gain.ki * self.angintegral
                lin_vel = self.x_gain.kp * x_error + self.x_gain.kd * self.xdiff + self.x_gain.ki * self.xintegral
                ang_vel = max(-MAXANG, min(ang_vel, MAXANG))
                lin_vel = max(0, min(lin_vel, MAXX))

                self.xdiff = x_error - self.xdiff
                self.xintegral += x_error

                self.angdiff = ang_error - self.angdiff
                self.angintegral += ang_error
                
            else:
                ang_vel = self.ang_gain.kp * ang_error + self.ang_gain.kd * self.angdiff + self.ang_gain.ki * self.angintegral
                ang_vel = max(-MAXANG, min(ang_vel, MAXANG))
                self.angdiff = ang_error - self.angdiff
                self.angintegral += ang_error
                lin_vel = 0
                
            
            self.velocity.linear.x = lin_vel
            self.velocity.angular.z = ang_vel
            self.vel_pub.publish(self.velocity)
        else:
        
            self.velocity.linear.x = 0
            self.velocity.angular.z = 0
            self.vel_pub.publish(self.velocity)
            print('Waypoint Reached')


if __name__ == '__main__':
    # Instance of Controller
    rospy.init_node('TurtleBotController')
    rate = rospy.Rate(10)
    rospy.loginfo('Controller Initiated')
    controller = Controller()

    rospy.spin()
        

    


