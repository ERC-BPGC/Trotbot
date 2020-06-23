#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist, Point, Quaternion, PoseStamped
from tf.transformations import euler_from_quaternion as efq 

try:
    from gennav.controllers import OmniWheelPID
except ImportError:
    rospy.logwarn("Unable to import from GENNAV")
import numpy as np 
from collections import namedtuple

Orientation = namedtuple('Orientation', ['roll', 'pitch', 'yaw'])
Gains = namedtuple('Gains', ['kp', 'kd', 'ki'])

DISTMIN = 0.1
MAXX = 1
MAXY = 1

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
        self.nextWay = None
        self.goal_reached = False

        self.goal = None
        self.current_index = 0
        self.move_to_goal = False
        self.final_goal = None

        # Initialize subs and pubs
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.path_sub = rospy.Subscriber('/path', Path, self.__path_sub)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.__odom_sub)
        self.goal_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.__goal_sub)
        # Parameters for PID tuning
        self.x_gain = Gains(0.25, 0 , 0)
        self.y_gain = Gains(0.25, 0, 0)

        self.xdiff, self.ydiff, self.xintegral, self.yintegral = 0, 0, 0, 0

        rospy.loginfo("Controller initiated")

    def __odom_sub(self, msg):
        self.position = msg.pose.pose.position
        quaternion = msg.pose.pose.orientation
        orientatioN = efq([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        self.orientation = Orientation(orientatioN[0], orientatioN[1], orientatioN[2])
        
        self.set_goal()
        
    def __path_sub(self, msg):
        self.path = msg
        poses = self.path.poses
        self.path_points = [Point(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z) for pose in poses]
        self.current_index = 0

        rospy.loginfo("New Path recieved")        

    def __goal_sub(self, msg):

        self.goal_reached = False
        self.final_goal = msg.pose.position
        self.current_index = 0

        rospy.loginfo("New Goal Set")        

    def set_goal(self):
        '''
            Sets next waypoint fot the bot to travel to
        '''
        if self.final_goal is not None:
            try:
                if np.sqrt((self.final_goal.x - self.position.x)**2 + (self.position.y - self.final_goal.y)**2) < 0.1:
                    rospy.loginfo_once('final_goal_reached')
                    self.goal_reached = True
                    self.move_to_goal = False
                    self.current_index = 0  

                if len(self.path_points) != 0 and not self.goal_reached:
                    self.nextWay = self.path_points[self.current_index]
                    # print(self.current_index)
                    if abs(self.nextWay.x - self.position.x) < DISTMIN and abs(self.nextWay.y - self.position.y) < DISTMIN:
                        self.move_to_goal = False                    
                        self.current_index += 1
    
                    else :
                        self.goal = self.nextWay
                        self.move_to_goal = True
                        self._move_bot()

                elif self.goal_reached:
                    self.velocity.linear.x, self.velocity.linear.y = 0, 0
                    self.vel_pub.publish(self.velocity)
                    rospy.loginfo_once('-------Completed Path-------')
                    rospy.loginfo_once('-------Waiting for new goal------')

            except Exception as err:
                rospy.logwarn_once(err)
                self.vel_pub.publish(Twist())
        
        
    def vel_constraint(self, velocity, dir):
        '''
            sets tyhe velocity constraints
        '''
        if dir.lower() == 'x':
            vel_param = MAXX            
        elif dir.lower() == 'y':
            vel_param = MAXY 
           
        if velocity > vel_param:
            velocity = vel_param
        elif velocity < - vel_param:
            velocity = -vel_param
        else:
            velocity = velocity

        return velocity

    def _move_bot(self):
        '''
            handles all the velocity commands to be published
        '''
        x_error = self.nextWay.x - self.position.x
        y_error = self.nextWay.y - self.position.y        

        if self.move_to_goal:

            x_error = self.nextWay.x - self.position.x
            y_error = self.nextWay.y - self.position.y

            velx = self.x_gain.kp * x_error + self.x_gain.kd * self.xdiff + self.x_gain.ki * self.xintegral
            vely = self.y_gain.kp * y_error + self.y_gain.kd * self.ydiff + self.y_gain.ki * self.yintegral

            velx, vely = self.vel_constraint(velx, 'x'), self.vel_constraint(vely, 'y')

            self.xdiff = x_error - self.xdiff
            self.ydiff = y_error - self.ydiff

            self.xintegral += x_error
            self.yintegral += y_error
            
            self.velocity.linear.x = velx
            self.velocity.linear.y = vely

            self.vel_pub.publish(self.velocity)
            # print('vel published')
            # print("x_error: ", x_error)
            # print("y_error: ", y_error)

        else:
        
            self.velocity.linear.x = 0
            self.velocity.linear.y = 0
            self.vel_pub.publish(self.velocity)
            print('Waypoint Reached')

    def shutdown(self):

        rospy.loginfo("Controller Node Shutdown!!")
        rospy.loginfo("reducing velocity to Zero")
        self.velocity.linear.x = 0
        self.velocity.linear.y = 0
        
        self.vel_pub.publish(self.velocity)


if __name__ == '__main__':
    rospy.init_node('omnicontroller')
    rate = rospy.Rate(10)

    controller = Controller()
    

    rospy.spin()
        
    controller.shutdown()
    


