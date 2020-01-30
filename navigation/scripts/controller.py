#!/usr/bin/env python


import rospy
import actionlib

import shapely
from shapely.geometry import Point, LineString
import math
import numpy.linalg as linalg
import utils
import collections
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from utils import Orientation, REACH_DIST

from navigation.msg import MoveBotAction, MoveBotGoal, MoveBotResult, MoveBotFeedback


# define the function to transform the goal point
class Controller():

    def __init__(self):
        """
            Initializes the controller class.
        """
        # self.feedback = 
        self.position = Point(0, 0) 
        self.orientation = Orientation(0, 0, 0)
        self.result = False
        self.velocity = Twist()
        self.goal = Point(0, 0)
        self.request_received = False

        self.server = actionlib.SimpleActionServer("move_bot", MoveBotAction, self.__set_goal, False)  # change the boolean to turn on the server automatically

        self.odom_update = rospy.Subscriber("odom", Odometry, self.__odom_update)
        self.vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=5)

        self.rate = rospy.Rate(100)
        rospy.loginfo("...Controller Initialized...")
        
        self.server.start()


    def __set_goal(self, moveGoal):
        """
            Callback function for the action server
            Updates the goal location
            Arg: /navigation/MoveGoal
        """
        self.request_received = True
        self.goal = Point(moveGoal.goal.x, moveGoal.goal.y, 0)

        while not self.goal.x < REACH_DIST and self.goal.y < REACH_DIST:
            if self.request_received and self.goal.x < REACH_DIST and self.goal.y < REACH_DIST:
                self.result = True
                self.server.set_succeeded(self.result, "Reached Goal Point :)")

            if self.server.is_preempt_requested():
                self.result = False
                self.server.set_preempted(self.result, "Goal Preempt")


    def __odom_update(self, data):
        """
            Callback function for odom update
        """

        self.position, self.orientation = utils.unwrap_pose(data.pose.pose)
        self.goal = utils.transform(Point(self.goal), self.position, self.orientation)

        self.__set_velocity()

        self.rate.sleep()


    def __set_velocity(self):
        """
            Publishes velocity on /cmd_vel
            Simple proportional logic is used to generate the velocities
        """
        goal_norm = linalg.norm(self.goal)
        
        self.velocity.linear.x = self.goal.x / goal_norm
        self.velocity.linear.y = self.goal.y / goal_norm

        # rospy.loginfo("Publishing velocity")
        # self.vel_pub.publish(self.velocity)


def main():
    my_controller = Controller()

    rospy.spin()

if __name__ == "__main__":
    rospy.init_node("controller", anonymous=True)

    try:
        main()

        rospy.logwarn("Killing controller!")

    except Exception as err:
        rospy.loginfo("%s was thrown",err)