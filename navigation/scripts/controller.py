#!/usr/bin/env python


import rospy
import actionlib
from tf import transformations
import shapely
from shapely.geometry import Point, LineString
import math
import numpy.linalg as linalg
import utils
import collections
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

from navigation.msg import MoveBotAction, MoveBotGoal, MoveBotResult, MoveBotFeedback

Orientation = collections.namedtuple('Orientation', ['roll', 'pitch', 'yaw'])
REACH_DIST = 0.01


# define the function to transform the goal point
class Controller():
    def __init__(self):
        """
            Initializes the controller class.
        """
        self.feedback = MoveBotFeedback()
        self.result = MoveBotResult()
        self.velocity = Twist()
        
        self.result.ack = False
        self.feedback.ack = False

        self.goal = None

        self.server = actionlib.SimpleActionServer('move_bot', MoveBotAction, self.server_cb, True)  # change the boolean to turn on the server automatically

        self.odom_update = rospy.Subscriber("odom", Odometry, self.update_odom)
        self.vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=5)
        
        rospy.loginfo("...Controller Initialized...")


    def transform_goal(self):
        """
            Transform the goal point after getting the odom update
        """
        print(self.goal)
        goal = Point(self.goal.x,self.goal.y)

        goal = shapely.affinity.translate(goal, -self.position.x, -self.position.y)
        goal = shapely.affinity.rotate(goal, angle=math.degrees(self.orientation.yaw), origin=(0, 0))
        self.goal.x = goal.x
        self.goal.y = goal.y


    def server_cb(self, moveGoal):
        """
            Callback function for the action server
            Updates the goal location

            Arg: /navigation/MoveGoal
        """
        self.goal = moveGoal.goal


    def update_odom(self, data):
        """
            Callback function for odom update
        """
        self.position = Point(data.pose.pose.position.x, data.pose.pose.position.y)

        self.orientation = Orientation(
			*transformations.euler_from_quaternion(
                [data.pose.pose.orientation.x, data.pose.pose.orientation.y,
                 data.pose.pose.orientation.z, data.pose.pose.orientation.w]))

        self.transform_goal()

        if self.goal.x < REACH_DIST and self.goal.y < REACH_DIST:
            self.result.ack = True
            self.feedback.ack = True
            self.server.set_succeeded(self.result, "Reached Goal Point :)")

        if self.server.is_preempt_requested():
            self.result.ack = False
            self.feedback.ack = False
            self.server.set_preempted(self.result, "Goal Preempt")

        self.server.publish_feedback(self.feedback)

        self.set_vel()


    def set_vel(self):
        """
            Publishes velocity on /cmd_vel
            Simple proportional logic is used to generate the velocities

        """
        goal_norm = linalg.norm([self.goal.x,self.goal.y])
        self.velocity.linear.x = self.goal.x / goal_norm
        self.velocity.linear.y = self.goal.y / goal_norm

        self.vel_pub.publish(self.velocity)

def main():
    my_controller = Controller()
    rospy.spin()

if __name__ == "__main__":
    rospy.init_node("controller",anonymous=True)
    try:
        main()

    except Exception as err:
        rospy.loginfo("%s was thrown",err)
