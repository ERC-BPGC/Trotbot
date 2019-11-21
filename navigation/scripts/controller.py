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


# TO DO: publish on cmd_vel

# define the function to transform the goal point
class Controller():
    def __init__(self):
        """

        """
        # self.feedback = 
        self.result = False
        self.velocity = Twist()
        
        self.server = actionlib.SimpleActionServer("move_bot", MoveBotAction, self.server_cb, False)  # change the boolean to turn on the server automatically

        self.odom_update = rospy.Subscriber("odom", Odometry, self.update_odom)
        self.vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=5)
        
        rospy.loginfo("...Controller Initialized...")

    def transform_goal(self):
        """

        """
        self.goal[0] = self.goal[0] - self.position.x
        self.goal[1] = self.goal[1] - self.position.y

    def server_cb(self, moveGoal):
        """

        """
        self.goal = moveGoal.goal


    def update_odom(self, data):
        """

        """
        self.position = Point(data.pose.pose.position.x, data.pose.pose.position.y)
        self.orientation = Orientation(
			transformations.euler_from_quaternion([
				data.pose.pose.orientation.x, data.pose.pose.orientation.y, 
				data.pose.pose.orientation.z, data.pose.pose.orientation.w
		]))

        self.transform_goal()

        if self.goal[0] < REACH_DIST and self.goal[1] < REACH_DIST:
            self.result = True
            self.server.set_succeeded(self.result, "Reached Goal Point :)")

        if self.server.is_preempt_requested():
            self.result = False
            self.server.set_preempted(self.result, "Goal Preempt")

        self.set_vel()


    def set_vel(self):
        """

        """
        goal_norm = linalg.norm(self.goal)
        self.velocity.linear.x = self.goal[0] / goal_norm
        self.velocity.linear.y = self.goal[1] / goal_norm

        self.vel_pub.publish(self.velocity)

def main():
    my_controller = Controller()
    rospy.spin()

if __name__ == "__main__":
    rospy.init_node("controller",anonymous=True)
    main()
