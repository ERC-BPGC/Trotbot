#! /usr/bin/env python
import rospy
import actionlib
import math
import utils
import collections
import shapely

from shapely.geometry import Point, LineString

from utils import Orientation, REACH_DIST
from geometry_msgs.msg import Point32
from nav_msgs.msg import Odometry
from navigation.msg import PolyArray, PointArray, MoveBotAction, MoveBotGoal
from navigation.srv import Planner, PlannerRequest, PlannerResponse

class Manager():
    def __init__(self):
        self.position = Point(0, 0)
        self.orientation = Orientation(0, 0, 0)
        self.current_goal_point = Point(0, 0)
        self.obstacles = []
        self.path = collections.deque()
        
        self.plan_path = rospy.ServiceProxy('rrt_planner_service', Planner)
        self.odometry_sub = rospy.Subscriber("odom", Odometry, self.__odom_update)
        self.obstacle_sub = rospy.Subscriber("obstacles", PolyArray, self.__obstacle_update)
        self.controller_client = actionlib.SimpleActionClient('move_bot', MoveBotAction)
        
        rospy.loginfo("...Manager Initialized...")
        self.controller_client.wait_for_server()


    def go_to(self, goal_point):
        self.current_goal_point = goal_point
        # print("current goal is:", goal_point.x, goal_point.y)
        self.__call_path_planner(self.current_goal_point)
        
        while len(self.path) is not 0:
            self.__move_to_next_point()
            
    def __move_to_next_point(self):
        next_point = self.path[0]
        goal_for_controller = MoveBotGoal()
        if type(next_point) == type((0,0)):
            goal_for_controller.goal.x = next_point[0]
            goal_for_controller.goal.y = next_point[1]
        elif type(next_point) == type(Point(0,0)):
            goal_for_controller.goal.x = next_point.x
            goal_for_controller.goal.y = next_point.y

        self.controller_client.send_goal(goal_for_controller, feedback_cb=self.__next_point_reached)
        self.controller_client.wait_for_result()
        
    def __next_point_reached(self, _, done):
        if done.ack:
            self.path.popleft()
            
    def __odom_update(self, data):
        self.position, self.orientation = utils.unwrap_pose(data.pose.pose)
        if len(self.path) > 1:
            self.path = collections.deque(utils.transform(
                LineString(self.path), self.position, self.orientation).coords)
            self.current_goal_point = utils.transform(self.current_goal_point, self.position, self.orientation)
        
        if self.current_goal_point.x < REACH_DIST and self.current_goal_point.y < REACH_DIST:
            return  # TODO: Add completion mech
    
    def __obstacle_update(self, data):
        self.obstacles = [[(point.x, point.y) for point in polygon.points] for polygon in data.polygons]
        
        if len(self.path) < 2 or not utils.check_intersection(self.path, self.obstacles):
            self.__call_path_planner(self.current_goal_point)
            
    def __call_path_planner(self, goal_point):
        # rospy.logwarn("Waiting for Service")
         
        rospy.wait_for_service("rrt_planner_service")
        
        try:
            response = PlannerResponse()
            request = PlannerRequest()
            request.start.x, request.start.y = 0, 0
            request.goal.x, request.goal.y = list(goal_point.coords)[0]
            
            rospy.loginfo("Request Goal is:%d,%d", request.goal.x, request.goal.y)
            
            request.obstacle_list.polygons = [PointArray([Point32(x=p[0], y=p[1]) for p in o]) for o in self.obstacles]
            response = self.plan_path(request)

            rospy.loginfo("Planned path")

            if response.ack:
                self.path = collections.deque([Point(pt.x, pt.y) for pt in response.path.points])
            else:
                print("Failed to compute path!")
                
        except rospy.ServiceException as e:
            print("Service call failed: %s", e)


def main():
    bot = Manager()
    local_goal = Point(2, 0)  # Will come from global planner when complete
    bot.go_to(local_goal)
    
if __name__ == "__main__":
    rospy.init_node("manager", anonymous=True)
    
    # try:
    main()
    rospy.spin()

    rospy.logwarn("Killing dynamic manager!")

    # except Exception as err:
        # rospy.loginfo("%s was thrown",err)
