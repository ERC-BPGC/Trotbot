#!/usr/bin/env python

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler
from obstacle_expander.msg import Exp_msg

rospy.init_node("local_rviz")

def path_update(data):
    final_path=Path()
    final_path.header.frame_id="odom"

    for i in data.bliss:
        posing=PoseStamped()
        posing.header.frame_id="odom"
        posing.pose.position.x=i.x
        posing.pose.position.y=i.y
        posing.pose.position.z=i.z
        final_path.poses.append(posing)

        # while not rospy.is_shutdown():
    pub.publish(final_path)

path_sub = rospy.Subscriber("final_path", Exp_msg,path_update)
pub=rospy.Publisher("local_rviz_path",Path,queue_size=1)
rospy.spin()


# rospy.spin()
