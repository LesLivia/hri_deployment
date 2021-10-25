#!/usr/bin/env python
import rospy
import sys
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import time

def ttb3_cmd_nav(speed=0.01):
    #pub = rospy.Publisher('move_base/goal', MoveBaseActionGoal, queue_size=10)
    #pub_plan = rospy.Publisher('move_base/NavfnROS/plan', Path, queue_size=10)
    rospy.init_node('ttb3_cmd_nav', anonymous=False)
    rospy.loginfo(speed)
    rate = rospy.Rate(10)
    count = 0
    move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction) 
    move_base.wait_for_server()

    goal_msg = MoveBaseGoal()

    goal_msg.target_pose.header.frame_id = "map"
    goal_msg.target_pose.header.stamp = rospy.Time.now()
    goal_msg.target_pose.pose.position.x = 0.26636081934
    goal_msg.target_pose.pose.position.y = -0.642632722855
    goal_msg.target_pose.pose.orientation.w = 0.691019155664 
    print(goal_msg)

    move_base.send_goal(goal_msg)
    wait = move_base.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return move_base.get_result()

    
if __name__ == '__main__':
    try:
        ttb3_cmd_nav()
    except rospy.ROSInterruptException:
        pass
