#!/usr/bin/env python
import rospy
import sys
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist
import actionlib
from actionlib_msgs.msg import GoalID

def ttb3_cmd_vel(speed, goal_id=None):
    if speed>=0.0:
        pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        rospy.init_node('ttb3_cmd_vel', anonymous=False)
        vel_msg = Twist()
        vel_msg.linear.x = speed
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0
        pub.publish(vel_msg) 
    else:
        pub = rospy.Publisher('move_base/cancel', GoalID, queue_size=10)
        rospy.init_node('ttb3_cmd_vel', anonymous=False)
        cancel_msg = GoalID()
        cancel_msg.stamp = rospy.Time.now()
        cancel_msg.id = goal_id
        move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction) 
        move_base.cancel_goal()

if __name__ == '__main__':
    try:
        if len(sys.argv)>2:
            ttb3_cmd_vel(float(sys.argv[1]), sys.argv[2])
        else:
            ttb3_cmd_vel(float(sys.argv[1]))
    except rospy.ROSInterruptException:
        pass
