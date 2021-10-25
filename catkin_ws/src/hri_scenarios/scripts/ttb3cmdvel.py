#!/usr/bin/env python
import rospy
import sys
from geometry_msgs.msg import Twist

def ttb3_cmd_vel(speed):
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rospy.init_node('ttb3_cmd_vel', anonymous=False)
    rospy.loginfo(speed)
    rate = rospy.Rate(10)
    count = 0
    vel_msg = Twist()
    vel_msg.linear.x = speed
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0
    print(vel_msg)
    while count < 1:
	pub.publish(vel_msg) 
        rate.sleep()
        count+=1

if __name__ == '__main__':
    try:
        ttb3_cmd_vel(float(sys.argv[1]))
    except rospy.ROSInterruptException:
        pass
