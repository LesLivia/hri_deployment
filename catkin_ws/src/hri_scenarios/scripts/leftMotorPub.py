#!/usr/bin/env python
import rospy
import sys
from std_msgs.msg import String

def leftMotorPub(data):
    pub = rospy.Publisher('leftMotorSpeed', String, queue_size=10)
    rospy.init_node('leftMotorPub', anonymous=False)
    rospy.loginfo(data)
    rate = rospy.Rate(10)
    count = 0
    while count < 3:
	pub.publish(data) 
        rate.sleep()
        count+=1

if __name__ == '__main__':
    try:
        leftMotorPub(str(sys.argv[1]))
    except rospy.ROSInterruptException:
        pass
