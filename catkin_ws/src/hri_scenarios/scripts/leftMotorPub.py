#!/usr/bin/env python
import rospy
import sys
from std_msgs.msg import Float32

def leftMotorPub(speed):
    pub = rospy.Publisher('leftMotorSpeed', Float32, queue_size=10)
    rospy.init_node('leftPub', anonymous=False)
    rospy.loginfo(speed)
    rate = rospy.Rate(10)
    count = 0
    while count < 3:
	pub.publish(speed) 
        rate.sleep()
        count+=1

if __name__ == '__main__':
    try:
	print(sys.argv)
        leftMotorPub(float(sys.argv[1]))
    except rospy.ROSInterruptException:
        pass
