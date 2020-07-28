#!/usr/bin/env python
import rospy
import sys
from std_msgs.msg import Float32

def allMotorPub(speed):
    pub = rospy.Publisher('allMotorSpeed', Float32, queue_size=10)
    rospy.init_node('allMotorPub', anonymous=False)
    rospy.loginfo(speed)
    rate = rospy.Rate(10)
    count = 0
    while count < 3:
	pub.publish(speed) 
        rate.sleep()
        count+=1

if __name__ == '__main__':
    try:
        allMotorPub(float(sys.argv[1]))
    except rospy.ROSInterruptException:
        pass
