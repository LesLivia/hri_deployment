#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32

def rightMotorPub(speed):
    pub = rospy.Publisher('rightMotorSpeed', Float32, queue_size=10)
    rospy.init_node('rightMotorPub', anonymous=False)
    rospy.loginfo(speed)
    rate = rospy.Rate(10)
    count = 0
    while count < 3:
	pub.publish(speed) 
        rate.sleep()
        count+=1

if __name__ == '__main__':
    try:
        rightMotorPub(3.14)
    except rospy.ROSInterruptException:
        pass
