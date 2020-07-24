#!/usr/bin/env python2
import rospy
from std_msgs.msg import Float32

def rightMotorPub():
    pub = rospy.Publisher('rightMotorSpeed', Float32, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
	data = 3.14;
        rospy.loginfo(data)
        pub.publish(data)
        rate.sleep()

if __name__ == '__main__':
    try:
        rightMotorPub()
    except rospy.ROSInterruptException:
        pass
