#!/usr/bin/env python
import rospy
import sys
from std_msgs.msg import String

def missionStatusPub(msg):
    pub = rospy.Publisher('missionStatus', String, queue_size=10)
    rospy.init_node('missionStatusPub', anonymous=False)
    rospy.loginfo(msg)
    rate = rospy.Rate(10)
    count = 0
    while count < 3:
        pub.publish(msg)
        rate.sleep()
        count+=1

if __name__ == '__main__':
    try:
        missionStatusPub(str(sys.argv[1]))
    except rospy.ROSInterruptException:
        pass
