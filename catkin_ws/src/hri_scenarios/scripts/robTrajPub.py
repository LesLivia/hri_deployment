#!/usr/bin/env python
import rospy
import sys
from std_msgs.msg import String

def robTrajPub(points):
    pub = rospy.Publisher('robotTraj', String, queue_size=10)
    rospy.init_node('robotTraj', anonymous=False)
    # rospy.loginfo(points)
    rate = rospy.Rate(10)
    count = 0
    while count < 3:
        pub.publish(points)
        rate.sleep()
        count+=1

if __name__ == '__main__':
    try:
        robTrajPub(str(sys.argv[1]))
    except rospy.ROSInterruptException:
        pass
