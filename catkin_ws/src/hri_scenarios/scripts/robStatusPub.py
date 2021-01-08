#!/usr/bin/env python
import rospy
import sys
from std_msgs.msg import String

def robotStatusPub(data):
    pub = rospy.Publisher('robotMove', String, queue_size=10)
    rospy.init_node('robotStatus', anonymous=False)
    # rospy.loginfo(data)
    rate = rospy.Rate(10)
    count = 0
    while count < 3:
	pub.publish(data) 
        rate.sleep()
        count+=1

if __name__ == '__main__':
    try:
        robotStatusPub(str(sys.argv[1]))
    except rospy.ROSInterruptException:
        pass
