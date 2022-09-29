#!/usr/bin/env python
import rospy
import sys
from std_msgs.msg import String

args = rospy.myargv(argv=sys.argv)
mom = args[1]
id_param = mom[0]

def ReachingRobotPub(data):
    pub = rospy.Publisher('RispSync', String, queue_size=10)
    rospy.init_node('rispPub'+id_param, anonymous=False)
    rate = rospy.Rate(10)
    pub.publish(data)
    rate.sleep()


if __name__ == '__main__':
    try:
        ReachingRobotPub(str(sys.argv[2]))
    except rospy.ROSInterruptException:
        pass
