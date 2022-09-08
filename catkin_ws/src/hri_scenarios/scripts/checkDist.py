#!/usr/bin/env python
import rospy
import os
import sys
from std_msgs.msg import String

args = rospy.myargv(argv=sys.argv)
id_param = args[1]

def callback(data):
    n_robots = 3
    f = open("../scene_logs/robotDistance.log", "a")
    s = data.data
    l = s.split(':')
    if id_param == l[1]:
        for i in range(0, 7):
            f.write(l[i] + ":")
        f.write("\n")
    f.close()

def listener():
    rospy.init_node('checkDist'+id_param, anonymous=False)
    rospy.Subscriber("robDist", String, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
