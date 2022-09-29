#!/usr/bin/env python
import rospy
import os
import sys
from std_msgs.msg import String

args = rospy.myargv(argv=sys.argv)
id_param = args[1]

def callback(data):
    f = open("../scene_logs/syncrisp.log", "a")
    f.write(data.data)
    f.write("\n")
    f.close()

def listener():
    rospy.init_node('rispSub'+id_param, anonymous=False)
    rospy.Subscriber("RispSync", String, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()

