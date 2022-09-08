#!/usr/bin/env python
import rospy
import os
import sys
from std_msgs.msg import String

args = rospy.myargv(argv=sys.argv)
id_param = args[1]

def callback(data):
    if id_param == '0':
        f = open("../scene_logs/humansServed.log", "a")
        f.write("\n" + data.data)
        f.close()

def listener():
    rospy.init_node('humSvcSub'+id_param, anonymous=False)
    rospy.Subscriber("humService", String, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
