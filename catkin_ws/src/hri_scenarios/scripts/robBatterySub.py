#!/usr/bin/env python
import rospy
import os
import sys
from std_msgs.msg import String

args = rospy.myargv(argv=sys.argv)
id_param = args[1]

def callback(data):
    f = open("../scene_logs/robotBattery.log", "a")
    s = data.data
    l = s.split(':')
    if id_param in l:
        index = l.index(id_param)
        f.write("\n" + l[index-1] + ":" + l[index] + ":" + l[index+1])
    f.close()

def listener():
    rospy.init_node('robBatterySub'+id_param, anonymous=False)
    rospy.Subscriber("robBatteryCharge", String, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
