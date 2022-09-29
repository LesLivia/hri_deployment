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
    dict = {}
    sorted_dict = {}
    if id_param == l[1]:
        for i in range(0, 8):
            f.write(":" + l[i])
        for j in range(2, 8, 2):
            if l[j] != id_param:
                dict[int(l[j])] = l[j+1]
        sorted_keys = sorted(dict, key=dict.get)  # [1, 3, 2]
        for w in sorted_keys:
            sorted_dict[w] = dict[w]
        for key in sorted_dict:
            f.write(':'+str(key))
        f.write('\n')
    f.close()

def listener():
    rospy.init_node('checkDist'+id_param, anonymous=False)
    rospy.Subscriber("robDist", String, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
