#!/usr/bin/env python
import rospy
import os
import sys
from std_msgs.msg import String

args = rospy.myargv(argv=sys.argv)
id_param = args[1]

def callback(data):
    try:
        if id_param == '0':
            f = open("../scene_logs/humanFatigue.log", "a")
            f.write("\n" + data.data)
            f.close()
    except IOError:
        rospy.loginfo(data.data[:50])
        print(data.data)
        
def listener():
    rospy.init_node('humFtgSub'+str(id_param), anonymous=False)
    rospy.Subscriber("humFatigue", String, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
