#!/usr/bin/env python
import rospy
import os
import sys
from std_msgs.msg import String

def callback(data):
    try:
        f = open("../scene_logs/humanFatigue.log", "a")
        f.write("\n" + data.data)
        f.close()
    except IOError:
	rospy.loginfo(data.data[:50])
        print(data.data)
        
def listener():
    rospy.init_node('humFtgSub', anonymous=False)
    rospy.Subscriber("humFatigue", String, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
