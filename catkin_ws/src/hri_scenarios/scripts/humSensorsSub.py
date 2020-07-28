#!/usr/bin/env python
import rospy
import os
import sys
from std_msgs.msg import String

def callback(data):
    f = open("../scene_logs/humanPosition.log", "a")
    f.write("\n" + data.data)
    f.close()
        
def listener():
    rospy.init_node('humPosSub', anonymous=False)
    rospy.Subscriber("humPosition", String, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
