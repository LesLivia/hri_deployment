#!/usr/bin/env python
import rospy
import os
import sys
from geometry_msgs.msg import PoseWithCovarianceStamped

def callback(data):
    f = open("../scene_logs/robotPosition.log", "a")
    ts=data.header.stamp.secs
    pos=data.pose.pose.position
    ort=data.pose.pose.orientation
    pos_log="{:.2f}:{:.4f}#{:.4f}#{:.4f}#{:.4f}#{:.4f}#{:.4f}\n"
    f.write(pos_log.format(ts, pos.x, pos.y, pos.z, ort.x, ort.y, ort.z))
    f.close()
        
def listener():
    rospy.init_node('ttb3subpos', anonymous=False)
    rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
