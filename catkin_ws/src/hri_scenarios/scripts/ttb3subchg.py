#!/usr/bin/env python
import rospy
import os
import sys
from std_msgs.msg import String
from sensor_msgs.msg import BatteryState

def callback(data):
    f = open("../scene_logs/robotBattery.log", "a")
    ts=data.header.stamp.secs
    percentage=(float(data.voltage)-11.1)/0.9*100
    chg_log="{:.2f}:{:.2f}\n".format(ts, percentage)
    f.write(chg_log)
    f.close()
        
def listener():
    rospy.init_node('ttb3subchg', anonymous=False)
    rospy.Subscriber("battery_state", BatteryState, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
