#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32

def leftMotorPub():
    pub = rospy.Publisher('leftMotorSpeed', Float32, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        #hello_str = "hello world %s" % rospy.get_time()
	data = 3.14;
        rospy.loginfo(data)
        pub.publish(data)
        rate.sleep()

if __name__ == '__main__':
    try:
        leftMotorPub()
    except rospy.ROSInterruptException:
        pass
