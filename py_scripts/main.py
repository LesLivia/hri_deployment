#!/usr/bin/env python
import time
import os
import sys
import rospy_utils.hrirosnode as hriros
from threading import Thread
from agents.mobilerobot import MobileRobot
from agents.human import Human
from agents.coordinates import Point

try:
	print('Launching application...')
	#bill = Human(1, 0, 10, 1, 1)

	#bill.start_reading_position()
	#time.sleep(1)
	#thread_h = Thread(target = bill.follow_position)
	#thread_h.start()

	rob = MobileRobot(1, 10, 5)

	rob.start_reading_position()
	time.sleep(2)
	thread_r = Thread(target = rob.follow_position)
	thread_r.start()

	time.sleep(1)
	dest = Point(10.0, 3.0)
	rob.navigate_to(dest)
	#rob.start_moving(rob.max_speed)
	#time.sleep(5)
	#rob.stop_moving()
	#time.sleep(1)
	#rob.turn_left(1.57)
	#time.sleep(5)
	#rob.turn_right(-1.57)
	#time.sleep(5)
	#rob.start_moving(rob.max_speed*3)
	#time.sleep(5)
	#rob.stop_moving()
	
	print('Execution finished.')

except (KeyboardInterrupt, SystemExit):
	print('Exception caught...')
	#hriros.roskill_nodes('/humPosSub')
	#hriros.roskill_nodes('/robPosSub')
	#hriros.roskill_nodes('/allPub')
	#hriros.roskill_nodes('/rightMotorPub')
	#hriros.roskill_nodes('/leftPub')	






