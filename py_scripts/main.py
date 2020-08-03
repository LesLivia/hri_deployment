#!/usr/bin/env python
import time
import os
import sys
import rospy_utils.hrirosnode as hriros
import vrep_utils.vrep as vrep
from threading import Thread
from agents.mobilerobot import MobileRobot
from agents.human import Human
from agents.coordinates import Point

print('Launching application...')

vrep_sim = vrep.connect(19999)
vrep.start_sim(vrep_sim)

bill = Human(1, 0, 10, 1, 1)
rob = MobileRobot(1, 10, 5)
try:
	#bill.start_reading_data()
	rob.start_reading_data()
	time.sleep(5)

	#thread_h = Thread(target = bill.follow_position)
	#thread_h.start()
	#thread_h_f = Thread(target = bill.follow_fatigue)
	#thread_h_f.start()

	thread_r = Thread(target=rob.follow_position)
	thread_r.start()
	thread_rb = Thread(target=rob.follow_charge)
	thread_rb.start()
	
	time.sleep(1)
	dest = Point(5.0, 2.0)
	rob.navigate_to(dest)

	rob.set_sim_running(0)
	bill.set_sim_running(0)
	vrep.stop_sim(vrep_sim)
	quit()
	print('Execution finished.')
except (KeyboardInterrupt, SystemExit):
	rob.set_sim_running(0)
	bill.set_sim_running(0)
	vrep.stop_sim(vrep_sim)
	quit()
