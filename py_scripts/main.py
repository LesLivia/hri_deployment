#!/usr/bin/env python
import time
import os
import sys
import rospy_utils.hrirosnode as hriros
import vrep_utils.vrep as vrep
import agents.navigation as nav
from threading import Thread
from multiprocessing import Pool
from agents.mobilerobot import MobileRobot
from agents.human import Human, start_reading_data, follow_position, follow_fatigue
from agents.coordinates import Point
from agents.orchestrator import Orchestrator, OpChk
from agents.mission import *

print('Launching application...')

vrep_sim = vrep.connect(19997)
vrep.start_sim(vrep_sim)

bill = Human(1, 10, 1, 1)
alice = Human(2, 10, 1, 1)
rob = MobileRobot(1, 15.0, 5.0)

dest = [Point(32.0, 4.0), Point(22.0, 4.0)]
humans = [bill, alice]

patterns = [Pattern.HUM_FOLLOWER, Pattern.HUM_LEADER]	
mission = Mission(patterns, dest)	

try:
	# START ROS NODES THAT ACQUIRE DATA FROM SENSORS
	start_reading_data(humans)
	rob.start_reading_data()
	time.sleep(5)
	
	# START MONITORING HUMAN SENSOR DATA LOGS
	thread_h = Thread(target = follow_position, args=[humans])
	thread_h.start()
	thread_h_f = Thread(target = follow_fatigue, args=[humans])
	thread_h_f.start()
	
	# START MONITORING ROBOT SENSOR DATA LOGS
	thread_r = Thread(target=rob.follow_position)
	thread_r.start()
	thread_rb = Thread(target=rob.follow_charge)
	thread_rb.start()
	
	# START MISSION
	time.sleep(5)
	opchk = OpChk(0.5, 0.0, rob, humans, mission)
	orch = Orchestrator(opchk)
	thread_m = Thread(target = orch.run_mission)
	thread_m.start()
	
	# keep going as long as the mission is not over	
	while rob.is_sim_running():
		pass

	if mission.get_scs():
		print('Mission successfully completed.')
	if mission.fail:
		print('Mission failed.')

	# when mission is over (either with success or failure),
	# shut everything down
	for hum in humans:
		hum.set_sim_running(0)
	vrep.stop_sim(vrep_sim)
	print('Execution finished.')
	quit()

except (KeyboardInterrupt, SystemExit):
	rob.set_sim_running(0)
	bill.set_sim_running(0)
	vrep.stop_sim(vrep_sim)
	quit()
