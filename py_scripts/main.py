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
from agents.human import Human, start_reading_data, follow_position, follow_fatigue, FatigueProfile
from agents.coordinates import Point
from agents.orchestrator import Orchestrator, OpChk
from agents.mission import *

print('Launching application...')

vrep_sim = vrep.connect(19997)
vrep.start_sim(vrep_sim)

bill = Human(1, 10, FatigueProfile.YOUNG_SICK, 1)
carl = Human(2, 10, FatigueProfile.ELDERLY_HEALTHY, 1)
rob = MobileRobot(1, 12.0, 5.0)

dest = [Point(22.5, 6.0)] #, Point(5.0, 10.0)]
unique_humans = [bill]
humans = [bill] #, bill]

patterns = []
f = open('mission.txt', 'r')
lines = f.readlines()
for line in lines:
	if line.replace('\n', '') == 'LEADER':
		patterns.append(Pattern.HUM_LEADER)
	else:
		patterns.append(Pattern.HUM_FOLLOWER)
f.close()

mission = Mission(patterns, dest)	

try:
	# START ROS NODES THAT ACQUIRE DATA FROM SENSORS
	start_reading_data(humans)
	rob.start_reading_data()
	time.sleep(5)
			
	# START MISSION
	opchk = OpChk(0.5, 0.0, rob, humans, mission)
	orch = Orchestrator(opchk)

	orch.run_mission()

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
