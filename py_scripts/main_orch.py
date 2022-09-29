#!/usr/bin/env python
import time
import vrep_utils.vrep as vrep
import os
import configparser
import traceback
import sys
from agents.mobilerobot import MobileRobot
from agents.human import Human, start_reading_data, FatigueProfile
from agents.coordinates import Point
from agents.orchestrator import Orchestrator, OpChk
from agents.mission import *
from utils.logger import Logger


print('Argument List:', str(sys.argv))
mom = str(sys.argv[1])
robot_id = str(mom)
in_int = int(sys.argv[2])
LOGGER = Logger("MAIN")
print("Robot_id: ", robot_id)
print("Missione: ", in_int)
port = 19997+int(robot_id)
vrep_sim = vrep.connect(port)
print("PORT: " + str(port))
# SCENARIO CONFIGURATION
alice = Human(0, 10, FatigueProfile.YOUNG_SICK, 1)	# human (id, speed, ftg_profile, fw_profile)
bill = Human(1, 10, FatigueProfile.ELDERLY_HEALTHY, 1)
carl = Human(2, 10, FatigueProfile.ELDERLY_SICK, 1)
dora = Human(3, 10, FatigueProfile.YOUNG_HEALTHY, 1)
rob = MobileRobot(robot_id, 12.0, 5.0)	# robot (id, max_speed, max_accel)

#position:
#    x: 4.39499902725
#    y: 0.289999425411
dest = [Point(20.0, 2.0)]			# define the point of destination
unique_humans = [alice, bill, carl, dora]	# for real application
humans = [alice]					# for simulations

patterns = []
f = open('mission.txt', 'r')		# check in the mission.txt if it is follower, leader or recipient
lines = f.readlines()
for line in lines:
	if line.replace('\n', '') == 'LEADER':
		patterns.append(Pattern.HUM_LEADER)
	elif line.replace('\n', '') == 'RECIPIENT':
		patterns.append(Pattern.HUM_RECIPIENT)
	else:
		patterns.append(Pattern.HUM_FOLLOWER)
f.close()

mission = Mission(patterns, dest)	# creates new mission with specific destination and pattern

# DEPLOYMENT
try:
	# START ROS NODES THAT ACQUIRE DATA FROM SENSORS
	rob.start_reading_data()
	time.sleep(15)

	# START MISSION
	if in_int == 0:
		opchk = OpChk(0.5, 0.0, rob, humans, None, robot_id)
	else:
		opchk = OpChk(0.5, 0.0, rob, humans, mission, robot_id)
	orch = Orchestrator(opchk)
	orch.run_mission()

	if mission.get_scs():
		LOGGER.info('Mission successfully completed.')
	if mission.fail:
		LOGGER.info('Mission failed.')

	if str(robot_id == 0):
		for hum in humans:
			hum.set_sim_running(0)
		os.system("./resources/shutdownhum.sh")				# DA MODIFICARE, DOVE LO METTO?
		vrep.stop_sim(vrep_sim)

	LOGGER.info('Execution finished.')
	LOGGER.info('Shutting down ROS nodes.')
	os.system("./resources/shutdown.sh "+str(robot_id))
	quit()

except Exception:
	print(traceback.format_exc())
	LOGGER.info('Shutting down ROS nodes.')
	os.system("./resources/shutdown.sh "+str(robot_id))
	os.system("./resources/shutdownhum.sh")
	rob.set_sim_running(0)
	bill.set_sim_running(0)
	vrep.stop_sim(vrep_sim)
	quit()
