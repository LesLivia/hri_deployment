#!/usr/bin/env python
import time
import vrep_utils.vrep as vrep
import os
import configparser
import traceback
from agents.mobilerobot import MobileRobot
from agents.human import Human, start_reading_data, FatigueProfile
from agents.coordinates import Point
from agents.orchestrator import Orchestrator, OpChk
from agents.mission import *
from utils.logger import Logger

config = configparser.ConfigParser()
config.read('./resources/config.ini')
config.sections()

LOGGER = Logger("MAIN")

LOGGER.info('Bringing up the environment...')
MAP_NAME = config['DEPLOYMENT ENVIRONMENT']['MAP_NAME']
if config['DEPLOYMENT ENVIRONMENT']['ENV']=='S':
	VREP_PATH = config['DEPLOYMENT ENVIRONMENT']['VREP_PATH']
	SCENE_PATH = config['DEPLOYMENT ENVIRONMENT']['VREP_SCENES_PATH']
	# Start VRep 
	os.system("./resources/bringup_sim.sh {} {} {}".format(VREP_PATH, SCENE_PATH, MAP_NAME+'_sim'))
	vrep_sim = vrep.connect(19997)
	vrep.start_sim(vrep_sim)
else:
	MAP_PATH = config['DEPLOYMENT ENVIRONMENT']['MAP_PATH']
	os.system("./resources/bringup_real.sh {} {}".format(MAP_PATH, MAP_NAME))
	MODE = config['DEPLOYMENT ENVIRONMENT']['MODE']
	if MODE=='hybrid':
		VREP_PATH = config['DEPLOYMENT ENVIRONMENT']['VREP_PATH']
		SCENE_PATH = config['DEPLOYMENT ENVIRONMENT']['VREP_SCENES_PATH']
		# Start VRep 
		os.system("./resources/bringup_sim.sh {} {} {}".format(VREP_PATH, SCENE_PATH, MAP_NAME))
		vrep_sim = vrep.connect(19997)
		vrep.start_sim(vrep_sim)

# SCENARIO CONFIGURATION
alice = Human(1, 10, FatigueProfile.YOUNG_SICK, 1)
bill = Human(2, 10, FatigueProfile.ELDERLY_HEALTHY, 1)
carl = Human(3, 10, FatigueProfile.ELDERLY_SICK, 1)
dora = Human(4, 10, FatigueProfile.YOUNG_HEALTHY, 1)
rob = MobileRobot(1, 12.0, 5.0)

#position: 
#    x: 4.39499902725
#    y: 0.289999425411
dest = [Point(15.0, 2.0)]
unique_humans = [alice, bill, carl, dora]
humans = [alice]

patterns = []
f = open('mission.txt', 'r')
lines = f.readlines()
for line in lines:
	if line.replace('\n', '') == 'LEADER':
		patterns.append(Pattern.HUM_LEADER)
	elif line.replace('\n', '') == 'RECIPIENT':
		patterns.append(Pattern.HUM_RECIPIENT)
	else:
		patterns.append(Pattern.HUM_FOLLOWER)
f.close()

mission = Mission(patterns, dest)	

# SCENARIO DEPLOYMENT
try:
	# START ROS NODES THAT ACQUIRE DATA FROM SENSORS
	start_reading_data(humans)
	rob.start_reading_data()
	time.sleep(15)
			
	# START MISSION
	opchk = OpChk(0.5, 0.0, rob, humans, mission)
	orch = Orchestrator(opchk)

	orch.run_mission()

	if mission.get_scs():
		LOGGER.info('Mission successfully completed.')
	if mission.fail:
		LOGGER.info('Mission failed.')

	# When mission is over (either with success or failure),
	# shut everything down
	for hum in humans:
		hum.set_sim_running(0)
	vrep.stop_sim(vrep_sim)
	LOGGER.info('Execution finished.')
	LOGGER.info('Shutting down ROS nodes.')
	os.system("./resources/shutdown.sh")
	quit()

except Exception:
	print(traceback.format_exc())
	LOGGER.info('Shutting down ROS nodes.')
	os.system("./resources/shutdown.sh")
	rob.set_sim_running(0)
	bill.set_sim_running(0)
	vrep.stop_sim(vrep_sim)
	quit()
