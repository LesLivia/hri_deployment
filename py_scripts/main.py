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

config = configparser.ConfigParser()	# a configuration file divided in sections, each containing keys with values (dictionaries)
config.read('./resources/config.ini')
config.sections()

LOGGER = Logger("MAIN")

LOGGER.info('Bringing up the environment...')
MAP_NAME = config['DEPLOYMENT ENVIRONMENT']['MAP_NAME']					# prendi il nome della mappa
if config['DEPLOYMENT ENVIRONMENT']['ENV']=='S':	
	VREP_PATH = config['DEPLOYMENT ENVIRONMENT']['VREP_PATH']			# prendi il path di vrep
	SCENE_PATH = config['DEPLOYMENT ENVIRONMENT']['VREP_SCENES_PATH']	# prendi il path della scena
	# Start VRep
	os.system("./resources/bringup_sim.sh {} {} {}".format(VREP_PATH, SCENE_PATH, MAP_NAME+'_sim'))	# writes and executes the command
	vrep_sim = vrep.connect(19997)	# 19997 ???
	vrep.start_sim(vrep_sim)		# start the simulation
	# print("SIMULAZIONE PARTITA")
else:
	MAP_PATH = config['DEPLOYMENT ENVIRONMENT']['MAP_PATH']
	os.system("./resources/bringup_real.sh {} {}".format(MAP_PATH, MAP_NAME))
	MODE = config['DEPLOYMENT ENVIRONMENT']['MODE']
	if MODE=='hybrid':	# se ibrido fai partire entrambi
		VREP_PATH = config['DEPLOYMENT ENVIRONMENT']['VREP_PATH']
		SCENE_PATH = config['DEPLOYMENT ENVIRONMENT']['VREP_SCENES_PATH']
		# Start VRep
		os.system("./resources/bringup_sim.sh {} {} {}".format(VREP_PATH, SCENE_PATH, MAP_NAME))
		vrep_sim = vrep.connect(19997)
		vrep.start_sim(vrep_sim)
"""
# SCENARIO CONFIGURATION
alice = Human(0, 10, FatigueProfile.YOUNG_SICK, 1)	# human (id, speed, ftg_profile, fw_profile)
bill = Human(1, 10, FatigueProfile.ELDERLY_HEALTHY, 1)
carl = Human(2, 10, FatigueProfile.ELDERLY_SICK, 1)
dora = Human(3, 10, FatigueProfile.YOUNG_HEALTHY, 1)
rob = MobileRobot(robot_id, 12.0, 5.0)	# robot (id, max_speed, max_accel)

#position: 
#    x: 4.39499902725
#    y: 0.289999425411
dest = [Point(15.0, 2.0)]			# define the point of destination
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
	start_reading_data(humans)
	rob.start_reading_data()
	time.sleep(15)
	
	# START MISSION
	opchk = OpChk(0.5, 0.0, rob, humans, mission, robot_id)
	# opchk = OpChk(0.5, 0.0, rob, humans, mission)	# OpChk(t_int, t_proc, MobileRobot, List Human, Mission) <-- t_int = ogni quanto ricontrolla
	orch = Orchestrator(opchk)						# Orchestrator (OpChk)

	while(in_int == 0):
		orch.not_run()

	orch.run_mission()								# tutto qua dentro il run, poi check solo su com'è andata la missione

	if mission.get_scs():
		LOGGER.info('Mission successfully completed.')
	if mission.fail:
		LOGGER.info('Mission failed.')

	# When mission is over (either with success or failure), shut everything down
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
	"""
