#!/usr/bin/env python
import time
import vrep_utils.vrep as vrep
import os
import configparser
from agents.human import Human, start_reading_data, FatigueProfile
from agents.coordinates import Point
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
	vrep_sim = vrep.connect(19997)
	vrep.start_sim(vrep_sim)		# start the simulation
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

# SCENARIO CONFIGURATION
alice = Human(0, 10, FatigueProfile.YOUNG_SICK, 1)	# human (id, speed, ftg_profile, fw_profile)
bill = Human(1, 10, FatigueProfile.ELDERLY_HEALTHY, 1)
carl = Human(2, 10, FatigueProfile.ELDERLY_SICK, 1)
dora = Human(3, 10, FatigueProfile.YOUNG_HEALTHY, 1)

#position:
#    x: 4.39499902725
#    y: 0.289999425411
dest = [Point(20.0, 2.0)]			# define the point of destination
unique_humans = [alice, bill, carl, dora]	# for real application
humans = [alice]					# for simulations

#position:
#    x: 4.39499902725
#    y: 0.289999425411
LOGGER.info("Cleaning log files...")

f = open("../scene_logs/synchelp.log", "r+")
f.truncate(0)
f.close()
f1 = open("../scene_logs/syncrisp.log", "r+")
f1.truncate(0)
f1.close()
f2 = open('../scene_logs/robotPosition.log', 'r+')
f2.truncate(0)
f2.close()
f3 = open('../scene_logs/robotDistance.log', 'r+')
f3.truncate(0)
f3.close()
f4 = open('../scene_logs/robotBattery.log', 'r+')
f4.truncate(0)
f4.close()
f5 = open("../scene_logs/humanPosition.log", "r+")
f5.truncate(0)
f5.close()
f6 = open("../scene_logs/humanFatigue.log", "r+")
f6.truncate(0)
f6.close()
f7 = open('../scene_logs/humansServed.log', 'r+')
f7.truncate(0)
f7.close()

LOGGER.info("Finished cleaning")

start_reading_data(humans)

quit()
