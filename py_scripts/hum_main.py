#!/usr/bin/env python
import sys
import time
import vrep_utils.vrep as vrep
from agents.mobilerobot import MobileRobot
from agents.human import Human, FatigueProfile
from agents.coordinates import Point
from agents.mission import *
from hum_control.hum_controller import HumanController
from utils.logger import Logger
import agents.navigation as nav
import configparser

LOGGER = Logger("HUM MAIN")

LOGGER.info('Bringing up human controller...')
vrep_sim = -1
if vrep_sim == -1:
	LOGGER.info('Connection to VRep failed, trying a different door...')	
	vrep_sim = vrep.connect(19999)
if vrep_sim == -1:
	LOGGER.info('Connection to VRep failed, trying a different door...')	
	vrep_sim = vrep.connect(19995)
if vrep_sim == -1:
	LOGGER.info('Connection to VRep failed, trying a different door...')	
	vrep_sim = vrep.connect(19997)

# MISSION CONFIGURATION
bill = Human(1, 10, FatigueProfile.YOUNG_SICK, 1)
carl = Human(2, 10, FatigueProfile.ELDERLY_HEALTHY, 1)
rob = MobileRobot(1, 8.0, 5.0)

start = [Point(22.0, 6.0), Point(5.0, 2.0)]
dest = [Point(26.0, 2.0), Point(14.0, 4.0)]
unique_humans = [bill, carl]
humans = [bill, carl]

patterns = []
f = open('mission.txt', 'r')
lines = f.readlines()
for line in lines:
	if line.replace('\n', '') == 'LEADER':
		patterns.append(Pattern.HUM_LEADER)
	else:
		patterns.append(Pattern.HUM_FOLLOWER)
f.close()

mission = Mission(patterns, dest, start=start)	

contr = HumanController(humans, vrep_sim)

try:
	time.sleep(5)
	LOGGER.info('Starting human controller...')

	contr.run(mission)	

	LOGGER.info('Shutting human controller down.')
	quit()
except (KeyboardInterrupt, SystemExit):
	quit()
