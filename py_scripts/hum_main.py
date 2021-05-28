#!/usr/bin/env python
import sys
import time
import vrep_utils.vrep as vrep
from agents.mobilerobot import MobileRobot
from agents.human import Human, FatigueProfile
from agents.coordinates import Point
from agents.mission import *
from hum_control.hum_controller import HumanController

vrep_sim = vrep.connect(19999)
if vrep_sim == -1:
	# connection to v-rep could not be established
	quit()

vrep.start_sim(vrep_sim)

bill = Human(1, 10, FatigueProfile.YOUNG_SICK, 1)
carl = Human(2, 10, FatigueProfile.ELDERLY_HEALTHY, 1)
rob = MobileRobot(1, 8.0, 5.0)

dest = [Point(16.0, -1.1)]
unique_humans = [bill]
humans = [bill]
patterns = [Pattern.HUM_FOLLOWER]	

mission = Mission(patterns, dest)	


debug = bool(sys.argv[1]) if len(sys.argv)>1 else False
contr = HumanController(humans, vrep_sim, debug)

try:
	time.sleep(15)
	print('(Human Controller) Execution starting...')

	contr.run(mission)	

	print('(Human Controller) Execution finished.')
	quit()
except (KeyboardInterrupt, SystemExit):
	quit()
