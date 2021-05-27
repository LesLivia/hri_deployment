#!/usr/bin/env python
import time
import vrep_utils.vrep as vrep
from agents.mobilerobot import MobileRobot
from agents.human import Human, FatigueProfile
from agents.coordinates import Point
from agents.mission import *
from hum_control.hum_controller import HumanController

vrep_sim = vrep.connect(19999)
vrep.start_sim(vrep_sim)

bill = Human(1, 10, FatigueProfile.YOUNG_SICK, 1)
carl = Human(2, 10, FatigueProfile.ELDERLY_HEALTHY, 1)
rob = MobileRobot(1, 8.0, 5.0)

# Alternative A
dest = [Point(24.0, 10.5)]
unique_humans = [bill]
humans = [bill, bill]
patterns = [Pattern.HUM_FOLLOWER]	

mission = Mission(patterns, dest)	
contr = HumanController(humans, vrep_sim)

try:
	time.sleep(7)
	print('(Human Controller) Execution starting...')

	contr.run(mission)	

	print('(Human Controller) Execution finished.')
	quit()
except (KeyboardInterrupt, SystemExit):
	quit()
