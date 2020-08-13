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
from agents.human import Human
from agents.coordinates import Point
from agents.orchestrator import Orchestrator
from agents.mission import *

print('Launching application...')

vrep_sim = vrep.connect(19999)
vrep.start_sim(vrep_sim)

bill = Human(1, Pattern.HUM_FOLLOWER, 10, 1, 1)
rob = MobileRobot(1, 10, 5)

dest = [Point(22.0, 18.0)]
humans = [bill]
patterns = []
for hum in humans:
	patterns.append(hum.ptrn)
		
mission = Mission(patterns, dest)	

orch = Orchestrator(5, 1, rob, humans, mission)

try:
	# START ROS NODES THAT ACQUIRE DATA FROM SENSORS
	bill.start_reading_data()
	rob.start_reading_data()
	time.sleep(7)
	
	# START MONITORING HUMAN SENSOR DATA LOGS
	thread_h = Thread(target = bill.follow_position)
	thread_h.start()
	thread_h_f = Thread(target = bill.follow_fatigue)
	thread_h_f.start()
	
	# START MONITORING ROBOT SENSOR DATA LOGS
	thread_r = Thread(target=rob.follow_position)
	thread_r.start()
	thread_rb = Thread(target=rob.follow_charge)
	thread_rb.start()
	
	# START MISSION
	time.sleep(3)
	# plan trajectory
	traj = nav.plan_traj(rob.get_position(), dest[0], nav.init_walls())
	str_traj = ''
	for point in traj:
		str_traj += str(point.x) + ',' + str(point.y)
		if not traj.index(point)==len(traj)-1:
			str_traj += '#'
	node = 'robTrajPub.py'
	pool = Pool()
	pool.starmap(hriros.rosrun_nodes, [(node, [str_traj])])

	#thread_m = Thread(target = orch.run_mission)
	#thread_m.start()
	
	# keep going as long as the mission is not over	
	#while not mission.get_scs() and not mission.fail:
	#	pass

	#if mission.get_scs():
	#	print('Mission successfully completed.')
	#if mission.fail:
	#	print('Mission failed.')

	# when mission is over (either with success or failure),
	# shut everything down
	bill.set_sim_running(0)
	#vrep.stop_sim(vrep_sim)
	#print('Execution finished.')
	#quit()

except (KeyboardInterrupt, SystemExit):
	rob.set_sim_running(0)
	bill.set_sim_running(0)
	vrep.stop_sim(vrep_sim)
	quit()
