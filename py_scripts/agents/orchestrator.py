#!/usr/bin/env python
import time
from enum import Enum
from typing import List
from agents.coordinates import Point
from agents.mobilerobot import MobileRobot
from agents.human import Human
from agents.mission import Mission

class Operating_Modes(Enum):
	ROBOT_IDLE = 1
	ROBOT_LEAD = 2
	ROBOT_RECH = 3
	ROBOT_CARR = 4
	ROBOT_FOLL = 5
	
class Orchestrator:
	def __init__(self, t_int: int, t_proc: int, rob: MobileRobot, hum: List[Human], m: Mission):
		# PARAMS
		self.t_int = t_int
		self.t_proc = t_proc

		self.currOp = Operating_Modes.ROBOT_IDLE
		self.currH = 0
		
		self.rob = rob
		self.humans = hum
		self.mission = m

		# THRESHOLDS
		self.STOP_DIST = 4.0
		self.RESTART_DIST = 2.0

		self.RECHARGE_TH = 10.0
		self.STOP_RECHARGE = 50.0
		self.FAIL_CHARGE = 1.0
		
		self.FAIL_FATIGUE = 0.97
		self.STOP_FATIGUE = 0.75
		self.RESUME_FATIGUE = 0.3;

	def run_mission(self):
		print('Starting mission...')
		while not self.check_scs() and not self.check_fail():
			print('Checking actions...')
			self.check_actions()
			time.sleep(self.t_int)

		self.rob.set_sim_running(0)
		return

	def check_actions(self):
		if self.check_scs():
			print('Mission ended successfully.')
			return 

		if self.check_fail():
			print('Mission failed.')
			return

		self.rob.navigate_to(self.mission.dest[self.currH])

		if self.currOp == Operating_Modes.ROBOT_IDLE:
			self.check_start()
		elif self.currOp == Operating_Modes.ROBOT_LEAD or self.currOp == Operating_Modes.ROBOT_CARR:
			self.check_r_move()
		elif self.currOp == Operating_Modes.ROBOT_RECH:
			self.check_r_rech()
		#elif self.currOp==5:
			#check_h_move()

		self.check_service_provided()

	def check_scs(self):
		return self.mission.get_scs()

	def check_fail(self):
		if self.rob.get_charge()<=self.FAIL_CHARGE:
			self.mission.fail = True
		
		if self.humans[self.currH].get_fatigue()>=self.FAIL_FATIGUE:
			self.mission.fail = True
			
		return self.mission.fail

	def check_service_provided(self):
		dest = self.mission.dest[self.currH]
		position = self.rob.get_position()
		pos = Point(position.x, position.y)
		_min_dist = 1.0
		print('Currently going to: ' + str(dest.x) + ' ' + str(dest.y))
		if pos.distance_from(dest) <= _min_dist:
			self.mission.set_served(self.currH)
			self.currH+=1

	def check_start(self):
		return
	
	def check_r_move(self):
		return
	
	def check_r_rech(self):
		return
		














		
