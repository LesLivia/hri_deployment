#!/usr/bin/env python
import time
import vrep_utils.vrep as vrep
from typing import List
from agents.human import Human, FatigueProfile
from agents.coordinates import Point
from agents.orchestrator import Orchestrator, OpChk
from agents.mission import *

class HumanController:
	def __init__(self, h: List[Human], clientID):
		self.clientID = clientID
		self.h = h
		self.currH = 0

	def start_h_action(self):
		vrep.start_human(self.clientID, self.h[self.currH].hum_id)

	def stop_h_action(self):
		vrep.stop_human(self.clientID, self.h[self.currH].hum_id)

	def run_follower(self):
		self.start_h_action()
		time.sleep(10)
		self.stop_h_action()

	def run_leader(self):
		pass

	def run_recipient(self):
		pass

	def run(self, m: Mission):
		if m.p[self.currH] == Pattern.HUM_FOLLOWER:
			print('FOLLOWER starting...')
			self.run_follower()
		elif m.p[self.currH] == Pattern.HUM_LEADER:
			# TODO
			self.run_leader()
		elif m.p[self.currH] == Pattern.HUM_RECIPIENT:
			# TODO
			self.run_recipient()
		else:
			print('No pattern found.')

	

	
