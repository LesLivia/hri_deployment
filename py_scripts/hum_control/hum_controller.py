#!/usr/bin/env python
import time
import vrep_utils.vrep as vrep
from typing import List
from enum import Enum
from agents.human import Human, FatigueProfile
from agents.coordinates import Point
from agents.orchestrator import Orchestrator, OpChk
from agents.mission import *

class Loc(Enum):
	INIT = -1
	IDLE = 0
	BUSY = 1
	PASSED_OUT = 99

	def __str__(self):
		if self == Loc.INIT:
			return 'INIT'
		elif self == Loc.IDLE:
			return 'IDLE'
		elif self == Loc.BUSY:
			return 'BUSY'
		else:
			return 'PASSED_OUT'

class HumanController:
	def __init__(self, h: List[Human], clientID, debug: bool = False):
		self.clientID = clientID
		self.h = h
		self.currH = 0
		self.Tpoll = 1.0
		self.served = [False]*len(h)
		self.debug = debug
		# SHA-graph variables
		self.LOC = Loc.INIT

	def set_loc(self, loc: Loc):
		if self.debug:
			print('SHA: switching from {} to {}'.format(str(self.LOC), str(loc)))
		self.loc = loc

	def start_h_action(self):
		self.set_loc(Loc.BUSY)
		vrep.start_human(self.clientID, self.h[self.currH].hum_id)

	def stop_h_action(self):
		self.set_loc(Loc.IDLE)
		vrep.stop_human(self.clientID, self.h[self.currH].hum_id)

	def run_follower(self):
		self.set_loc(Loc.IDLE)
		while not self.served[self.currH]:
			self.start_h_action()
			time.sleep(10)
			self.stop_h_action()
			self.served[self.currH] = True
		print('Human {} successfully served'.format(self.h[self.currH].hum_id))
		self.currH+=1

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

	

	
