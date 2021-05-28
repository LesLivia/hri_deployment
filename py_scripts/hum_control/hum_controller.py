#!/usr/bin/env python
import time
import vrep_utils.vrep as vrep
import agents.navigation as nav
import rospy_utils.hriconstants as const
from typing import List
from enum import Enum
from agents.human import Human, FatigueProfile
from agents.coordinates import Point
from agents.orchestrator import Orchestrator, OpChk
from agents.mission import *

POS_LOG = '../scene_logs/humanPosition.log'
FTG_LOG = '../scene_logs/humanFatigue.log'
ROB_POS_LOG = '../scene_logs/robotPosition.log'

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
		self.m = None
		# SHA-graph variables
		self.LOC = Loc.INIT

	def set_loc(self, loc: Loc):
		if self.debug:
			print('SHA: switching from {} to {}'.format(str(self.LOC), str(loc)))
		self.LOC = loc

	def read_data(self, log: str):
		if log=='FTG':
			f = open(FTG_LOG)
		elif log=='HUM_POS':
			f = open(POS_LOG)
		else:
			f = open(ROB_POS_LOG)
		
		lines = f.readlines()
		if len(lines)<1:
			return None
	
		last_line = lines[-1]
		if log == 'FTG':
			return float(last_line.split(':')[2])
		elif log=='HUM_POS':
			read = Point.parse_point(last_line.split(':')[2])
			return Point(read.x+const.VREP_X_OFFSET, read.y+const.VREP_Y_OFFSET)
		else:
			read = Point.parse_point(last_line.split(':')[1])
			return Point(read.x+const.VREP_X_OFFSET, read.y+const.VREP_Y_OFFSET)

	# PLAN (AND PUBLISH) TRAJECTORY FROM CURRENT POS TO CURRENT DESTINATION
	def plan_trajectory(self, start: Point, dest: Point):
		traj = nav.plan_traj(start, dest, nav.init_walls())
		str_traj = ''
		for point in traj:
			str_traj += str(point.x) + ',' + str(point.y)
			if not traj.index(point)==len(traj)-1:
				str_traj += '#'
		if len(traj)>0:
			vrep.set_hum_trajectory(const.VREP_CLIENT_ID, self.currH+1, str_traj)

	def turn_left(self):
		self.set_loc(Loc.BUSY)
		vrep.turn_left(self.clientID, self.h[self.currH].hum_id)

	def turn_right(self):
		self.set_loc(Loc.BUSY)
		vrep.turn_right(self.clientID, self.h[self.currH].hum_id)

	def start_h_action(self):
		self.set_loc(Loc.BUSY)
		vrep.start_human(self.clientID, self.h[self.currH].hum_id)

	def start_h_action(self):
		self.set_loc(Loc.BUSY)
		vrep.start_human(self.clientID, self.h[self.currH].hum_id)

	def stop_h_action(self):
		self.set_loc(Loc.IDLE)
		vrep.stop_human(self.clientID, self.h[self.currH].hum_id)
		
	def run_follower(self):
		self.set_loc(Loc.IDLE)
		while not self.served[self.currH] and not vrep.check_connection(self.clientID):
			ftg = self.read_data('FTG')
			pos = self.read_data('HUM_POS')
			rob_pos = self.read_data('ROB_POS')
			dist_to_rob = pos.distance_from(rob_pos)

			if not self.served[self.currH] or dist_to_rob>2.0:
				self.plan_trajectory(pos, rob_pos)
				self.start_h_action()

			time.sleep(self.Tpoll)

			dist_to_dest = pos.distance_from(self.m.dest[self.currH])
			self.served[self.currH] = dist_to_dest < 1.0
			if self.debug:
				print('HUMAN in {}, ftg: {:.5f}'.format(pos, ftg))
				print('DIST TO DEST: {:.5f}'.format(dist_to_dest))
				print('DIST TO ROB: {:.5f}'.format(dist_to_rob))

			if self.served[self.currH] or dist_to_rob<1.0:
				self.stop_h_action()
		print('Human {} successfully served'.format(self.h[self.currH].hum_id))
		self.currH+=1

	def run_leader(self):
		pass

	def run_recipient(self):
		pass

	def run(self, m: Mission):
		self.m = m
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

	

	
