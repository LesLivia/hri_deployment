#!/usr/bin/env python
import time
import random
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
ROOM_LOG = '../scene_logs/environmentData.log'

DOOR_POS = Point(17.0+const.VREP_X_OFFSET, -1.0+const.VREP_Y_OFFSET)
CHAIR_POS = Point(18.6+const.VREP_X_OFFSET, 4.67+const.VREP_Y_OFFSET)

class Loc(Enum):
	INIT = -1
	IDLE = 0
	BUSY = 1
	SIT = 2
	RUN = 3
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
		self.freeWillTh = 101
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
		elif log=='ROOM':
			f = open(ROOM_LOG)
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
		elif log=='ROOM':
			read = last_line.split(':')[1]
			return (float(read.split('#')[0]), float(read.split('#')[1]))	
		else:
			read = Point.parse_point(last_line.split(':')[1])
			return Point(read.x+const.VREP_X_OFFSET, read.y+const.VREP_Y_OFFSET)

	# PLAN (AND PUBLISH) TRAJECTORY FROM CURRENT POS TO CURRENT DESTINATION
	def plan_trajectory(self, start: Point, dest: Point):
		if dest!=CHAIR_POS:
			traj = nav.plan_traj(start, dest, nav.init_walls())
		else:
			point_1 = Point(DOOR_POS.x-const.VREP_X_OFFSET, DOOR_POS.y-const.VREP_Y_OFFSET)
			point_2 = Point(18.6, -1.1)
			point_3 = Point(18.6, 0.0)
			point_4 = Point(18.6, 2.0)
			point_5 = Point(CHAIR_POS.x-const.VREP_X_OFFSET, CHAIR_POS.y-const.VREP_Y_OFFSET)
			traj = [point_1, point_2, point_3, point_4, point_5]
		str_traj = ''
		for point in traj:
			str_traj += str(point.x) + ',' + str(point.y)
			if not traj.index(point)==len(traj)-1:
				str_traj += '#'
		if len(traj)>0:
			vrep.set_hum_trajectory(const.VREP_CLIENT_ID, self.h[self.currH].hum_id, str_traj)

	def set_state(self, state:int):
		vrep.set_hum_state(self.clientID, self.h[self.currH].hum_id, state)

	def start_h_action(self):
		self.set_loc(Loc.BUSY)
		# room_data = (temperature [°C], humidity [%])
		room_data = self.read_data('ROOM')
		harsh_env = room_data is not None and (room_data[0]<=15.0 or room_data[0]>=30.0 or room_data[1]<=30.0 or room_data[1]>=50)
		if harsh_env:
			psbl_states = [7]
		else:
			psbl_states = [1]#, 5, 9]
		self.set_state(psbl_states[random.randint(0, len(psbl_states)-1)])
		vrep.start_human(self.clientID, self.h[self.currH].hum_id)

	def stop_h_action(self):
		self.set_loc(Loc.IDLE)
		vrep.stop_human(self.clientID, self.h[self.currH].hum_id)

	def send_sit_cmd(self, running = False):
		if running:
			psbl_states = [4]
		else:
			# room_data = (temperature [°C], humidity [%])
			room_data = self.read_data('ROOM')
			print(room_data)
			harsh_env = room_data is not None and (room_data[0]<=15.0 or room_data[0]>=30.0 or room_data[1]<=30.0 or room_data[1]>=50)
			if harsh_env:
				psbl_states = [6]
			else:
				psbl_states = [2]

		self.set_loc(Loc.SIT)				 
		self.set_state(psbl_states[random.randint(0, len(psbl_states)-1)])
		# vrep.sit(self.clientID, self.h[self.currH].hum_id)

	def send_stand_cmd(self):
		self.set_loc(Loc.IDLE)
		# room_data = (temperature [°C], humidity [%])
		room_data = self.read_data('ROOM')
		harsh_env = room_data is not None and (room_data[0]<=15.0 or room_data[0]>=30.0 or room_data[1]<=30.0 or room_data[1]>=50)
		if harsh_env:
			psbl_states = [8]
		else:
			psbl_states = [0]

		self.set_state(psbl_states[random.randint(0, len(psbl_states)-1)])
		# vrep.stand(self.clientID, self.h[self.currH].hum_id)

	def send_run_cmd(self):
		self.set_loc(Loc.RUN)
		self.set_state(3)
		# vrep.run_cmd(self.clientID, self.h[self.currH].hum_id)

	def reset_hum(self):
		if len(self.m.start)>0:
			vrep.reset_hum(self.clientID, self.h[self.currH].hum_id, self.m.start[self.currH])

	def send_served_cmd(self):
		self.set_loc(Loc.IDLE)
		vrep.served_cmd(self.clientID, self.h[self.currH].hum_id)

	def free_will(self):
		random.seed()
		freeWill = random.randint(0, 100)
		if freeWill >= self.freeWillTh:
			print('AUTONOMOUS ACTION')
		return freeWill >= self.freeWillTh

	def free_sit(self, hum_pos: Point):
		dist_to_door = hum_pos.distance_from(DOOR_POS)
		if dist_to_door < 5.0:
			needs_chair = random.randint(0, 100) >= self.freeWillTh
			return needs_chair
		else:
			return False

	def run_follower(self):
		self.reset_hum()
		self.set_loc(Loc.IDLE)
		SIT_ONCE = True
		will_sit = False
		while not self.served[self.currH] and not vrep.check_connection(self.clientID):
			ftg = self.read_data('FTG')
			pos = self.read_data('HUM_POS')
			rob_pos = self.read_data('ROB_POS')
			dist_to_rob = pos.distance_from(rob_pos)

			free_start = self.LOC == Loc.IDLE and self.free_will()
			if (not self.served[self.currH] and dist_to_rob > 2.0) or free_start:
				if will_sit and not SIT_ONCE:
					will_sit = False
					self.send_stand_cmd()
				if will_sit:
					self.plan_trajectory(pos, CHAIR_POS)
				else:
					self.plan_trajectory(pos, rob_pos)
				self.start_h_action()

			time.sleep(self.Tpoll)
			
			if SIT_ONCE and not will_sit:
				will_sit = self.free_sit(pos) 
			# else:
			#	will_sit = False
			dest = CHAIR_POS if will_sit else self.m.dest[self.currH]

			dist_to_dest = pos.distance_from(dest)
			self.served[self.currH] = dist_to_dest < 2.0 and not will_sit
			if self.debug:
				print('HUMAN in {}, ftg: {:.5f}'.format(pos, ftg))
				print('DIST TO DEST: {:.5f}'.format(dist_to_dest))
				print('DIST TO ROB: {:.5f}'.format(dist_to_rob))

			free_stop = self.LOC == Loc.BUSY and self.free_will()
			if self.served[self.currH] or dist_to_rob < 1.0 or free_stop or (will_sit and dist_to_dest < 2.0):
				self.stop_h_action()
				rest_time = random.randint(8, 20)
				if will_sit and dist_to_dest < 2.0:
					SIT_ONCE = False
					self.send_sit_cmd()
					time.sleep(20)
				elif free_stop:
					time.sleep(rest_time)

		print('Human {} successfully served'.format(self.currH+1))
		if self.currH < len(self.h) - 1:
			self.currH+=1

	def run_leader(self):
		self.reset_hum()
		self.set_loc(Loc.IDLE)
		running = False
		SIT_ONCE = True
		will_sit = False
		while not self.served[self.currH] and not vrep.check_connection(self.clientID):
			ftg = self.read_data('FTG')
			pos = self.read_data('HUM_POS')
			rob_pos = self.read_data('ROB_POS')
			dist_to_rob = pos.distance_from(rob_pos)

			if SIT_ONCE and not will_sit:
				will_sit = self.free_sit(pos) 
			dest = CHAIR_POS if will_sit else self.m.dest[self.currH]

			if not self.served[self.currH]:
				# in_office = 1.0+const.VREP_X_OFFSET<=pos.x<=11+const.VREP_X_OFFSET and 1.4+const.VREP_Y_OFFSET<=pos.y<=9.5+const.VREP_Y_OFFSET
				in_office = random.randint(0,100)>=self.freeWillTh
				if will_sit and not SIT_ONCE:
					will_sit = False
					self.send_stand_cmd()
				if will_sit:
					self.plan_trajectory(pos, CHAIR_POS)
				else:
					self.plan_trajectory(pos, dest)

				if in_office or running:
					running = True
					self.send_run_cmd()
				else:
					self.start_h_action()

			time.sleep(self.Tpoll)
			
			dist_to_dest = pos.distance_from(dest)
			self.served[self.currH] = dist_to_dest < 1.0  if dest==self.m.dest[self.currH] else False
			if self.debug:
				print('HUMAN in {}, ftg: {:.5f}'.format(pos, ftg))
				print('DIST TO DEST: {:.5f}'.format(dist_to_dest))
				print('DIST TO ROB: {:.5f}'.format(dist_to_rob))

			if self.served[self.currH] or dist_to_rob < 2.0 or dist_to_rob > 8.0 or (will_sit and dist_to_dest < 2.0):
				self.stop_h_action()
				rest_time = random.randint(8, 20)
				if will_sit and dist_to_dest < 2.0:
					SIT_ONCE = False
					self.send_sit_cmd(running)
					time.sleep(20)

				if self.served[self.currH]:
					time.sleep(random.randint(10, 20))
					running = False
					self.send_served_cmd()

		if self.served[self.currH]:
			print('Human {} successfully served'.format(self.currH+1))
		if self.currH < len(self.h) - 1:
			self.currH+=1

	def run_recipient(self):
		pass

	def run(self, m: Mission):
		self.m = m
		while not self.served[-1] and not vrep.check_connection(self.clientID):
			if m.p[self.currH] == Pattern.HUM_FOLLOWER:
				print('FOLLOWER starting...')
				self.run_follower()
			elif m.p[self.currH] == Pattern.HUM_LEADER:
				print('LEADER starting...')
				self.run_leader()
			elif m.p[self.currH] == Pattern.HUM_RECIPIENT:
				# TODO
				self.run_recipient()
			else:
				print('No pattern found.')
		vrep.stop_sim(self.clientID)


	

	
