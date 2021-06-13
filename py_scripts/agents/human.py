#!/usr/bin/env python
import os
import time
import rospy_utils.hrirosnode as hriros
import rospy_utils.hriconstants as const
import math
import numpy as np
import scipy.io
from scipy import signal
from typing import List
from multiprocessing import Pool
from agents.position import Position
from agents.coordinates import Point
from enum import Enum

class FatigueProfile(Enum):
	YOUNG_HEALTHY = 1
	YOUNG_SICK = 2
	ELDERLY_HEALTHY = 3
	ELDERLY_SICK = 4

	def get_def_rates(self):
		if self==FatigueProfile.YOUNG_HEALTHY:
			return [0.0004, 0.0005]
		elif self==FatigueProfile.YOUNG_SICK:
			return [0.004, 0.005]
		elif self==FatigueProfile.ELDERLY_HEALTHY:
			return [0.0005, 0.0004]
		elif self==FatigueProfile.ELDERLY_SICK:
			return [0.005, 0.004]

class Human:
	def __init__(self, hum_id, speed, ftg_profile, fw_profile):
		self.hum_id = hum_id
		self.speed = speed
		self.ftg_profile = ftg_profile
		self.fw_profile = fw_profile
		self.moving = False
		self.position = None
		self.fatigue = 0.0
		self.f_0 = 0
		self.last_switch = 0.0
		self.emg_walk = []
		self.emg_rest = []
		self.lambdas = [0.0005]
		self.mus = [0.0005]
		self.def_bursts_mov = []
		self.def_bursts_rest = []
		self.cand_bursts_mov = []
		self.cand_bursts_rest = []

	def set_position(self, position: Position):
		self.position = position

	def get_position(self):
		return self.position

	def set_fatigue(self, ftg: float):
		self.fatigue = ftg

	def get_fatigue(self):
		return self.fatigue

	def set_f_o(self, f_0: float):
		self.f_0 = f_0

	def get_f_o(self):
		return self.f_0

	def set_last_switch(self, t: float):
		self.last_switch = t

	def get_last_switch(self):
		return self.last_switch

	def set_emg_signal(self, state: str, emg: List[float]):
		if state=='m':
			self.emg_walk += emg
		else:
			self.emg_rest += emg

	def get_emg_signal(self, state: str):
		return self.emg_walk if state=='m' else self.emg_rest

	def set_lambdas(self, l: float):
		self.lambdas.append(l)

	def get_lambdas(self):
		return self.lambdas

	def set_mus(self, m: float):
		self.mus.append(m)

	def get_mus(self):
		return self.mus

	def set_sim_running(self, run):
        	self.sim_running = run

	def is_sim_running(self):
		return self.sim_running

	def set_is_moving(self, moving: bool):
		self.moving = moving
	
	def is_moving(self):
		return self.moving

def start_reading_data(humans: List[Human]):
	f = open('../scene_logs/humanPosition.log', 'r+')
	f.truncate(0)
	f.close()

	node = 'humSensorsSub.py'

	pool = Pool()
	pool.starmap(hriros.rosrun_nodes, [(node, '')])

	f = open('../scene_logs/humanFatigue.log', 'r+')
	f.truncate(0)
	f.close()

	node = 'humFtgSub.py'

	pool = Pool()
	pool.starmap(hriros.rosrun_nodes, [(node, '')])

	f = open('../scene_logs/humansServed.log', 'r+')
	f.truncate(0)
	f.close()

	node = 'humServiceSub.py'

	pool = Pool()
	pool.starmap(hriros.rosrun_nodes, [(node, '')])

	for hum in humans:
		f = open('../scene_logs/emg_to_ftg{}.log'.format(hum.hum_id), 'r+')
		f.truncate(0)
		f.close()	

	for hum in humans:	
		hum.set_sim_running(1)

def follow_position(hums: List[Human]):
	filename = '../scene_logs/humanPosition.log'
	f = open(filename, 'r')
	lines = f.read().splitlines()
	
	if len(lines)>0:
		line = lines[-1]
		humId = int((line.split(':')[1]).replace('hum', ''))
		hum = hums[humId-1]
		pos_str = line.split(':')[2]
		new_position = Position.parse_position(pos_str)
		new_position.x += const.VREP_X_OFFSET
		new_position.y += const.VREP_Y_OFFSET
	else:
		new_position = None

	hum.set_position(new_position)


def follow_fatigue(hums: List[Human]):
	filename = '../scene_logs/humanFatigue.log'
	f = open(filename, 'r')
	lines = f.read().splitlines()

	if len(lines)>0:
		line = lines[-1]
		humId = int((line.split(':')[1]).replace('hum', ''))
		hum = hums[humId-1]
		new_ftg = float((line.split(':')[2]))
	else:
		new_ftg = None

	hum.set_fatigue(new_ftg)

