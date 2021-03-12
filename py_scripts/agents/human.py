#!/usr/bin/env python
import os
import time
import rospy_utils.hrirosnode as hriros
import rospy_utils.hriconstants as const
import agents.mgrs.emg_mgr as emg_mgr
import math
import biosignalsnotebooks as bsnb
import numpy as np
import scipy.io
from scipy import signal
from scipy.signal import periodogram
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
	_cached_stamp = 0
	_cached_pos = None
	_last_read_line = 1
	while hums[0].is_sim_running():
		stamp = os.stat(filename).st_mtime
		if stamp != _cached_stamp:
			f = open(filename, 'r')
			lines = f.read().splitlines()
			new_lines = lines[_last_read_line:]
			for line in new_lines:
				humId = int((line.split(':')[1]).replace('hum', ''))
				hum = hums[humId-1]
				pos_str = line.split(':')[2]
				if hum.get_position():
					_cached_pos = hum.get_position()
				new_position = Position.parse_position(pos_str)
				new_position.x += const.VREP_X_OFFSET
				new_position.y += const.VREP_Y_OFFSET

				hum.set_position(new_position)
			_cached_stamp = stamp
			_last_read_line = len(lines)-1


def overlaps(burst: List[float], prevs: List[List[float]]):
    for cand in prevs:
        if cand[0] > burst[1]:
            continue
        else:
            return cand

    return None


def emg_to_ftg(hum: Human, def_lambda=None, def_mu=None):
	SAMPLING_RATE = 1080
	T_POLL = 2
	state = 'm' if hum.is_moving() else 'r'
	signal_mv = hum.get_emg_signal(state)
	if hum.is_moving():
		est_rate = hum.get_lambdas()[-1] if len(hum.get_lambdas())>0 else def_lambda
		definitive_bursts = hum.def_bursts_mov
		candidate_bursts = hum.cand_bursts_mov
	else:  
		est_rate = hum.get_mus()[-1] if len(hum.get_mus())>0 else def_mu
		definitive_bursts = hum.def_bursts_rest
		candidate_bursts = hum.cand_bursts_rest

	cf = 0.0001 if hum.ftg_profile in [1, 3] else 0.001

	start = definitive_bursts[len(definitive_bursts) - 1][1] if len(definitive_bursts) > 0 else 0
	sig = signal_mv[start:]
	try:
		b_s, b_e = emg_mgr.get_bursts(sig, SAMPLING_RATE)
		for (index, b) in enumerate(b_s):
			adj_burst = [b + start, b_e[index] + start]
			best_fit = overlaps(adj_burst, candidate_bursts)
			if best_fit is None:
				candidate_bursts.append(adj_burst)
			elif best_fit is not None:
				definitive_bursts.append(adj_burst)
				candidate_bursts.remove(best_fit)

		bursts_start = [burst[0] for burst in definitive_bursts]
		bursts_end = [burst[1] for burst in definitive_bursts]
		mnf = emg_mgr.calculate_mnf(signal_mv, SAMPLING_RATE, cf=cf, b_s=bursts_start, b_e=bursts_end)
		q, m, x, est_values = emg_mgr.mnf_lin_reg(mnf, [x / SAMPLING_RATE for x in bursts_end])
		if (hum.is_moving() and m >= 0) or ((not hum.is_moving()) and m <= 0):
		    raise ValueError
		print('EST RATE: {}'.format(math.fabs(m)))
		if hum.is_moving():
			hum.set_lambdas(math.fabs(m))
		else:
			hum.set_mus(math.fabs(m))
	except ValueError:
		if hum.is_moving():
			hum.set_lambdas(est_rate)
		else:
			hum.set_mus(est_rate)

	all_rates = hum.get_lambdas() if hum.is_moving() else hum.get_mus()
	avg_rate = sum(all_rates)/len(all_rates) if len(all_rates)>0 else est_rate

	t = SIM_T - hum.get_last_switch()
	t = max(0.0, t)
	F_0 = hum.get_f_o()
	x = np.arange(0.0, t, T_POLL)
	if hum.is_moving():
		y = [1 - (1-F_0)*math.exp(-avg_rate*i) for i in x]
	else:
		y = [F_0*math.exp(-avg_rate*i) for i in x]
	F = y[-1] if len(y)>0 else F_0

	filename = '../scene_logs/emg_to_ftg{}.log'.format(hum.hum_id)
	f = open(filename, 'a')
	ftg_str = ''
	for i in y:
		ftg_str += '{:.4f}#'.format(i)
	f.write('hum{}: ({}, {:.6f}, {:.6f}, {:.4f}) {}\n'.format(hum.hum_id, state, avg_rate, F_0, t, ftg_str))
	f.close()

	return F


def follow_fatigue(hums: List[Human]):
	T_POLL = 2.0
	global SIM_T
	SIM_T = 0.0
	filename = '../scene_logs/humanFatigue.log'
	_cached_stamp = 0
	_cached_status = [False]*len(hums)
	_last_read_line = 1
	while hums[0].is_sim_running():
		stamp = os.stat(filename).st_mtime
		if stamp != _cached_stamp:
			f = open(filename, 'r')
			lines = f.read().splitlines()
			new_lines = lines[_last_read_line:]
			for line in new_lines:
				SIM_T = float(line.split(':')[0])
				humId = int((line.split(':')[1]).replace('hum', ''))
				hum = hums[humId-1]
				new_status = None
				if line.split(':')[2]=='m':
					new_status = True 
				elif line.split(':')[2]=='r': 
					new_status = False
				if new_status is not None and new_status!=_cached_status[humId-1]:
					print('human switched to {} {} at {}'.format(new_status, line.split(':')[2], line.split(':')[0]))
					hum.set_f_o(hum.get_fatigue())
					hum.set_is_moving(new_status)	
					hum.set_last_switch(float(line.split(':')[0])-T_POLL)	
					_cached_status[humId-1] = new_status		
				new_emg_pts = line.split(':')[3].split('#')
				new_emg_pts = [float(pt) for pt in new_emg_pts[:len(new_emg_pts)-2]]
				hum.set_emg_signal(line.split(':')[2], new_emg_pts)
				default_rates = hum.ftg_profile.get_def_rates()
				new_ftg = emg_to_ftg(hum, def_lambda=default_rates[0], def_mu=default_rates[1])
				hum.set_fatigue(new_ftg)
			_cached_stamp = stamp
			_last_read_line = len(lines)-1

