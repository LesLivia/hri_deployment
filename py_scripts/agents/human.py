#!/usr/bin/env python
import os
import time
import rospy_utils.hrirosnode as hriros
import rospy_utils.hriconstants as const
import agents.mgrs.emg_mgr as emg_mgr
from typing import List
from multiprocessing import Pool
from agents.position import Position
from agents.coordinates import Point
import math
import biosignalsnotebooks as bsnb
import numpy as np
import scipy.io
from scipy import signal
from scipy.signal import periodogram


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
		self.lambdas = []
		self.mus = []

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

	f = open('../scene_logs/emg_to_ftg.log', 'r+')
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

				# if _cached_pos is not None:
					# _cached_pt = Point(_cached_pos.x, _cached_pos.y)
					# new_pt = Point(new_position.x, new_position.y)
					# if hum.is_moving() and new_pt.distance_from(_cached_pt) < 0.5:
						# print('Human is idle')
					#	hum.set_is_moving(False)
					# elif not hum.is_moving() and new_pt.distance_from(_cached_pt) >= 0.5:
						# print('Human is moving')
					#	hum.set_is_moving(True)

			_cached_stamp = stamp
			_last_read_line = len(lines)-1

def emg_to_ftg(hum: Human, def_lambda=None, def_mu=None, cf=0):
	SAMPLING_RATE = 1080
	T_POLL = 2
	state = 'm' if hum.is_moving() else 'r'
	signal_mv = hum.get_emg_signal(state)
	if hum.is_moving():
		est_rate = hum.get_lambdas()[-1] if len(hum.get_lambdas())>0 else def_lambda
	else:  
		est_rate = hum.get_mus()[-1] if len(hum.get_mus())>0 else def_mu

	b_s, b_e = emg_mgr.get_bursts(signal_mv, SAMPLING_RATE)
	try:
		mean_freq_data = []
		for (index, start) in enumerate(b_s):
			emg_pts = signal_mv[start: b_e[index]]
			freqs, power = periodogram(emg_pts, fs=SAMPLING_RATE)
			# MNF
			try:
	    			mnf = sum(freqs * power) / sum(power)
	    			mean_freq_data.append(math.log(mnf))
			except ZeroDivisionError:
	    			print('error in division')

		mean_freq_data = [i * (1 - cf * index) for (index, i) in enumerate(mean_freq_data)]

		bursts = b_e / SAMPLING_RATE
		q, m, x, est_values = emg_mgr.mnf_lin_reg(mean_freq_data, bursts)
		if (hum.is_moving() and m<0) or (not hum.is_moving() and m>0):		
			est_rate = math.fabs(m)
		# print('ESTIMATED RATE: {:.6f}'.format(est_rate))
	except ValueError:
		print('Insufficient EMG bursts ({})'.format(len(b_s)))		
		# print('ESTIMATED RATE: {:.6f}, MET: {:.2f}min'.format(est_lambda, MET))

	if hum.is_moving():
		hum.set_lambdas(est_rate)
	else:
		hum.set_mus(est_rate)

	t = (len(hum.get_emg_signal('m'))+len(hum.get_emg_signal('r')))/(SAMPLING_RATE*T_POLL) - hum.get_last_switch()
	print(str(hum.get_last_switch()) + ' ' + str(t))
	t = max(0.0, t)

	all_rates = hum.get_lambdas() if hum.is_moving() else hum.get_mus()
	avg_rate = sum(all_rates)/len(all_rates)
	# print('avg rate so far ({}): {}'.format(state, avg_rate))
	F_0 = hum.get_f_o()
	F = 1 - (1-F_0)*math.exp(-avg_rate*t) if hum.is_moving() else F_0*math.exp(-avg_rate*t)

	filename = '../scene_logs/emg_to_ftg.log'
	f = open(filename, 'a')
	f.write('({}, {:.4f}, {:.4f}, {:.2f}) {:.4f}\n'.format(state, avg_rate, F_0, t, F))
	f.close()

	return F

def follow_fatigue(hums: List[Human]):
	T_POLL = 2.0
	filename = '../scene_logs/humanFatigue.log'
	_cached_stamp = 0
	_cached_status = False
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
				new_status = None
				if line.split(':')[2]=='m':
					new_status = True 
				elif line.split(':')[2]=='r': 
					new_status = False
				if new_status is not None and new_status!=_cached_status:
					print('human switched to {} {} at {}'.format(new_status, line.split(':')[2], line.split(':')[0]))
					print('{} {}'.format(len(hum.get_emg_signal('m')), len(hum.get_emg_signal('r'))))
					print(line[:500])
					hum.set_f_o(hum.get_fatigue())
					hum.set_is_moving(new_status)	
					hum.set_last_switch(float(line.split(':')[0])-T_POLL)	
					_cached_status = new_status		
				new_emg_pts = line.split(':')[3].split('#')
				new_emg_pts = [float(pt) for pt in new_emg_pts[:len(new_emg_pts)-2]]
				hum.set_emg_signal(line.split(':')[2], new_emg_pts)
				new_ftg = emg_to_ftg(hum, def_lambda=0.0005, def_mu=0.0002, cf=0)
				hum.set_fatigue(new_ftg)
			_cached_stamp = stamp
			_last_read_line = len(lines)-1

