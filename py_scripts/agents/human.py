#!/usr/bin/env python
import os
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
		self.emg = []
		self.lambdas = []

	def set_position(self, position: Position):
		self.position = position

	def get_position(self):
		return self.position

	def set_fatigue(self, ftg: float):
		self.fatigue = ftg

	def get_fatigue(self):
		return self.fatigue

	def set_emg_signal(self, emg: List[float]):
		self.emg += emg

	def get_emg_signal(self):
		return self.emg

	def set_lambdas(self, l: float):
		self.lambdas.append(l)

	def get_lambdas(self):
		return self.lambdas

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

				if _cached_pos is not None:
					_cached_pt = Point(_cached_pos.x, _cached_pos.y)
					new_pt = Point(new_position.x, new_position.y)
					if hum.is_moving() and new_pt.distance_from(_cached_pt) < 0.5:
						# print('Human is idle')
						hum.set_is_moving(False)
					elif not hum.is_moving() and new_pt.distance_from(_cached_pt) >= 0.5:
						# print('Human is moving')
						hum.set_is_moving(True)

			_cached_stamp = stamp
			_last_read_line = len(lines)-1

def emg_to_ftg(hum: Human, initial_guess=None, cf=0):
	SAMPLING_RATE = 1080
	signal_mv = hum.get_emg_signal()
	est_lambdas = initial_guess

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

		# First, design the Buterworth filter
		# N = 3  # Filter order
		# Wn = 0.3  # Cutoff frequency
		# B, A = signal.butter(N, Wn, output='ba')
		# smooth_data = signal.filtfilt(B, A, mean_freq_data)
		mean_freq_data = [i * (1 - cf * index) for (index, i) in enumerate(mean_freq_data)]

		bursts = b_e / SAMPLING_RATE
		q, m, x, est_values = emg_mgr.mnf_lin_reg(mean_freq_data, bursts)
		if m < 0:
			est_lambda = math.fabs(m)
		else:
			est_lambda = initial_guess
		MET = math.log(1 - 0.05) / -est_lambda
		# print('ESTIMATED RATE: {:.6f}, MET: {:.2f}min'.format(est_lambda, MET))
	except ValueError:
		# print('Insufficient EMG bursts ({})'.format(len(b_s)))
		est_lambda = initial_guess
		MET = math.log(1 - 0.05) / -est_lambda
		# print('ESTIMATED RATE: {:.6f}, MET: {:.2f}min'.format(est_lambda, MET))

	hum.set_lambdas(est_lambda)
	t = len(signal_mv)/SAMPLING_RATE
	all_lambdas = hum.get_lambdas()
	avg_lambda = sum(all_lambdas)/len(all_lambdas)
	# print('avg lambda so far: {}'.format(avg_lambda))
	F = 1 - math.exp(-avg_lambda * t)
	filename = '../scene_logs/emg_to_ftg.log'
	f = open(filename, 'a')
	f.write(str(F)+'\n')
	f.close()
	return F

def follow_fatigue(hums: List[Human]):
	filename = '../scene_logs/humanFatigue.log'
	_cached_stamp = 0
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
				new_emg_pts = line.split(':')[2].split('#')
				new_emg_pts = [float(pt) for pt in new_emg_pts[:len(new_emg_pts)-2]]
				hum.set_emg_signal(new_emg_pts)
				new_ftg = emg_to_ftg(hum, 0.0005, 0.0001)
				hum.set_fatigue(new_ftg)
			_cached_stamp = stamp
			_last_read_line = len(lines)-1

