#!/usr/bin/env python
import os
from multiprocessing import Pool
import rospy_utils.hrirosnode as hriros

class Human:
	def __init__(self, hum_id, ptrn, speed, ftg_profile, fw_profile):
		self.hum_id = hum_id
		self.ptrn = ptrn
		self.speed = speed
		self.ftg_profile = ftg_profile
		self.fw_profile = fw_profile

	def start_reading_position(self):
		f = open('../scene_logs/humanPosition.log', 'r+')
		f.truncate(0)

		node = 'humSensorsSub.py'

		pool = Pool()
		pool.starmap(hriros.rosrun_nodes, [(node, '')])

	def follow_position(self):
		filename = '../scene_logs/humanPosition.log'
		_cached_stamp = 0
		while True:
			stamp = os.stat(filename).st_mtime
			if stamp != _cached_stamp:
				f = open(filename, 'r')
				lines = f.read().splitlines()
				last_line = lines[-1]
				print(last_line)
				_cached_stamp = stamp


