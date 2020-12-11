#!/usr/bin/env python
import os
import rospy_utils.hrirosnode as hriros
import rospy_utils.hriconstants as const
from typing import List
from multiprocessing import Pool
from agents.position import Position
from agents.coordinates import Point

class Human:
	def __init__(self, hum_id, speed, ftg_profile, fw_profile):
		self.hum_id = hum_id
		self.speed = speed
		self.ftg_profile = ftg_profile
		self.fw_profile = fw_profile
		self.moving = False
		self.position = None

	def set_position(self, position: Position):
		self.position = position

	def get_position(self):
		return self.position

	def set_fatigue(self, ftg: float):
		self.fatigue = ftg

	def get_fatigue(self):
		return self.fatigue

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

	node = 'humSensorsSub.py'

	pool = Pool()
	pool.starmap(hriros.rosrun_nodes, [(node, '')])

	f = open('../scene_logs/humanFatigue.log', 'r+')
	f.truncate(0)

	node = 'humFtgSub.py'

	pool = Pool()
	pool.starmap(hriros.rosrun_nodes, [(node, '')])

	f = open('../scene_logs/humansServed.log', 'r+')
	f.truncate(0)

	node = 'humServiceSub.py'

	pool = Pool()
	pool.starmap(hriros.rosrun_nodes, [(node, '')])

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
				humId = int((line.split(':')[0]).replace('hum', ''))
				hum = hums[humId-1]
				pos_str = line.split(':')[1]
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
				humId = int((line.split('#')[1]).replace('hum', ''))
				hum = hums[humId-1]
				new_ftg = float(line.split('#')[2])
				hum.set_fatigue(new_ftg)
			_cached_stamp = stamp
			_last_read_line = len(lines)-1
