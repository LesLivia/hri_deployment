#!/usr/bin/env python
import os
import rospy_utils.hrirosnode as hriros
import rospy_utils.hriconstants as const
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

	def start_reading_data(self):
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

		self.set_sim_running(1)

	def follow_position(self):
		filename = '../scene_logs/humanPosition.log'
		_cached_stamp = 0
		_cached_pos = None
		while self.is_sim_running():
			try:
				stamp = os.stat(filename).st_mtime
				if stamp != _cached_stamp:
					f = open(filename, 'r')
					lines = f.read().splitlines()
					last_line = lines[-1]
					if self.get_position():
						_cached_pos = self.get_position()
					new_position = Position.parse_position(last_line)
					new_position.x += const.VREP_X_OFFSET
					new_position.y += const.VREP_Y_OFFSET

					self.set_position(new_position)

					if _cached_pos is not None:
						_cached_pt = Point(_cached_pos.x, _cached_pos.y)
						new_pt = Point(new_position.x, new_position.y)
						if self.is_moving() and new_pt.distance_from(_cached_pt) < 0.5:
							print('Human is idle')
							self.set_is_moving(False)
						elif not self.is_moving() and new_pt.distance_from(_cached_pt) >= 0.5:
							print('Human is moving')
							self.set_is_moving(True)
					_cached_stamp = stamp
			except (KeyboardInterrupt, SystemExit):
				print('Stopping human position monitoring...')
				return

	def follow_fatigue(self):
		filename = '../scene_logs/humanFatigue.log'
		_cached_stamp = 0
		while self.is_sim_running():
			try:
				stamp = os.stat(filename).st_mtime
				if stamp != _cached_stamp:
					f = open(filename, 'r')
					lines = f.read().splitlines()
					last_line = lines[-1]
					new_ftg = float(last_line)

					self.set_fatigue(new_ftg)
					_cached_stamp = stamp
			except (KeyboardInterrupt, SystemExit):
				print('Stopping human fatigue monitoring...')
				return


