#!/usr/bin/env python
import os
import rospy_utils.hrirosnode as hriros
import rospy_utils.hriconstants as const
from multiprocessing import Pool
from agents.position import Position

class MobileRobot:
	def __init__(self, rob_id, max_speed, max_accel):
		self.rob_id = rob_id
		self.max_speed = max_speed
		self.max_accel = max_accel

	def set_position(self, position: Position):
		self.position = position

	def get_position(self):
		return self.position

	def start_reading_position(self):
		f = open('../scene_logs/robotPosition.log', 'r+')
		f.truncate(0)

		node = 'robSensorsSub.py'

		pool = Pool()
		pool.starmap(hriros.rosrun_nodes, [(node, '')])

	def follow_position(self):
		filename = '../scene_logs/robotPosition.log'
		_cached_stamp = 0
		while True:
			stamp = os.stat(filename).st_mtime
			if stamp != _cached_stamp:
				f = open(filename, 'r')
				lines = f.read().splitlines()
				last_line = lines[-1]
				self.set_position(Position.parse_position(last_line))
				print(str(self.get_position()))
				_cached_stamp = stamp

	def start_moving(self, targetSpeed):
		node = 'allMotorPub.py'

		pool = Pool()
		pool.starmap(hriros.rosrun_nodes, [(node, str(targetSpeed))])

	def stop_moving(self):
		node = 'allMotorPub.py'
		targetSpeed = '0.0'

		pool = Pool()
		pool.starmap(hriros.rosrun_nodes, [(node, str(targetSpeed))])

	def turn_left(self):
		node = 'rightMotorPub.py'

		pool = Pool()
		pool.starmap(hriros.rosrun_nodes, [(node, str(self.max_speed/2))])

	def turn_right(self):
		node = 'leftMotorPub.py'

		pool = Pool()
		pool.starmap(hriros.rosrun_nodes, [(node, str(self.max_speed/2))])


