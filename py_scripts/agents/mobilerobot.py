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
				#print('Robot: ' + str(self.get_position()))
				_cached_stamp = stamp

	def start_moving(self, targetSpeed):
		print('Robot moving forward...')

		node = 'allMotorPub.py'

		pool = Pool()
		pool.starmap(hriros.rosrun_nodes, [(node, str(targetSpeed))])

	def stop_moving(self):
		print('Robot stopping...')

		node = 'allMotorPub.py'
		targetSpeed = '0.0'

		pool = Pool()
		pool.starmap(hriros.rosrun_nodes, [(node, str(targetSpeed))])

	def turn_left(self, deg: float):
		node = 'rightMotorPub.py'
		
		orientStart = float(self.get_position().g)
		orientDest = float(orientStart+deg)
		epsilon = 0.4
		if abs(orientStart-orientDest) >= epsilon:
			print('Robot turning ' + str(deg) + 'rad left...')
			pool = Pool()
			pool.starmap(hriros.rosrun_nodes, [(node, str(self.max_speed/10))])

			orientCurr = float(orientStart)

			while abs(orientCurr-orientDest) >= epsilon:
				orientCurr = float(self.get_position().g)

			print('Robot current orientation: ' + str(orientCurr))
			print('Stop turning...') 
			pool.starmap(hriros.rosrun_nodes, [(node, str(0))])
		
	def turn_right(self, deg: float):
		node = 'leftMotorPub.py'
		
		orientStart = float(self.get_position().g)
		orientDest = float(orientStart+deg)
		epsilon = 0.4
		if abs(orientStart-orientDest) >= epsilon:
			print('Robot turning ' + str(deg) + 'rad right...')
			pool = Pool()
			pool.starmap(hriros.rosrun_nodes, [(node, str(self.max_speed/10))])

			orientCurr = float(orientStart)

			while abs(orientCurr-orientDest) >= epsilon:
				orientCurr = float(self.get_position().g)

			print('Robot current orientation: ' + str(orientCurr))
			print('Stop turning...') 
			pool.starmap(hriros.rosrun_nodes, [(node, str(0))])


