#!/usr/bin/env python
import os
import time
import rospy_utils.hrirosnode as hriros
import rospy_utils.hriconstants as const
import agents.navigation as nav
from multiprocessing import Pool
from agents.position import Position
from agents.coordinates import Point
from datetime import datetime


class MobileRobot:
	def __init__(self, rob_id, max_speed, max_accel):
		self.rob_id = rob_id
		self.max_speed = max_speed
		self.max_accel = max_accel
		self.curr_speed = 0.0
		self.sim_running = 0

	def set_position(self, position: Position):
		self.position = position

	def get_position(self):
		return self.position

	def set_charge(self, charge: float):
		self.charge = charge

	def get_charge(self):
		return self.charge

	def set_sim_running(self, run):
		self.sim_running = run

	def is_sim_running(self):
		return self.sim_running

	def start_reading_data(self):
		# clear robot position log file
		f = open('../scene_logs/robotPosition.log', 'r+')
		f.truncate(0)
		f.close()
		# launch ROS node that subscribes to robot GPS data
		node = 'robSensorsSub.py'

		pool = Pool()
		pool.starmap(hriros.rosrun_nodes, [(node, [''])])

		# clear robot position log file
		f = open('../scene_logs/robotBattery.log', 'r+')
		f.truncate(0)
		f.close()
		# launch ROS node that subscribes to robot GPS data
		node = 'robBatterySub.py'

		pool = Pool()
		pool.starmap(hriros.rosrun_nodes, [(node, [''])])

		self.set_sim_running(1)

	def follow_position(self):
		filename = '../scene_logs/robotPosition.log'
		_cached_stamp = 0
		while self.is_sim_running():
			try:
				# when a new line is written to log file,
				# robot position attribute is updated as well
				# -> this needs to be continuously running in a
				# parallel thread
				stamp = os.stat(filename).st_mtime
				if stamp != _cached_stamp:
					f = open(filename, 'r')
					lines = f.read().splitlines()

					last_line = lines[-1]
					newPos = Position.parse_position(last_line.split(':')[1])

					# VRep layout origin is different from the
					# one in the Uppaal model: translation is necessary
					newPos.x += const.VREP_X_OFFSET
					newPos.y += const.VREP_Y_OFFSET

					self.set_position(newPos)
					_cached_stamp = stamp
			except (KeyboardInterrupt, SystemExit):
				print('Stopping robot position monitoring...')
				return

	def follow_charge(self):
		filename = '../scene_logs/robotBattery.log'
		_cached_stamp = 0
		while self.is_sim_running():
			try:
				# when a new line is written to log file,
				# robot charge attribute is updated as well
				# -> this needs to be continuously running in a
				# parallel thread
				stamp = os.stat(filename).st_mtime
				if stamp != _cached_stamp:
					f = open(filename, 'r')
					lines = f.read().splitlines()

					last_line = lines[-1]
					new_charge = float(last_line.split(':')[1])

					now = datetime.now()
					current_time = now.strftime("%H:%M:%S")
					# print("Current Time =", current_time)

					self.set_charge(new_charge)
					_cached_stamp = stamp
			except (KeyboardInterrupt, SystemExit):
				print('Stopping robot battery charge monitoring...')
				return
	
	def start_moving(self, targetSpeed):
		node = 'robStatusPub.py'
		data = '1#'
		print(data)
		if targetSpeed > 0:
		    data = data + str(targetSpeed)
		# requested target speed is published to both robot motors,
		# so that the robot starts moving straight
		pool = Pool()
		pool.starmap(hriros.rosrun_nodes, [(node, [data])])
		self.curr_speed = targetSpeed
		print('Robot moving...')

	def stop_moving(self):
		node = 'robStatusPub.py'
		data = '0#0.0'
		# both motors speed is set to 0, so that the robot stops moving
		pool = Pool()
		pool.starmap(hriros.rosrun_nodes, [(node, [data])])
		self.curr_speed = 0.0
		print('Robot stopping...')

