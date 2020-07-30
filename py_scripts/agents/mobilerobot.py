#!/usr/bin/env python
import os
import rospy_utils.hrirosnode as hriros
import rospy_utils.hriconstants as const
import agents.navigation as nav
from multiprocessing import Pool
from agents.position import Position
from agents.coordinates import Point

class MobileRobot:
	def __init__(self, rob_id, max_speed, max_accel):
		self.rob_id = rob_id
		self.max_speed = max_speed
		self.max_accel = max_accel
		self.sim_running = 0

	def set_position(self, position: Position):
		self.position = position

	def get_position(self):
		return self.position
	
	def set_sim_running(self, run):
		self.sim_running = run	

	def is_sim_running(self):
		return self.sim_running	

	def start_reading_position(self):
		# clear robot position log file
		f = open('../scene_logs/robotPosition.log', 'r+')
		f.truncate(0)
		# launch ROS node that subscribes to robot GPS data
		node = 'robSensorsSub.py'
				
		pool = Pool()
		pool.starmap(hriros.rosrun_nodes, [(node, '')])
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
					newPos = Position.parse_position(last_line)

					# VRep layout origin is different from the 
					# one in the Uppaal model: translation is necessary
					newPos.x += const.VREP_X_OFFSET
					newPos.y += const.VREP_Y_OFFSET
		
					self.set_position(newPos)
					_cached_stamp = stamp
			except (KeyboardInterrupt, SystemExit):
				print('Stopping robot position monitoring...')
				return

	def start_moving(self, targetSpeed):
		node = 'allMotorPub.py'
		# requested target speed is published to both robot motors,
		# so that the robot starts moving straight
		pool = Pool()
		pool.starmap(hriros.rosrun_nodes, [(node, str(targetSpeed))])
		print('Robot moving forward...')

	def stop_moving(self):
		node = 'allMotorPub.py'
		targetSpeed = '0.0'
		# both motors speed is set to 0, so that the robot stops moving
		pool = Pool()
		pool.starmap(hriros.rosrun_nodes, [(node, str(targetSpeed))])
		print('Robot stopping...')

	def turn_left(self, deg: float):
		# if the robot needs to turn left/right, 
		# actuation is required for only one robot
		node = 'rightMotorPub.py'

		# the robot can only revolve around its z-axis,
		# therefore starting orientation is saved so that
		# the master knows when the rotation is complete
		# (orientDest has been reached)
		orientStart = float(self.get_position().g)
		orientDest = float(orientStart+deg)
		
		# this is necessary to account for ROS delay
		# in sending the "stop" command to VRep
		epsilon = 0.45
		if abs(orientStart-orientDest) >= epsilon:
			pool = Pool()
			pool.starmap(hriros.rosrun_nodes, [(node, str(self.max_speed/10))])
			print('Robot turning ' + str(deg) + 'rad left...')

			orientCurr = float(orientStart)
			# while the requested orientation is yet to be reached, 
			# keed acquiring robot orientation from GPS sensor
			while abs(orientCurr-orientDest) >= epsilon:
				orientCurr = float(self.get_position().g)

			# when the requested orientation has been reached,
			# motor speed is set to 0 and the robot stops turning
			pool.starmap(hriros.rosrun_nodes, [(node, str(0))])
			print('Robot current orientation: ' + str(orientCurr))
			print('Stop turning...') 
		
	def turn_right(self, deg: float):
		node = 'leftMotorPub.py'
		
		orientStart = float(self.get_position().g)
		orientDest = float(orientStart+deg)
		epsilon = 0.45
		if abs(orientStart-orientDest) >= epsilon:
			pool = Pool()
			pool.starmap(hriros.rosrun_nodes, [(node, str(self.max_speed/10))])
			print('Robot turning ' + str(deg) + 'rad right...')

			orientCurr = float(orientStart)

			while abs(orientCurr-orientDest) >= epsilon:
				orientCurr = float(self.get_position().g)

			pool.starmap(hriros.rosrun_nodes, [(node, str(0))])
			print('Robot current orientation: ' + str(orientCurr))
			print('Stop turning...') 

	
	def navigate_to(self, dest: Point):
		start = self.get_position()
		pos = Point(start.x, start.y)
		rob_theta = round(start.g, 2)
		print('Robot in: ' + str(pos) + ' current orientation: ' + str(rob_theta))
		print('navigating to: ' + str(dest))
		
		std_length = 10
		std_height = 3

		checks = nav.get_dir_to_check(pos, dest, rob_theta, std_length, std_height)
		print(checks)		
		if checks[2]:
			print('Robot should move forward')
			self.start_moving(3.0)

			while checks[2]:
				curr = self.get_position()
				pos = Point(curr.x, curr.y)
				checks = nav.get_dir_to_check(pos, dest, rob_theta, std_length, std_height)

			self.stop_moving()
			print(checks)		
			if checks[0]:
				print('Robot should turn left')
			elif checks[1]:
				print('Robot should turn right')
		
		















