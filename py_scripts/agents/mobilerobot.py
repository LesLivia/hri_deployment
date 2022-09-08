#!/usr/bin/env python

import os
import configparser
import rospy_utils.hrirosnode as hriros
import rospy_utils.hriconstants as const
from multiprocessing import Pool			# where are they?
from agents.position import Position
from agents.coordinates import Point
from datetime import datetime
from utils.logger import Logger

config = configparser.ConfigParser()
config.read('./resources/config.ini')
config.sections()
ENV = config['DEPLOYMENT ENVIRONMENT']['ENV']

class MobileRobot:
	def __init__(self, rob_id, max_speed, max_accel):
		self.rob_id = rob_id			# main characteristics of the robot
		self.max_speed = max_speed
		self.max_accel = max_accel
		self.last_time = 0.0
		self.last_time_charge = 0.0
		self.curr_speed = 0.0
		self.sim_running = 0
		self.LOGGER = Logger("ROBOT {}".format(rob_id))

	def set_position(self, position: Position):	# define set and get functions:
		self.position = position				# set -> associate the property to the current value
												# get -> return the current value of the property
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

	def set_last_time(self, time):
		self.last_time = time

	def get_last_time(self):
		return self.last_time

	def set_last_time_charge(self, time):
		self.last_time_charge = time

	def get_last_time_charge(self):
		return self.last_time_charge

	# READ DATA FROM POSITION AND CHARGE LOG
	def start_reading_data(self):
		# clear robot position log file								bisogna farlo perché potrebbe essere ancora scritto dall'altra volta
		if self.rob_id == '0':
			f = open('../scene_logs/robotPosition.log', 'r+')
			f.truncate(0)
			f.close()
			f1 = open('../scene_logs/robotDistance.log', 'r+')
			f1.truncate(0)
			f1.close()
			f2 = open('../scene_logs/robotBattery.log', 'r+')
			f2.truncate(0)
			f2.close()
			f3 = open("../scene_logs/humanPosition.log", "a")
			f3.truncate(0)
			f3.close()
			f4 = open("../scene_logs/humanFatigue.log", "a")
			f4.truncate(0)
			f4.close()

		# launch ROS node that subscribes to robot GPS data
		if ENV=='S':
			node = 'robSensorsSub.py'
		else:
			node = 'ttb3subpos.py'

		self.LOGGER.info('Subscribing to position data...')
		pool = Pool()
		pool.starmap(hriros.rosrun_nodes, [(node, [self.rob_id])])

		# launch ROS node that subscribes to robot GPS data
		if ENV=='S':
			node = 'robBatterySub.py'
		else:
			node = 'ttb3subchg.py'
		self.LOGGER.info('Subscribing to charge data...')
		pool = Pool()
		pool.starmap(hriros.rosrun_nodes, [(node, [self.rob_id])])

		#############################
		node = 'checkDist.py'
		self.LOGGER.info('Subscribing to distance data...')
		pool = Pool()
		pool.starmap(hriros.rosrun_nodes, [(node, [self.rob_id])])
		self.set_sim_running(1)								# the robot is running
		#############################

	# TRACK THE POSITION OF THE ROBOT
	def follow_position(self):
		filename = '../scene_logs/robotPosition.log'
		f = open(filename, 'r')
		lines = f.read().splitlines()
		if len(lines) > 2:
			# last_line = lines[-1]
			n_robots = 3
			flag = 0
			j = -n_robots
			while j in range(-n_robots, -1) and flag == 0:
				l = lines[j].split(':')
				if self.rob_id in l:
					index = l.index(self.rob_id)
					t1 = self.get_last_time()
					if float(l[index-1]) > t1:
						new_pos = Position.parse_position(l[index+1])
						if ENV=='S':
							new_pos.x += const.VREP_X_OFFSET
							new_pos.y += const.VREP_Y_OFFSET
						else:
							new_pos.x += const.REAL_X_OFFSET+6.05
							new_pos.y += const.REAL_Y_OFFSET+6.25
						self.set_position(new_pos)
						self.set_last_time(float(l[index-1]))
						self.LOGGER.debug('Updating position to ({:.2f}, {:.2f})'.format(new_pos.x, new_pos.y))
						flag = 1
				j = j + 1
		else:
			new_pos = None
			self.set_position(new_pos)
			self.LOGGER.debug('Updating position to ({:.2f}, {:.2f})'.format(new_pos.x, new_pos.y))


	# TRACK THE CHARGE OF THE ROBOT
	def follow_charge(self):
		#filename2 = '../scene_logs/robotOut.log'
		filename = '../scene_logs/robotBattery.log'
		f = open(filename, 'r')
		#f2 = open(filename2, 'a')
		lines = f.read().splitlines()
		if len(lines) > 2:
			n_robots = 3
			flag = 0
			j = -n_robots
			while j in range(-n_robots, -1) and flag == 0:
				l = lines[j].split(':')
				if self.rob_id in l:
					index = l.index(self.rob_id)
					t1 = self.get_last_time_charge()
					if float(l[index-1]) > t1:
						new_charge = float(l[index+1])
						#f2.write("TIME: " + str(t1) + "CHARGE" + str(new_charge))
						self.set_charge(new_charge)
						self.set_last_time_charge(float(l[index-1]))
						self.LOGGER.debug('Updating charge to ({:.2f})'.format(new_charge))
						flag = 1
				j = j + 1
		else:
			new_charge = 0
			self.set_charge(new_charge)
			self.LOGGER.debug('Updating charge to ({:.2f})'.format(new_charge))

	# START THE MOTION
	def start_moving(self, targetSpeed, dest:Point=None):
		if ENV=='S':
			# requested target speed is published to both robot motors, so that the robot starts moving straight
			node = 'robStatusPub.py'
			data = str(self.rob_id)+'#1#'

			if targetSpeed > 0:
				data = data + str(targetSpeed)
		else: 	
			node = 'ttb3cmdnav.py'
			data = '{:.4f} {:.4f}'.format(dest.x, dest.y)

		pool = Pool()
		pool.starmap(hriros.rosrun_nodes, [(node, [data])])
		self.curr_speed = targetSpeed
		self.LOGGER.info('Instructing robot to start moving (target speed {:.2f})...'.format(targetSpeed))

	# STOP THE MOTION
	def stop_moving(self):
		if ENV=='S':
			node = 'robStatusPub.py'
			data = str(self.rob_id)+'#0#0.0'
			pool = Pool()
			pool.starmap(hriros.rosrun_nodes, [(node, [data])])
		else:
			os.system("./resources/cancel_goal.sh")
		# both motors speed is set to 0, so that the robot stops moving
		self.curr_speed = 0.0
		self.LOGGER.info('Instructing robot to stop moving...')
