#!/usr/bin/env python
import configparser
import rospy_utils.hrirosnode as hriros
import rospy_utils.hriconstants as const
from multiprocessing import Pool
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
		self.rob_id = rob_id
		self.max_speed = max_speed
		self.max_accel = max_accel
		self.curr_speed = 0.0
		self.sim_running = 0
		self.LOGGER = Logger("ROBOT {}".format(rob_id))

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
		if ENV=='S':
			node = 'robSensorsSub.py'
		else:
			node = 'ttb3subpos.py'

		self.LOGGER.info('Subscribing to position data...')
		pool = Pool()
		pool.starmap(hriros.rosrun_nodes, [(node, [''])])

		# clear robot position log file
		f = open('../scene_logs/robotBattery.log', 'r+')
		f.truncate(0)
		f.close()
		# launch ROS node that subscribes to robot GPS data
		if ENV=='S':
			node = 'robBatterySub.py'
		else:
			node = 'ttb3subchg.py'

		self.LOGGER.info('Subscribing to charge data...')
		pool = Pool()
		pool.starmap(hriros.rosrun_nodes, [(node, [''])])

		self.set_sim_running(1)

	def follow_position(self):
		filename = '../scene_logs/robotPosition.log'
		f = open(filename, 'r')
		lines = f.read().splitlines()
		if len(lines)>0:
			last_line = lines[-1]
			new_pos = Position.parse_position(last_line.split(':')[1])

			# VRep layout origin is different from the
			# one in the Uppaal model: translation is necessary
			new_pos.x += const.VREP_X_OFFSET
			new_pos.y += const.VREP_Y_OFFSET
		else:
			new_pos = None

		self.LOGGER.debug('Updating position to ({:.2f}, {:.2f})'.format(new_pos.x, new_pos.y))
		self.set_position(new_pos)

	def follow_charge(self):
		filename = '../scene_logs/robotBattery.log'
		f = open(filename, 'r')
		lines = f.read().splitlines()
	
		if len(lines)>0:
			last_line = lines[-1]
			new_charge = float(last_line.split(':')[1])
		else:
			new_charge = None

		self.LOGGER.debug('Updating charge to ({:.2f})'.format(new_charge))
		self.set_charge(new_charge)
	
	def start_moving(self, targetSpeed):
		node = 'robStatusPub.py'
		data = '1#'

		if targetSpeed > 0:
		    data = data + str(targetSpeed)
		# requested target speed is published to both robot motors,
		# so that the robot starts moving straight
		pool = Pool()
		pool.starmap(hriros.rosrun_nodes, [(node, [data])])
		self.curr_speed = targetSpeed
		self.LOGGER.info('Instructing robot to start moving (target speed {:.2f})...'.format(targetSpeed))

	def stop_moving(self):
		node = 'robStatusPub.py'
		data = '0#0.0'
		# both motors speed is set to 0, so that the robot stops moving
		pool = Pool()
		pool.starmap(hriros.rosrun_nodes, [(node, [data])])
		self.curr_speed = 0.0
		self.LOGGER.info('Instructing robot to stop moving...')

