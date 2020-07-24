#!/usr/bin/env python
from multiprocessing import Pool
import rospy_utils.hrirosnode as hriros

class MobileRobot:
	def __init__(self, rob_id, max_speed, max_accel):
		self.rob_id = rob_id
		self.max_speed = max_speed
		self.max_accel = max_accel

	def start_moving(self, targetSpeed):
		node = 'allMotorPub.py'

		pool = Pool()
		pool.starmap(hriros.rosrun_nodes, [(node, str(targetSpeed))])

	def stop_moving(self):
		node = 'allMotorPub.py'
		targetSpeed = '0.0'

		pool = Pool()
		pool.starmap(hriros.rosrun_nodes, [(node, str(targetSpeed))])



