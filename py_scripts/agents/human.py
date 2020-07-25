#!/usr/bin/env python
from multiprocessing import Pool
import rospy_utils.hrirosnode as hriros

class Human:
	def __init__(self, hum_id, ptrn, speed, ftg_profile, fw_profile):
		self.hum_id = hum_id
		self.ptrn = ptrn
		self.speed = speed
		self.ftg_profile = ftg_profile
		self.fw_profile = fw_profile

	def start_moving(self, targetSpeed):
		node = 'allMotorPub.py'

		pool = Pool()
		pool.starmap(hriros.rosrun_nodes, [(node, str(targetSpeed))])

	def stop_moving(self):
		node = 'allMotorPub.py'
		targetSpeed = '0.0'

		pool = Pool()
		pool.starmap(hriros.rosrun_nodes, [(node, str(targetSpeed))])



