#!/usr/bin/env python
from enum import Enum
from typing import List
from agents.coordinates import Point
from multiprocessing import Pool
import rospy_utils.hrirosnode as hriros
import rospy_utils.hriconstants as const

class Pattern(Enum):
	HUM_FOLLOWER = 0
	HUM_LEADER = 1
	HUM_RECIPIENT = 2

class Mission:
	def __init__(self, p: List[Pattern], dest: List[Point], start: List[Point]=[]):
		self.p = p
		self.dest = dest
		self.start = start if len(start)>0 else [None]*len(p)
		self.served = [False]*len(p)
		self.fail = False
		
	def get_scs(self):	
		for hum in self.served:
			if not hum:
				return False
		return True
	# SET IF HUMAN HAVE BEEN SERVED
	def set_served(self, index: int):
		self.served[index] = True
		msg = 'SVD#' + str(index+1)
		self.publish_status(msg)
		
	def publish_status(self, msg):
		node = 'missionStatusPub.py'
		pool = Pool()
		pool.starmap(hriros.rosrun_nodes, [(node, [msg])])
		print('Robot stopping...')


