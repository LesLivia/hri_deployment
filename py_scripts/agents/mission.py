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
	def __init__(self, p: List[Pattern], dest: List[Point]):
		self.p = p
		self.dest = dest
		self.served = []
		for n in range(len(p)):   				# tot pattern. inizializza vettore con tot failure
			self.served.append(False)
		self.fail = False
		
	def get_scs(self):									# ti dice se è arrivato al successo
		for hum in self.served:
			if not hum:    								#se almeno uno è falso, entra dentro all if e ritorna false
				return False
		return True										# se tutti gli obiettivi sono raggiunti, ritorna true

	def set_served(self, index: int):				# metto una funzione in stato 'successo' e poi lo printo
		self.served[index] = True
		msg = 'SVD#' + str(index+1)
		self.publish_status(msg)
		
	def publish_status(self, msg):
		node = 'missionStatusPub.py'
		pool = Pool()
		pool.starmap(hriros.rosrun_nodes, [(node, [msg])])
		print('Robot stopping...')

		# questo da modificare se voglio aggiungere l id del rob che si sta fermando
