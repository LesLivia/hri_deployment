#!/usr/bin/env python
from enum import Enum
from typing import List
from agents.coordinates import Point

class Pattern(Enum):
	HUM_FOLLOWER = 0
	HUM_LEADER = 1
	HUM_RECIPIENT = 2

class Mission:
	def __init__(self, p: List[Pattern], dest: List[Point]):
		self.p = p
		self.dest = dest
		self.served = []
		for n in range(len(p)):
			self.served.append(False)
		self.fail = False
		
	def get_scs(self):	
		for hum in self.served:
			if not hum:
				return False
		return True

	def set_served(self, index: int):
		self.served[index] = True

