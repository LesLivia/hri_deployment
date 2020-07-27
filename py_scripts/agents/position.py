#!/usr/bin/env python

class Position:
	def __init__(self, x: float, y: float, z: float, a: float, b: float, g: float):
		self.x = x
		self.y = y
		self.z = z
		self.a = a
		self.b = b
		self.g = g

	def parse_position(posStr: str):
		fields = posStr.split('#')
		return Position(fields[0], fields[1], fields[2], fields[3], fields[4], fields[5])

	def __str__(self):
		return 'x: ' + str(self.x) + 'y: ' + str(self.y) + 'z: ' + str(self.z) + 'alpha: ' + str(self.a) + 'beta: ' + str(self.b) + 'gamma: ' + str(self.g)
