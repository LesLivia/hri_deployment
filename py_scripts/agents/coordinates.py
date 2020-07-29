#!/usr/bin/env python

class Point:
	def __init__(self, x: float, y: float):
		self.x = float(x)
		self.y = float(y)

	def parse_point(posStr: str):
		fields = posStr.split('#')
		return Position(fields[0], fields[1])

	def __str__(self):
		return 'x: ' + str(self.x) + '	y: ' + str(self.y)
