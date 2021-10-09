#!/usr/bin/env python
import math


class Point:
    def __init__(self, x: float, y: float):
        self.x = float(x)
        self.y = float(y)

    def __str__(self):
        return 'x: ' + str(self.x) + '	y: ' + str(self.y)

    def distance_from(self, pos2 ):
        result = float(math.sqrt((self.x - pos2.x) ** 2 + (self.y - pos2.y) ** 2))
        return result

    def parse_point(posStr: str):
        fields = posStr.split('#')
        return Point(float(fields[0]), float(fields[1]))

