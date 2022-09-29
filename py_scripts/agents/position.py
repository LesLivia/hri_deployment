#!/usr/bin/env python

class Position:
    def __init__(self, x: float, y: float, z: float, a: float, b: float, g: float):
        self.x = float(x)
        self.y = float(y)
        self.z = float(z)
        self.a = float(a)
        self.b = float(b)
        self.g = float(g)

    def parse_position(posStr: str):
        fields = posStr.split('#')
        return Position(float(fields[0]), float(fields[1]), float(fields[2]),
                        float(fields[3]), float(fields[4]), float(fields[5]))

    def __str__(self):
        return 'x: ' + str(self.x) + ' y: ' + str(self.y) + ' z: ' + str(self.z) + ' alpha: ' + str(
            self.a) + ' beta: ' + str(self.b) + ' gamma: ' + str(self.g)
