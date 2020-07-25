#!/usr/bin/env python
import time
from agents.mobilerobot import MobileRobot

print('Launching application...')

rob = MobileRobot(1, 10, 5)
rob.start_moving(rob.max_speed)
time.sleep(10)
rob.stop_moving()
time.sleep(1)
rob.turn_left()
time.sleep(5)
rob.turn_right()

print('Application stopped.')
