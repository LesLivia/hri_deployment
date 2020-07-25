#!/usr/bin/env python
import time
import os
from threading import Thread
from agents.mobilerobot import MobileRobot
from agents.human import Human

print('Launching application...')

global appStatus
appStatus = 1

bill = Human(1, 0, 10, 1, 1)
bill.start_reading_position()
time.sleep(1)
thread = Thread(target = bill.follow_position)
thread.start()

rob = MobileRobot(1, 10, 5)
rob.start_moving(rob.max_speed)
time.sleep(10)
rob.stop_moving()
time.sleep(1)
rob.turn_left()
time.sleep(5)
rob.turn_right()

appStatus = 0

print('Application stopped.')
