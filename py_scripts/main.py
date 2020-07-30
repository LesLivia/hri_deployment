#!/usr/bin/env python
import time
import os
import sys
import rospy_utils.hrirosnode as hriros
import vrep_utils.vrep as vrep
from threading import Thread
from agents.mobilerobot import MobileRobot
from agents.human import Human
from agents.coordinates import Point

print('Launching application...')

vrep_sim = vrep.connect(19999)
vrep.start_sim(vrep_sim)

#bill = Human(1, 0, 10, 1, 1)

#bill.start_reading_position()
#time.sleep(1)
#thread_h = Thread(target = bill.follow_position)
#thread_h.start()

rob = MobileRobot(1, 10, 5)

rob.start_reading_position()
time.sleep(2)
thread_r = Thread(target = rob.follow_position)
thread_r.start()

time.sleep(1)
dest = Point(10.0, 2.5)
rob.navigate_to(dest)
#rob.start_moving(rob.max_speed)
#time.sleep(5)
#rob.stop_moving()
#time.sleep(1)
#rob.turn_left(1.57)
#time.sleep(5)
#rob.turn_right(-1.57)
#time.sleep(5)
#rob.start_moving(rob.max_speed*3)
#time.sleep(5)
#rob.stop_moving()

rob.set_sim_running(0)
vrep.stop_sim(vrep_sim)
quit()
print('Execution finished.')






