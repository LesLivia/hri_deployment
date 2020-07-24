#!/usr/bin/env python
import rospy_utils.hrirosnode as hriros
import time
from itertools import repeat
from multiprocessing import Pool

print('Launching application...')

#Run Robot motors actuation nodes
nodes = 'allMotorPub.py'
speed = '1.0'

pool = Pool()
pool.starmap(hriros.rosrun_nodes, [(nodes, speed)])

time.sleep(10)

speed = '0.0'
pool = Pool()
pool.starmap(hriros.rosrun_nodes, [(nodes, speed)])

print('Application stopped.')
