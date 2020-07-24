#!/usr/bin/env python
import time
import subprocess
from itertools import repeat
from multiprocessing import Pool

def rosrun_nodes(node, arg):
    command = "rosrun hri_scenarios " + node + " " + str(arg)
    subprocess.Popen(command, shell=True)

def roskill_nodes(node):
    command = "rosrun hri_scenarios {}".format(node)
    subprocess.Popen(command, shell=True)


print('Launching application...')

#Run Robot motors actuation nodes
nodes = ['leftMotorPub.py', 'rightMotorPub.py']
speed = 1.0

pool = Pool()
pool.starmap(rosrun_nodes, zip(nodes, repeat(speed)))

cmd = ['rosnode', 'list']
result = subprocess.run( cmd, stdout=subprocess.PIPE )
print(result.stdout)

time.sleep(10)

speed = 0.0
pool = Pool()
pool.starmap(rosrun_nodes, zip(nodes, repeat(speed)))

print('Application stopped.')
