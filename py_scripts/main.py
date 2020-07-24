#!/usr/bin/env python

import subprocess
from multiprocessing import Pool

def rosrun_nodes(node):
    command = "rosrun hri_scenarios {}".format(node)
    subprocess.Popen(command, shell=True)

def roskill_nodes(node):
    command = "rosrun hri_scenarios {}".format(node)
    subprocess.Popen(command, shell=True)


print('Launching application...')

#Run Robot motors actuation nodes
nodes = ['leftMotorPub.py', 'rightMotorPub.py']

pool = Pool()
pool.map(rosrun_nodes, nodes)

cmd = ['rosnode', 'list']
result = subprocess.run( cmd, stdout=subprocess.PIPE )
print(result.stdout)

print('Application stopped.')
