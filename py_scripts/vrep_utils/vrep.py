#!/usr/bin/env python
import vrep_utils.lib.vrep.vrep as vrep

def connect(port):
	print('Connecting to VRep...')
	clientID=vrep.simxStart('127.0.0.1',port,True,True,5000,5)
	if clientID!=-1:
		print('Connection successfully established.')
	else:
		print('Connection to VRep could not be established.')





