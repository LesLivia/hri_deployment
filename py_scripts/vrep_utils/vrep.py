#!/usr/bin/env python
import vrep_utils.lib.vrep.vrep as vrep
from agents.coordinates import Point

def connect(port):
	print('Connecting to VRep...')
	clientID = vrep.simxStart('127.0.0.1',port,True,True,5000,5)
	if clientID != -1:
		print('Connection successfully established.')
	else:
		print('Connection to VRep could not be established.')

	return clientID

def start_sim(clientID):
	result = vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot)

def stop_sim(clientID):
	result = vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)
	if result != vrep.simx_return_ok:
		print('Something went wrong while stopping the simulation')

def draw_point(clientID, pos: Point):
	vrep.simxCallScriptFunction(clientID, 'wall', vrep.sim_scripttype_childscript, 'draw_point', [], [pos.x, pos.y], '', '', vrep.simx_opmode_blocking)




