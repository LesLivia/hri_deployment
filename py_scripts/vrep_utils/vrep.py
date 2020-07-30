#!/usr/bin/env python
import vrep_utils.lib.vrep.vrep as vrep
import rospy_utils.hriconstants as const
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
	vrep.simxCallScriptFunction(clientID, 'wall', vrep.sim_scripttype_childscript, 'draw_point', [], [pos.x-const.VREP_X_OFFSET, pos.y-const.VREP_Y_OFFSET], '', '', vrep.simx_opmode_blocking)

def draw_line(clientID, pos1: Point, pos2: Point):
	vrep.simxCallScriptFunction(clientID, 'wall', vrep.sim_scripttype_childscript, 'draw_line', [], [pos1.x-const.VREP_X_OFFSET, pos1.y-const.VREP_Y_OFFSET, pos2.x-const.VREP_X_OFFSET, pos2.y-const.VREP_Y_OFFSET], '', '', vrep.simx_opmode_blocking)

def clear_lines(clientID):
	vrep.simxCallScriptFunction(clientID, 'wall', vrep.sim_scripttype_childscript, 'clear_line', [], [], '', '', vrep.simx_opmode_blocking)

def draw_rectangle(clientID, pos1: Point, pos2: Point, pos3: Point, pos4: Point):
	clear_lines(const.VREP_CLIENT_ID)
	draw_line(const.VREP_CLIENT_ID, pos1, pos2)
	draw_line(const.VREP_CLIENT_ID, pos2, pos3)
	draw_line(const.VREP_CLIENT_ID, pos3, pos4)
	draw_line(const.VREP_CLIENT_ID, pos4, pos1)








