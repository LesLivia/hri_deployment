#!/usr/bin/env python
import vrep_utils.lib.vrep.vrep as vrep
import rospy_utils.hriconstants as const
from agents.coordinates import Point
from utils.logger import Logger

LOGGER = Logger("VREP")

def connect(port):
	LOGGER.info('Connecting to VRep...')
	clientID = vrep.simxStart('127.0.0.1',port,True,True,5000,5)
	if clientID != -1:
		LOGGER.info('Connection successfully established.')
	else:
		LOGGER.error('Connection to VRep could not be established.')
	return clientID 

def start_sim(clientID):
	result = vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot)

def stop_sim(clientID):
	result = vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)
	if result != vrep.simx_return_ok:
		LOGGER.info('Simulation stopped.')

def get_sim_time(clientID):
	result = vrep.simxCallScriptFunction(clientID, 'floor', vrep.sim_scripttype_childscript, 'get_time', [], [], '', '', vrep.simx_opmode_blocking)
	return result[2][0]

def check_connection(clientID):
	state = vrep.simxCallScriptFunction(clientID, 'floor', vrep.sim_scripttype_childscript, 'sim_state', [], [], '', '', vrep.simx_opmode_blocking)
	if state[0] != 0:
		LOGGER.warn('Connection lost')
	return state[0]

def set_trajectory(clientID, strTraj: str, id: str):
	LOGGER.debug('Sending new trajectory...')
	vrep.simxCallScriptFunction(clientID, 'MobileRobot'+id, vrep.sim_scripttype_childscript, 'setTraj', [], [], [strTraj], '', vrep.simx_opmode_blocking)

def set_state(clientID, strState: str, id: str):
	vrep.simxCallScriptFunction(clientID, 'MobileRobot'+id, vrep.sim_scripttype_childscript, 'setRobotState', [], [], [strState], '', vrep.simx_opmode_blocking)

def draw_point(clientID, pos: Point):
	vrep.simxCallScriptFunction(clientID, 'floor', vrep.sim_scripttype_childscript, 'draw_point', [], [pos.x-const.VREP_X_OFFSET, pos.y-const.VREP_Y_OFFSET], '', '', vrep.simx_opmode_blocking)

def draw_line(clientID, pos1: Point, pos2: Point):
	vrep.simxCallScriptFunction(clientID, 'floor', vrep.sim_scripttype_childscript, 'draw_line', [], [pos1.x-const.VREP_X_OFFSET, pos1.y-const.VREP_Y_OFFSET, pos2.x-const.VREP_X_OFFSET, pos2.y-const.VREP_Y_OFFSET], '', '', vrep.simx_opmode_blocking)

def clear_lines(clientID):
	vrep.simxCallScriptFunction(clientID, 'floor', vrep.sim_scripttype_childscript, 'clear_line', [], [], '', '', vrep.simx_opmode_blocking)

def draw_rectangle(clientID, pos1: Point, pos2: Point, pos3: Point, pos4: Point):
	clear_lines(const.VREP_CLIENT_ID)
	draw_line(const.VREP_CLIENT_ID, pos1, pos2)
	draw_line(const.VREP_CLIENT_ID, pos2, pos3)
	draw_line(const.VREP_CLIENT_ID, pos3, pos4)
	draw_line(const.VREP_CLIENT_ID, pos4, pos1)

# FUNCTIONS FOR AUTOMATED HUMAN CONTROL
def start_human(clientID, humID: int):
	LOGGER.debug('Instructing human {} to walk...'.format(humID))	
	vrep.simxCallScriptFunction(clientID, 'Human{}'.format(humID), vrep.sim_scripttype_childscript, 'start_walking', [], [], [], '', vrep.simx_opmode_blocking)

def stop_human(clientID, humID: int):
	LOGGER.debug('Instructing human {} to stop...'.format(humID))	
	vrep.simxCallScriptFunction(clientID, 'Human{}'.format(humID), vrep.sim_scripttype_childscript, 'stop_walking', [], [], [], '', vrep.simx_opmode_blocking)

def sit(clientID, humID: int):
	LOGGER.debug('Instructing human {} to sit...'.format(humID))	
	vrep.simxCallScriptFunction(clientID, 'Human{}'.format(humID), vrep.sim_scripttype_childscript, 'sit_cmd', [], [], [], '', vrep.simx_opmode_blocking)

def stand(clientID, humID: int):
	LOGGER.debug('Instructing human {} to stand up...'.format(humID))	
	vrep.simxCallScriptFunction(clientID, 'Human{}'.format(humID), vrep.sim_scripttype_childscript, 'stand_cmd', [], [], [], '', vrep.simx_opmode_blocking)

def run_cmd(clientID, humID: int):
	LOGGER.debug('Instructing human {} to sit...'.format(humID))		
	vrep.simxCallScriptFunction(clientID, 'Human{}'.format(humID), vrep.sim_scripttype_childscript, 'run_cmd', [], [], [], '', vrep.simx_opmode_blocking)

def served_cmd(clientID, humID: int):
	LOGGER.debug('Marking human {} as served...'.format(humID))	
	vrep.simxCallScriptFunction(clientID, 'Human{}'.format(humID), vrep.sim_scripttype_childscript, 'served_cmd', [], [], [], '', vrep.simx_opmode_blocking)

def set_hum_trajectory(clientID, humID:int, strTraj: str):
	LOGGER.debug('Sending trajectory to human {}...'.format(humID))	
	vrep.simxCallScriptFunction(clientID, 'Human{}'.format(humID), vrep.sim_scripttype_childscript, 'setTraj', [], [], [strTraj], '', vrep.simx_opmode_blocking)

def set_hum_state(clientID, humID:int, state: int):
	vrep.simxCallScriptFunction(clientID, 'Human{}'.format(humID), vrep.sim_scripttype_childscript, 'set_state_cmd', [state], [], [], '', vrep.simx_opmode_blocking)

def reset_hum(clientID, humID:int, pos:Point):
	vrep.simxCallScriptFunction(clientID, 'Human{}'.format(humID), vrep.sim_scripttype_childscript, 'reset_cmd', [], [pos.x-const.VREP_X_OFFSET, pos.y-const.VREP_Y_OFFSET], [], '', vrep.simx_opmode_blocking)
	vrep.simxCallScriptFunction(clientID, 'wearable_dev'.format(humID), vrep.sim_scripttype_childscript, 'reset_ftg', [], [], [], '', vrep.simx_opmode_blocking)
	
