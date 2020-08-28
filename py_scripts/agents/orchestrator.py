#!/usr/bin/env python
import time
import rospy_utils.hriconstants as const
import agents.navigation as nav
import rospy_utils.hrirosnode as hriros
import vrep_utils.vrep as vrep
from enum import Enum
from typing import List
from multiprocessing import Pool
from agents.coordinates import Point
from agents.mobilerobot import MobileRobot
from agents.human import Human
from agents.mission import *

class Operating_Modes(Enum):
	ROBOT_IDLE = 1
	ROBOT_LEAD = 2
	ROBOT_RECH = 3
	ROBOT_CARR = 4
	ROBOT_FOLL = 5
	
class Orchestrator:
	def __init__(self, t_int: int, t_proc: int, rob: MobileRobot, hum: List[Human], m: Mission):
		# PARAMS
		self.t_int = t_int
		self.t_proc = t_proc

		self.currOp = Operating_Modes.ROBOT_IDLE
		self.currH = 0
		
		self.rob = rob
		self.humans = hum
		self.mission = m
		self.rec_stages = 1
		if self.mission.p[self.currH] == Pattern.HUM_LEADER:
			human_pos = self.humans[self.currH].get_position()
			human_coord = Point(human_pos.x, human_pos.y)
			self.curr_dest = human_coord
		else:
			self.curr_dest = m.dest[self.currH]

		# THRESHOLDS
		self.STOP_DIST = 4.0
		self.RESTART_DIST = 2.0

		self.RECHARGE_TH = 40.0
		self.STOP_RECHARGE = 50.0
		self.FAIL_CHARGE = 1.0
		
		self.FAIL_FATIGUE = 0.97
		self.STOP_FATIGUE = 0.85
		self.RESUME_FATIGUE = 0.3;

	def run_mission(self):
		print('Starting mission...')
		while not self.check_scs() and not self.check_fail():
			#print('Checking actions...')
			self.check_actions()
			time.sleep(self.t_int)

		self.rob.set_sim_running(0)
		return

	def check_actions(self):
		if self.check_scs():
			return 

		if self.check_fail():
			return

		if self.currOp == Operating_Modes.ROBOT_IDLE:
			self.check_start()
		elif self.currOp == Operating_Modes.ROBOT_LEAD or self.currOp == Operating_Modes.ROBOT_CARR:
			self.check_r_move()
		elif self.currOp == Operating_Modes.ROBOT_RECH:
			self.check_r_rech()
		elif self.currOp == Operating_Modes.ROBOT_FOLL:
			self.check_h_move()

		self.check_service_provided()

	# CHECK IF ALL SERVICES HAVE BEEN PROVIDED, THUS MISSION HAS BEEN SUCCESSFULLY COMPLETED 
	def check_scs(self):
		return self.mission.get_scs()

	# CHECK IF MISSION HAS FAILED DUE TO BATTERY CHARGE TOO LOW, OR FATIGUE TOO HIGH
	def check_fail(self):
		if vrep.check_connection(const.VREP_CLIENT_ID)==-1:
			self.mission.fail = True

		if self.rob.get_charge()<=self.FAIL_CHARGE:
			self.mission.fail = True
		
		if self.humans[self.currH].get_fatigue()>=self.FAIL_FATIGUE:
			self.mission.fail = True
			
		return self.mission.fail

	# CHECK IF CURRENT SERVICE HAS BEEN PROVIDED, THUS THE MISSION CAN MOVE ON
	def check_service_provided(self):
		human_served = False
		# If human is a follower, service is provided when 
		# both human and robot are close to the destination
		if self.humans[self.currH].ptrn == Pattern.HUM_FOLLOWER:
			dest = self.mission.dest[self.currH]
			position = self.humans[self.currH].get_position()
			pos = Point(position.x, position.y)
			human_robot_dist = self.get_human_robot_dist()
			_min_dist = 1.5
			if position is not None and pos.distance_from(dest) <= _min_dist and human_robot_dist <= _min_dist:
				print('HUMAN ' + str(self.currH) + ' SERVED.')
				human_served = True
		# If human is leading, service is provided when 
		# human player says so
		elif self.humans[self.currH].ptrn == Pattern.HUM_LEADER	or (self.humans[self.currH].ptrn == Pattern.HUM_RECIPIENT and self.rec_stages == 2):
			filename = '../scene_logs/humansServed.log'
			f = open(filename, 'r')
			lines = f.read().splitlines()
			for line in lines:
				if line == 'human'+ str(self.humans[self.currH].hum_id) + 'served':
					print('HUMAN ' + str(self.currH) + ' SERVED.')
					human_served = True
					break
			
		# In any case, if current service has been completed,
		# robot stops and human index increases, and robot goes back
		# to idle if the human that was just served was not the last one
		if human_served:
			self.rob.stop_moving()
			self.mission.set_served(self.currH)
			self.currH+=1
			self.rec_stages = 1
			if self.currH < len(self.humans):
				self.currOp = Operating_Modes.ROBOT_IDLE
				
	# METHODS TO CHECK WHETHER ACTION CAN START
	def get_human_robot_dist(self):
		human_pos = self.humans[self.currH].get_position()
		robot_pos = self.rob.get_position()
		if human_pos is not None and robot_pos is not None:   
			human_coord = Point(human_pos.x, human_pos.y)
			robot_coord = Point(robot_pos.x, robot_pos.y) 
			human_robot_dist = robot_coord.distance_from(human_coord) 
			return human_robot_dist
		else:
			return 1000

	# PLAN (AND PUBLISH) TRAJECTORY FROM CURRENT POS TO CURRENT DESTINATION
	def plan_trajectory(self):
		traj = nav.plan_traj(self.rob.get_position(), self.curr_dest, nav.init_walls())
		str_traj = ''
		for point in traj:
			str_traj += str(point.x) + ',' + str(point.y)
			if not traj.index(point)==len(traj)-1:
				str_traj += '#'
		if len(traj)>0:
			node = 'robTrajPub.py'
			pool = Pool()
			pool.starmap(hriros.rosrun_nodes, [(node, [str_traj])])

	# CHECK WHETHER CURRENT ACTION SHOULD START
	def check_start(self):
		# If battery charge is getting low, 
		# robot switches to charging mode
		if self.rob.get_charge() < self.RECHARGE_TH:
			print('ROBOT CHARGE TOO LOW')
			self.rob.stop_moving()
			self.currOp = Operating_Modes.ROBOT_RECH
			self.curr_dest = const.VREP_RECH_STATION
			self.plan_trajectory()
			self.rob.start_moving(self.rob.max_speed)
		# otherwise the start condition depends on the pattern
		else:
			start = self.get_start_condition(self.humans[self.currH].ptrn)
			if start:
				print('Action can start, setting parameters...')
				self.set_op_params(self.humans[self.currH].ptrn)
				self.rob.start_moving(self.rob.max_speed)
				self.plan_trajectory()
		return
	
	# GET START CONDITION BASED ON CURRENT PATTERN
	def get_start_condition(self, p: int):	
		human_robot_dist = self.get_human_robot_dist()  
		battery_charge_sufficient = self.rob.get_charge() >= self.RECHARGE_TH
		human_fatigue_low = self.humans[self.currH].get_fatigue() <= self.STOP_FATIGUE

		if not battery_charge_sufficient:
			print('ROBOT CHARGE TOO LOW')
		if not human_fatigue_low:
			print('HUMAN FATIGUE TOO HIGH')
		print('HUMAN-ROBOT DISTANCE: ' + str(human_robot_dist))
		# If human is a follower, the action can start if the battery charge is sufficient,
		# if human fatigue is low, and if robot and human are close to each other
		if p == Pattern.HUM_FOLLOWER:
			return battery_charge_sufficient and human_fatigue_low and human_robot_dist < self.RESTART_DIST
		# If human is a leader, the action can start 
		# if the robot is distant from current destination (human position)
		elif p == Pattern.HUM_LEADER:
			robot_pos = self.rob.get_position()
			if robot_pos is not None:
				robot_pt = Point(robot_pos.x, robot_pos.y)
				return human_robot_dist >= self.RESTART_DIST or robot_pt.distance_from(self.curr_dest) > 2.0
			else:
				return human_robot_dist >= self.RESTART_DIST
		# If human is a recipient, the action can start if
		# battery charge is sufficient and human fatigue is low
		elif p == Pattern.HUM_RECIPIENT:
			if self.rec_stages == 1:
				return battery_charge_sufficient 
			else:	
				robot_pos = self.rob.get_position()
				robot_pt = Point(robot_pos.x, robot_pos.y)
				return human_robot_dist >= self.RESTART_DIST or robot_pt.distance_from(self.curr_dest) > 2.0
		else:
			return False

	def set_op_params(self, p: int):
		# Human follower -> destination = prescribed destination
		if p == Pattern.HUM_FOLLOWER:
			self.currOp = Operating_Modes.ROBOT_LEAD
			self.curr_dest = self.mission.dest[self.currH]
		# Human leader -> destination = current human position
		elif p == Pattern.HUM_LEADER:
			self.currOp = Operating_Modes.ROBOT_FOLL
			curr_human_pos = self.humans[self.currH].get_position()
			self.curr_dest = Point(curr_human_pos.x, curr_human_pos.y)
		# Human recipient -> (stage1) dest = prescribed dest, (stage2) dest = current human position
		elif p == Pattern.HUM_RECIPIENT:
			self.currOp = Operating_Modes.ROBOT_CARR
			if self.rec_stages == 1:
				self.curr_dest = self.mission.dest[self.currH]
			else:
				curr_human_pos = self.humans[self.currH].get_position()
				self.curr_dest = Point(curr_human_pos.x, curr_human_pos.y)				
		return
 	
	# METHODS TO CHECK WHETHER ACTION HAS TO STOP
	def get_stop_condition(self, p: int):
		human_robot_dist = self.get_human_robot_dist()
		battery_charge_insufficient = self.rob.get_charge() <= self.RECHARGE_TH
		human_fatigue_high = self.humans[self.currH].get_fatigue() >= self.STOP_FATIGUE
		if battery_charge_insufficient:
			print('!!ROBOT BATTERY CHARGE TOO LOW!!')
		if human_fatigue_high:
			print('!!HUMAN FATIGUE TOO HIGH!!')

		# Human Follower -> action must stop if battery charge is too low, fatigue is too high
		# or human and robot are excessively far from each other
		if p == Pattern.HUM_FOLLOWER:
			return battery_charge_insufficient or human_fatigue_high or human_robot_dist > self.STOP_DIST
		# Human Leader -> action must stop if current destination has been reached or robot is already
		# sufficiently close to human
		elif p == Pattern.HUM_LEADER: 
			robot_pos = self.rob.get_position()
			robot_pt = Point(robot_pos.x, robot_pos.y)
			return robot_pt.distance_from(self.curr_dest) <= 1.0 or human_robot_dist < self.RESTART_DIST
		# TODO
		elif p == Pattern.HUM_RECIPIENT: 
			robot_pos = self.rob.get_position()
			robot_pt = Point(robot_pos.x, robot_pos.y)
			if self.rec_stages == 1:
				return battery_charge_insufficient or robot_pt.distance_from(self.curr_dest) <= 1.0
			else:
				return robot_pt.distance_from(self.curr_dest) <= 1.0 or human_robot_dist < self.RESTART_DIST
		else:
			return False
	
	def check_r_move(self):
		stop = self.get_stop_condition(self.humans[self.currH].ptrn)
		if stop:
			print('Action has to stop')
			self.currOp = Operating_Modes.ROBOT_IDLE
			self.rob.stop_moving()
			if self.humans[self.currH].ptrn == Pattern.HUM_RECIPIENT and self.rec_stages==1:
				self.rec_stages = 2

	def check_h_move(self):
		stop = self.get_stop_condition(self.humans[self.currH].ptrn)
		if stop:
			print('Action has to stop')
			self.currOp = Operating_Modes.ROBOT_IDLE
			self.rob.stop_moving()
			human_pos = self.humans[self.currH].get_position()
			human_pt = Point(human_pos.x, human_pos.y)
			self.curr_dest = Point(human_pos.x, human_pos.y)

	def check_r_rech(self):
		robot_pos = self.rob.get_position()
		robot_pt = Point(robot_pos.x, robot_pos.y)
		# While the robot is in recharging mode, motion must stop if 
		# charging dock has already been reached
		if robot_pt.distance_from(self.curr_dest) < 0.5 and self.rob.curr_speed > 0.0:
			self.rob.stop_moving()			
		
		if self.rob.get_charge() >= self.STOP_RECHARGE:
			self.currOp = Operating_Modes.ROBOT_IDLE

