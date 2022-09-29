#!/usr/bin/env python

import configparser
import time
import os
import rospy_utils.hriconstants as const
import agents.navigation as nav
import vrep_utils.vrep as vrep
from enum import Enum
from typing import List
from agents.coordinates import Point
from agents.position import Position
from agents.mobilerobot import MobileRobot
from agents.human import Human, follow_fatigue, follow_position, FatigueProfile
from agents.mission import *
from utils.logger import Logger
import rospy_utils.hrirosnode as hriros
import rospy_utils.hriconstants as const
from multiprocessing import Pool


config = configparser.ConfigParser()
config.read('./resources/config.ini')
config.sections()

ENV = config['DEPLOYMENT ENVIRONMENT']['ENV']
LOGGER = Logger('ORCHESTRATOR')

class Operating_Modes(Enum):
	ROBOT_IDLE = 1
	ROBOT_LEAD = 2
	ROBOT_RECH = 3
	ROBOT_CARR = 4
	ROBOT_FOLL = 5
	########################
	ROBOT_SYNCH = 6

class OpChk:
	def __init__(self, t_int: float, t_proc: float, rob: MobileRobot, hum: List[Human], m: Mission, orch_id: str):
		self.LOGGER = Logger('ORCHESTRATOR {}'.format(orch_id))

		# AUTOMATON
		self.LOCATION = "off"
		self.scs = False
		self.fail = False
		self.stop = False
		self.res_sync = False
		# PARAMS
		self.orch_id = orch_id
		self.t_int = t_int
		self.t_proc = t_proc

		self.currOp = Operating_Modes.ROBOT_IDLE
		self.currH = 0

		self.rob = rob
		self.humans = hum
		self.mission = m
		self.rec_stages = 1
		self.stopped_human = False
		if self.mission is not None:
			if self.mission.p[self.currH] == Pattern.HUM_LEADER:
				follow_position(self.humans)
				human_pos = self.humans[self.currH].get_position()
				human_coord = Point(human_pos.x, human_pos.y)
				self.curr_dest = human_coord
			elif self.mission.p[self.currH] == Pattern.HUM_FOLLOWER:
				self.curr_dest = m.dest[self.currH]

		# THRESHOLDS
		self.STOP_DIST = 4.0
		self.RESTART_DIST = 2.0

		self.RECHARGE_TH = 10.0
		self.STOP_RECHARGE = 100.0
		self.FAIL_CHARGE = 1.0

		self.FAIL_FATIGUE = 0.97
		self.STOP_FATIGUE = 0.7
		self.RESUME_FATIGUE = 0.3

		self.SYNCH_TIME = 2
		self.MIN_DISTANCE = 7.0


	def initialize(self):
		self.stop = False
		self.scs = False
		self.fail = False

	def start(self):
		self.initialize()

		while not self.scs and not self.fail and not self.stop:
			self.rob.follow_position()
			self.rob.follow_charge()
			follow_position(self.humans)
			follow_fatigue(self.humans)

			self.LOCATION = 'op'
			time.sleep(self.t_int)
			# t_act >= T_int
			self.check_actions()
			self.LOCATION = 'chk'
			time.sleep(self.t_proc)
			# t_act >= T_proc

		self.LOCATION = 'off'
		return

	def check_actions(self):
		if self.mission is not None:
			self.check_scs()
			if self.scs:
				self.LOGGER.msg('Mission completed with success, stopping orchestrator...')
				return
			self.check_fail()
			if self.fail:
				self.LOGGER.error('Mission failed, stopping orchestrator...')
				return
		if self.currOp == Operating_Modes.ROBOT_IDLE:
			self.check_start()
		elif self.currOp == Operating_Modes.ROBOT_LEAD or self.currOp == Operating_Modes.ROBOT_CARR:
			self.check_r_move()
		elif self.currOp == Operating_Modes.ROBOT_RECH:
			self.check_r_rech()
		elif self.currOp == Operating_Modes.ROBOT_FOLL:
			self.check_h_move()
		elif self.currOp == Operating_Modes.ROBOT_SYNCH:
			self.check_sync()
		if self.mission is not None:
			self.check_service_provided()

	def check_sync(self):
		if self.mission is not None:
			filename = '../scene_logs/syncrisp.log'
			f = open(filename, 'r')
			lines = f.read().splitlines()
			l = lines[-1].split('#')
			help_id = int(l[0])
			f.close()
			LOGGER.info("Checking distance with robot: " + str(help_id))
			if self.get_stop_condition(help_id):
				self.synchronization()
				LOGGER.debug('Killing useless nodes')
				os.system("rosnode kill /helpSub"+str(self.orch_id))
				os.system("rosnode kill /checkDist"+str(self.orch_id))
				LOGGER.info('Subscribing to help topic')
				node = 'helpSub.py'
				pool = Pool()
				pool.starmap(hriros.rosrun_nodes, [(node, [self.orch_id])])
				self.currOp = Operating_Modes.ROBOT_IDLE
		else:
			if self.get_stop_condition(0):
				self.LOGGER.info('Arrived at destination, stopping action.')
				self.synchronization()
				self.pass_mission() 				# pass the mission to the helping robot
				os.system("rosnode kill /'helpSub"+str(self.orch_id))
				self.LOGGER.info('Subscribing to distance data')
				node = 'checkDist.py'
				pool = Pool()
				pool.starmap(hriros.rosrun_nodes, [(node, [self.orch_id])])
				self.LOGGER.info('Subscribing to synchronization response data')
				node1 = 'rispSub.py'
				pool = Pool()
				pool.starmap(hriros.rosrun_nodes, [(node1, [self.orch_id])])
				self.currOp = Operating_Modes.ROBOT_IDLE

	def synchronization(self):
		LOGGER.info("Starting synchronization...")
		self.LOCATION = 'o_sync'
		time.sleep(self.SYNCH_TIME)
		LOGGER.info("Synchronization finished")

	def pass_mission(self):
		dest = [Point(20.0, 2.0)]
		alice = Human(0, 10, FatigueProfile.YOUNG_SICK, 1)
		bill = Human(1, 10, FatigueProfile.ELDERLY_HEALTHY, 1)
		carl = Human(2, 10, FatigueProfile.ELDERLY_SICK, 1)
		dora = Human(3, 10, FatigueProfile.YOUNG_HEALTHY, 1)
		unique_humans = [alice, bill, carl, dora]
		humans = [alice]
		patterns = []
		f = open('mission.txt', 'r')
		lines = f.readlines()
		for line in lines:
			if line.replace('\n', '') == 'LEADER':
				patterns.append(Pattern.HUM_LEADER)
			elif line.replace('\n', '') == 'RECIPIENT':
				patterns.append(Pattern.HUM_RECIPIENT)
			else:
				patterns.append(Pattern.HUM_FOLLOWER)
		self.mission = Mission(patterns, dest)
		if self.mission.p[self.currH] == Pattern.HUM_LEADER:
			follow_position(self.humans)
			human_pos = self.humans[self.currH].get_position()
			human_coord = Point(human_pos.x, human_pos.y)
			self.curr_dest = human_coord
		elif self.mission.p[self.currH] == Pattern.HUM_FOLLOWER:
			self.curr_dest = self.mission.dest[self.currH]
		f.close()

	# Va messo nel get_start_condition?
	def check_wait_chk(self):
		LOGGER.info("Searching for nearby robots ...")
		filename = '../scene_logs/robotDistance.log'
		filename1 = '../scene_logs/syncrisp.log'
		f = open(filename, 'r')
		f1 = open(filename1, 'r')
		node = 'helpPub.py'
		lines = f.read().splitlines()
		l = lines[-1].split(':')
		robot_pos = str(self.rob.get_position())
		robot_pos = robot_pos.replace(" ", "_")
		self.res_sync = False
		n_robots = 3
		num = -n_robots + 1
		while num < 0:
			self.LOGGER.info("Asking help to robot #" + l[num])
			data = l[num] + '#' + robot_pos
			pool = Pool()
			pool.starmap(hriros.rosrun_nodes, [(node, [self.orch_id, data])])
			time.sleep(15)						# problema: dare il tempo al sub di scrivere nel log e all'altro orchestrator di agire
			lines = f1.read().splitlines()		# DA MODIFICARE
			pos_res = l[num] + '#1'
			neg_res = l[num] + '#0'
			if len(lines) != 0:
				if lines[-1] == pos_res:
					self.res_sync = True
					self.LOGGER.info("Robot " + l[num] + " accepted the request")
					return True
				elif lines[-1] == neg_res:
					self.LOGGER.info("Robot " + l[num] + " rejected the request, asking to another robot ...")
			num = num + 1
		self.LOGGER.warn("There are no robots available, going to recharge")
		self.res_sync = None
		return False

	def check_scs(self):
		self.scs = self.mission.get_scs()

	def check_fail(self):
		if vrep.check_connection(const.VREP_CLIENT_ID) != 0:
			self.LOGGER.warn('Lost connection to VRep...')
			self.mission.fail = True

		if self.rob.get_charge() <= self.FAIL_CHARGE:
			self.LOGGER.warn('Mission failing due to low charge...')
			self.mission.fail = True

		if self.humans[self.currH].get_fatigue() >= self.FAIL_FATIGUE:
			self.LOGGER.warn('Mission failing due to high fatigue...')
			self.mission.fail = True

		self.fail = self.mission.fail

	def record_hum_served(self):
		filename = '../scene_logs/humansServed.log'
		f = open(filename, 'a')
		ts = vrep.get_sim_time(const.VREP_CLIENT_ID)
		new_line = '{:.2f}:human{}served\n'.format(ts, self.humans[self.currH].hum_id)
		f.write(new_line)
		f.close()

	def check_service_provided(self):
		human_served = False
		# If human is a follower, service is provided when
		# both human and robot are close to the destination
		if self.mission.p[self.currH] == Pattern.HUM_FOLLOWER:
			dest = self.mission.dest[self.currH]
			position = self.rob.get_position()
			# position = self.humans[self.currH].get_position()
			pos = Point(position.x, position.y)
			human_robot_dist = self.get_human_robot_dist()
			_min_dist = 2.0
			self.LOGGER.debug('dist to dest: {}'.format(pos.distance_from(dest)))
			if position is not None and pos.distance_from(dest) <= _min_dist and human_robot_dist <= _min_dist:
				self.LOGGER.info('HUMAN ' + str(self.currH) + ' SERVED.')
				human_served = True
		# If human is leading, service is provided when the user decides it is
		elif self.mission.p[self.currH] == Pattern.HUM_LEADER or (self.mission.p[self.currH] == Pattern.HUM_RECIPIENT and self.rec_stages == 2):
			filename = '../scene_logs/humansServed.log'
			f = open(filename, 'r+')
			lines = f.read().splitlines()
			svd_lines = list(filter(lambda l: l.__contains__('served'), lines))
			#sought_line = 'human'+ str(self.humans[self.currH].hum_id) + 'served'
			#if any([l.__contains__(sought_line) for l in lines]):
			if len(svd_lines) == self.currH+1:
				self.LOGGER.info('HUMAN ' + str(self.currH) + ' SERVED.')
				human_served = True
			f.close()
		# In any case, if current service has been completed,
		# robot stops and human index increases, and robot goes back
		# to idle if the human that was just served was not the last one
		if human_served:
			self.record_hum_served()
			self.stop = True
			self.mission.set_served(self.currH)
			self.currH+=1
			self.rec_stages = 1
			if self.currH < len(self.humans):
				self.currOp = Operating_Modes.ROBOT_IDLE

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

	def get_rob_rob_distance(self, n: int):
		n_robots = 3
		f = open("../scene_logs/robotDistance.log", "r")
		l = f.read().splitlines()
		s = l[-1].split(':')
		check = n*2 + n_robots + 1
		distance = s[check]
		f.close()
		LOGGER.info("Distance: " + str(distance))
		if float(distance) >= self.MIN_DISTANCE:
			return False
		else:
			return True

	def plan_trajectory(self):
		if ENV == 'S':
			print("Destination: " + str(self.curr_dest))
			traj = nav.plan_traj(self.rob.get_position(), self.curr_dest, nav.init_walls())
			str_traj = ''
			for point in traj:
				str_traj += str(point.x) + ',' + str(point.y)
				if not traj.index(point)==len(traj)-1:
					str_traj += '#'
			if len(traj) > 0:
				vrep.set_trajectory(const.VREP_CLIENT_ID, str_traj, self.orch_id)
		else:
			if self.mission.p[self.currH] == Pattern.HUM_FOLLOWER or (self.mission.p[self.currH] == Pattern.HUM_RECIPIENT and self.rec_stages == 1):
				pass
			else:
				self.rob.start_moving(self.rob.max_speed, Point(self.curr_dest.x, self.curr_dest.y))

	def check_start(self):
		# If battery charge is getting low,
		# robot switches to synchronization mode or to charging mode
		if self.mission is not None:
			if self.rob.get_charge() < self.RECHARGE_TH and not self.res_sync and self.check_wait_chk():
				self.currOp = Operating_Modes.ROBOT_SYNCH
				self.LOCATION = 'wait_chk'
			elif self.rob.get_charge() < self.RECHARGE_TH and (self.res_sync is True or self.res_sync is None):
				self.LOGGER.warn('Low robot charge, initiating recharge procedure...')
				self.stop = True
				self.currOp = Operating_Modes.ROBOT_LEAD
				self.curr_dest = const.VREP_RECH_STATION
				self.plan_trajectory()
			# otherwise the start condition depends on the pattern
			else:
				start = self.get_start_condition(self.mission.p[self.currH])
				if start:
					self.LOGGER.info('Starting action...')
					self.set_op_params(self.mission.p[self.currH])
					self.plan_trajectory()
					self.stop = True
		else:
			LOGGER.debug("Checking start condition ...")
			self.res_sync = self.get_start_condition(0)
			if self.res_sync:
				self.LOGGER.info('Entered in R_sync')
				self.currOp = Operating_Modes.ROBOT_SYNCH
				self.LOCATION = 'move_chk'
				f = open('../scene_logs/synchelp.log', 'r+')
				lines = f.read().splitlines()
				mom = lines[-1].split('#')
				robot_pos = mom[1].split("_")
				pos = robot_pos[1]+"#"+robot_pos[3]+"#"+robot_pos[5]+"#"+robot_pos[7]+"#"+robot_pos[9]+"#"+robot_pos[11]
				last_pos = Position.parse_position(pos)
				dest = Point(last_pos.x, last_pos.y)
				self.curr_dest = dest
				LOGGER.debug('Moving to position: ' + str(dest))
				self.plan_trajectory()
				if ENV == 'S':
					self.rob.start_moving(self.rob.max_speed)
				else:
					self.rob.start_moving(self.rob.max_speed, Point(self.curr_dest.x, self.curr_dest.y))
		return

	def get_start_condition(self, p: int):
		if self.mission is not None:
			human_robot_dist = self.get_human_robot_dist()
			if not self.res_sync:
				battery_charge_sufficient = self.rob.get_charge() >= self.RECHARGE_TH
			else:
				battery_charge_sufficient = True
			human_fatigue_low = self.humans[self.currH].get_fatigue() <= self.RESUME_FATIGUE

			if not battery_charge_sufficient:
				self.LOGGER.info('Robot charge too low to start action.')
			if human_fatigue_low and self.stopped_human:
				self.stopped_human = False
				self.LOGGER.info('Human sufficiently rested.')
				self.mission.publish_status('DFTG#' + str(self.currH+1))

			self.LOGGER.info('Human-Robot distance: ({:.2f})'.format(human_robot_dist))
			# If human is a follower, the action can start if the battery charge is sufficient,
			# if human fatigue is low, and if robot and human are close to each other
			if p == Pattern.HUM_FOLLOWER:
				return battery_charge_sufficient and ((self.stopped_human and human_fatigue_low) or not self.stopped_human) and human_robot_dist < self.RESTART_DIST
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
				self.LOGGER.info('{}'.format(self.rec_stages))
				if self.rec_stages == 1:
					return battery_charge_sufficient
				else:
					robot_pos = self.rob.get_position()
					robot_pt = Point(robot_pos.x, robot_pos.y)
					return human_robot_dist >= self.RESTART_DIST or robot_pt.distance_from(self.curr_dest) > 2.0
			else:
				return False
		else:
			f = open('../scene_logs/synchelp.log', 'r+')
			lines = f.read().splitlines()
			if len(lines) != 0:
				mom = lines[-1].split('#')
				help_id = mom[0]
				if help_id == str(self.orch_id):
					node = 'rispPub.py'
					LOGGER.info('Publishing to risp topic...')
					data = str(self.orch_id) + '#1'
					pool = Pool()
					pool.starmap(hriros.rosrun_nodes, [(node, [self.orch_id, data])])
					f.close()
					return True
				else:
					f.close()
					return False

	def set_op_params(self, p: int):
		# Human follower -> destination = prescribed destination
		if p == Pattern.HUM_FOLLOWER:
			self.currOp = Operating_Modes.ROBOT_CARR
			self.curr_dest = self.mission.dest[self.currH]
		# Human leader -> destination = current human position
		elif p == Pattern.HUM_LEADER:
			self.currOp = Operating_Modes.ROBOT_FOLL
			curr_human_pos = self.humans[self.currH].get_position()
			if ENV == 'S':
				self.curr_dest = Point(curr_human_pos.x, curr_human_pos.y)
			else:
				self.curr_dest = Point(curr_human_pos.x-const.REAL_X_OFFSET, curr_human_pos.y-const.REAL_Y_OFFSET)
		# Human recipient -> (stage1) dest = prescribed dest, (stage2) dest = current human position
		elif p == Pattern.HUM_RECIPIENT:
			if self.rec_stages == 1:
				self.currOp = Operating_Modes.ROBOT_CARR
				self.curr_dest = self.mission.dest[self.currH]
			else:
				self.currOp = Operating_Modes.ROBOT_FOLL
				curr_human_pos = self.humans[self.currH].get_position()
				if ENV == 'S':
					self.curr_dest = Point(curr_human_pos.x, curr_human_pos.y)
				else:
					self.curr_dest = Point(curr_human_pos.x-const.REAL_X_OFFSET, curr_human_pos.y-const.REAL_Y_OFFSET)
				self.LOGGER.info('HERE {}'.format(self.curr_dest))
		return

	def get_stop_condition(self, p: int):
		if self.mission is not None and self.currOp != Operating_Modes.ROBOT_SYNCH:
			human_robot_dist = self.get_human_robot_dist()
			human_fatigue_high = self.humans[self.currH].get_fatigue() >= self.STOP_FATIGUE
			self.LOGGER.debug('Res_sync: ' + str(self.res_sync))
			if self.res_sync is False:
				battery_charge_insufficient = self.rob.get_charge() <= self.RECHARGE_TH
			else:
				battery_charge_insufficient = self.rob.get_charge() <= self.FAIL_CHARGE
			if battery_charge_insufficient:
				self.LOGGER.warn('Stopping action due to low battery...')
			if human_fatigue_high:
				self.mission.publish_status('FTG#' + str(self.currH+1))
				self.stopped_human = True
				self.LOGGER.warn('Stopping action due to excessive fatigue...')
			# Human Follower -> action must stop if battery charge is too low, fatigue is too high
			# or human and robot are excessively far from each other
			if p == Pattern.HUM_FOLLOWER:
				ris = human_robot_dist > self.STOP_DIST
				#LOGGER.info("Distanza con l'umano: " + str(ris))
				return battery_charge_insufficient or human_fatigue_high or human_robot_dist > self.STOP_DIST
			# Human Leader -> action must stop if current destination has been reached or robot is already sufficiently close to human
			elif p == Pattern.HUM_LEADER:
				robot_pos = self.rob.get_position()
				robot_pt = Point(robot_pos.x, robot_pos.y)
				hum_pos = self.humans[self.currH].get_position()
				hum_pt = Point(hum_pos.x, hum_pos.y)
				return human_robot_dist < self.RESTART_DIST
				# return robot_pt.distance_from(self.curr_dest) <= 1.0 or human_robot_dist < self.RESTART_DIST or hum_pt.distance_from(self.curr_dest) > 4.0
			# Human Recipient -> action must stop if battery charge is too low or destination has already been reached
			elif p == Pattern.HUM_RECIPIENT:
				robot_pos = self.rob.get_position()
				robot_pt = Point(robot_pos.x, robot_pos.y)
				if self.rec_stages == 1:
					if robot_pt.distance_from(self.curr_dest) <= self.RESTART_DIST:
						self.currOp = Operating_Modes.ROBOT_IDLE
						self.rec_stages = 2
					return battery_charge_insufficient or robot_pt.distance_from(self.curr_dest) <= self.RESTART_DIST
				else:
					return robot_pt.distance_from(self.curr_dest) <= self.RESTART_DIST or human_robot_dist < self.RESTART_DIST
			else:
				return False

		elif self.mission is not None and self.currOp == Operating_Modes.ROBOT_SYNCH:
			if self.get_rob_rob_distance(p):
				return True

		elif self.mission is None:
			if self.get_rob_rob_distance(int(self.orch_id)):
				self.rob.stop_moving()
				return True
		return False

	def check_r_move(self):
		self.stop = self.get_stop_condition(self.mission.p[self.currH])
		if self.stop:
			self.LOGGER.info('Stopping action.')
			#if self.mission.p[self.currH] == Pattern.HUM_RECIPIENT and self.rec_stages==1:
			#self.rec_stages = 2

	def check_h_move(self):
		self.stop = self.get_stop_condition(self.mission.p[self.currH])
		if self.stop:
			self.LOGGER.info('Stopping action.')
			# self.currOp = Operating_Modes.ROBOT_IDLE
			# self.rob.stop_moving()
		else:
			human_pos = self.humans[self.currH].get_position()
			hum_pt = Point(human_pos.x-const.REAL_X_OFFSET, human_pos.y-const.REAL_Y_OFFSET)
			# Point(curr_human_pos.x-const.REAL_X_OFFSET, curr_human_pos.y-const.REAL_Y_OFFSET)
			self.LOGGER.info('{}'.format(hum_pt.distance_from(self.curr_dest)))
			if hum_pt.distance_from(self.curr_dest) > 0.5:
				self.LOGGER.info('Restarting')
				self.curr_dest = hum_pt
				self.plan_trajectory()

	def check_r_rech(self):
		robot_pos = self.rob.get_position()
		robot_pt = Point(robot_pos.x, robot_pos.y)
		# While the robot is in recharging mode, motion must stop if
		# charging dock has already been reached
		n = robot_pt.distance_from(self.curr_dest)
		if robot_pt.distance_from(self.curr_dest) < 0.5 and self.rob.curr_speed > 0.0:
			self.LOGGER.info('Reached recharge station.')
			self.rob.stop_moving()

		if self.rob.get_charge() >= self.STOP_RECHARGE:
			self.LOGGER.info('Battery recharged. Resuming operations...')
			self.currOp = Operating_Modes.ROBOT_IDLE

class Orchestrator:
	def __init__(self, opchk: OpChk):
		self.opchk = opchk
		self.LOCATION = 'o_init'
		self.rob = opchk.rob
		self.humans = opchk.humans
		self.mission = opchk.mission
		self.LOGGER = Logger('ORCHESTRATOR {}'.format(opchk.orch_id))

	def run_mission(self):
		if self.opchk.mission is None:
			node1 = 'helpSub.py'
			pool = Pool()
			pool.starmap(hriros.rosrun_nodes, [(node1, [self.opchk.orch_id])])
			LOGGER.info('Subscribing to help topic...')
		else:
			self.LOGGER.info('Starting mission...')
			node = 'checkDist.py'
			self.LOGGER.info('Subscribing to distance data...')
			pool = Pool()
			pool.starmap(hriros.rosrun_nodes, [(node, [self.opchk.orch_id])])
			node1 = 'rispSub.py'
			self.LOGGER.info('Subscribing to synchronization response data...')
			pool = Pool()
			pool.starmap(hriros.rosrun_nodes, [(node1, [self.opchk.orch_id])])
		while not self.LOCATION == 'o_fail_1' and not self.LOCATION == 'o_fail_2' and not self.LOCATION == 'o_scs':
			self.opchk.start()
			self.LOCATION = 'idle'
			if self.opchk.stop:
				self.LOCATION = 'r_start'
				if ENV == 'S':
					self.rob.start_moving(self.rob.max_speed)
				else:
					#self.rob.start_moving(self.rob.max_speed, Point(0.303095638752, 0.335622757673))
					self.rob.start_moving(self.rob.max_speed, Point(self.opchk.curr_dest.x, self.opchk.curr_dest.y))
				self.LOCATION = 'h_start'
				# no effect, we cannot control the human
				self.LOCATION = 'x_move'
				self.opchk.start()
				self.rob.stop_moving()
				self.LOCATION = 'stopping'
				rob_to_rech = self.opchk.currOp == Operating_Modes.ROBOT_LEAD
				rob_leading = self.opchk.currOp == Operating_Modes.ROBOT_CARR
				try:
					hum_recipient = self.mission.p[self.opchk.currH] == Pattern.HUM_RECIPIENT
				except IndexError:
					hum_recipient = False
				if self.opchk.stop and not (rob_to_rech or (rob_leading and hum_recipient)):
					# stopHuman or not, we cannot control the human
					self.opchk.currOp = Operating_Modes.ROBOT_IDLE
					self.LOCATION = 'o_init'
				elif self.opchk.stop and (rob_to_rech or (rob_leading and hum_recipient)):
					self.LOCATION = 'to_2nd_task'
					if rob_to_rech:
						self.rob.start_moving(self.rob.max_speed)
						self.LOCATION = 'starting_2'
						self.opchk.currOp = Operating_Modes.ROBOT_RECH
						self.LOCATION = 'r_rech'
						self.opchk.start()
						self.LOCATION = 'stopping_2'
						if self.opchk.stop:
							self.opchk.currOp = Operating_Modes.ROBOT_IDLE
							self.LOCATION = 'o_init'
					elif rob_leading and hum_recipient:
						# cannot control human sync
						self.LOCATION = 'delivering'
						self.opchk.start()
						self.LOCATION = 'o_init'
				elif self.opchk.scs:
					self.LOCATION = 'o_scs'
				elif self.opchk.fail:
					self.LOCATION = 'o_fail_2'
			elif self.opchk.scs:
				self.rob.stop_moving()
				self.LOCATION = 'o_scs'
			elif self.opchk.fail:
				self.rob.stop_moving()
				self.LOCATION = 'o_fail_1'

		self.rob.set_sim_running(0)
		return
