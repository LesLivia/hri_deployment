#!/usr/bin/env python
import os
import time
import rospy_utils.hrirosnode as hriros
import rospy_utils.hriconstants as const
import agents.navigation as nav
import vrep_utils.vrep as vrep                         #importo con un altro nome piu comodo
from multiprocessing import Pool
from agents.position import Position 				# posso importare tutto ma cosi è piu comodo quando chiamo le funzioni
from agents.coordinates import Point
from datetime import datetime

from typing import List
from agents.human import *
from threading import Thread

class MobileRobot:
	def __init__(self, rob_id, max_speed, max_accel):	# max_accel anche se poi non la usiamo la lasciamo come parametro. la velocita è infatti gestita internamente a vrep
		self.rob_id = rob_id
		self.max_speed = max_speed
		self.max_accel = max_accel
		self.curr_speed = 0.0
		self.sim_running = 0

		self.free=True
		self.stop_recharge=95
		self.recharge_th=10
		self.execution_finished=False


	def set_position(self, position: Position):     # ha appena creato un nuovo attributo non specificato sopra
		self.position = position					# attributo non della classe, ma in piu dell oggetto. inatti si crea solo quando chiamo la funzione

	def get_position(self):
		return self.position						# se mi da AttributeError devo aspettare piu tempo con un time.spleep() per far arrivare i dati

	def set_charge(self, charge: float):          # qua lo stesso crea un nuovo attributo
		self.charge = charge

	def get_charge(self):
		return self.charge

	def set_sim_running(self, run):			# run è un int
		self.sim_running = run

	def is_sim_running(self):
		return self.sim_running




	def follow_charge(self):				#giustamente in una simulazione, la batteria deve seguire una funzione gia stabilita
		filename = '../scene_logs/robotBattery.log'
		_cached_stamp = 0
		while self.is_sim_running():
			# when a new line is written to log file,
			# robot charge attribute is updated as well
			# -> this needs to be continuously running in a
			# parallel thread
			stamp = os.stat(filename).st_mtime
			if stamp != _cached_stamp:
				f = open(filename, 'r')
				lines = f.read().splitlines()

				#last_line = lines[-1]			# ogni tanto mi da errore IndexError, devo fare aspettare piu il programma, ma fuori, non in questa funzione

				i=-1
				last_line=lines[i]
				while int(last_line.split(':')[2]) != self.rob_id:
					i=i-1
					last_line=lines[i]



				new_charge = float(last_line.split(':')[1])

				now = datetime.now()
				current_time = now.strftime("%H:%M:%S")
				# print("Current Time =", current_time)

				self.set_charge(new_charge)
				_cached_stamp = stamp
	



	# FORSE STOP_MOVING NON È DA PORTARE FUORI DALLA CLASSE


	def stop_moving(self):
		node = 'robStatusPub.py'
		data = '0#0.0'
		#vrep.set_state(const.VREP_CLIENT_ID, data)

		# PROVA

		rob_id=self.rob_id
		filename='MobileRobot'+str(rob_id)

		vrep.set_state(const.VREP_CLIENT_ID, data,filename)


		# both motors speed is set to 0, so that the robot stops moving
		pool = Pool()
		pool.starmap(hriros.rosrun_nodes, [(node, [data])])
		self.curr_speed = 0.0
		print('Robot',self.rob_id,' stopping...')


#######################################################################################################################



def start_reading_data_robs( robs: List[MobileRobot]):	# è normale che i robs li usi solo all ultimo per il sim_running??????????
		# clear robot position log file
		f = open('../scene_logs/robotPosition.log', 'r+')
		f.truncate(0)
		f.close()
		# launch ROS node that subscribes to robot GPS data
		node = 'robSensorsSub.py'

		pool = Pool()
		pool.starmap(hriros.rosrun_nodes, [(node, [''])])

		# clear robot position log file
		f = open('../scene_logs/robotBattery.log', 'r+')
		f.truncate(0)
		f.close()
		# launch ROS node that subscribes to robot GPS data
		node = 'robBatterySub.py'

		pool = Pool()
		pool.starmap(hriros.rosrun_nodes, [(node, [''])])

		for rob in robs:
			rob.set_sim_running(1)









def follow_position_rob(rob: MobileRobot):
		filename = '../scene_logs/robotPosition.log'
		_cached_stamp = 0
		while rob.is_sim_running():
			try:
				# when a new line is written to log file,
				# robot position attribute is updated as well
				# -> this needs to be continuously running in a
				# parallel thread
				stamp = os.stat(filename).st_mtime						# serve per la comunicazione dei nodi ros
				if stamp != _cached_stamp:
					f = open(filename, 'r')
					lines = f.read().splitlines()

					i=-1
					last_line=lines[i]
					while int(last_line.split(':')[2]) != rob.rob_id:
						i=i-1
						last_line=lines[i]

					newPos = Position.parse_position(last_line.split(':')[1])

					# print('ROBOT',rob.rob_id,'last line è',last_line)
					# print('NEW POS',newPos)


					# VRep layout origin is different from the
					# one in the Uppaal model: translation is necessary
					newPos.x += const.VREP_X_OFFSET		# se c è l if sopra, da qui fino a exept dovrebbero essere indentate
					newPos.y += const.VREP_Y_OFFSET

					rob.set_position(newPos)
					_cached_stamp = stamp

			except (KeyboardInterrupt, SystemExit):
				print('Stopping robot position monitoring...')
				return



def start_moving_rob(rob:MobileRobot, targetSpeed):

		node = 'robStatusPub.py'

		data ='1#'


		print(data)
		if targetSpeed > 0:
		    data = data + str(targetSpeed)
		#vrep.set_state(const.VREP_CLIENT_ID, data)

		# PROVA

		rob_id=rob.rob_id
		filename='MobileRobot'+str(rob_id)

		vrep.set_state(const.VREP_CLIENT_ID, data,filename)



		# requested target speed is published to both robot motors,
		# so that the robot starts moving straight
		pool = Pool()
		pool.starmap(hriros.rosrun_nodes, [(node, [data])])
		rob.curr_speed = targetSpeed


		print('Robot',rob.rob_id,' moving...')

###################################################################################################


def free_robs(robs: List[MobileRobot]):
		free_robs=[]
		for rob in robs:
			if rob.free:
				free_robs.append(rob)
		return free_robs


def nearest(robs: List[MobileRobot],hum: Human):
	if len(robs)==0:
		return None
	human_pos=hum.get_position()
	dist=10000
	for rob in robs:
		rob_pos=rob.get_position()
		rob_coord=Point(rob_pos.x,rob_pos.y)
		new_dist=rob_coord.distance_from(human_pos) # argomento di distance_from puo essere sia point che position. guardare funzione per capire
		if new_dist<dist:
			dist=new_dist
			chosen_rob=rob
	return chosen_rob

def nearest_to_point(robs: List[MobileRobot], point: Point):
	if len(robs)==0:
		return None
	dist=10000
	for rob in robs:
		rob_pos=rob.get_position()
		rob_coord=Point(rob_pos.x,rob_pos.y)
		new_dist=rob_coord.distance_from(point) # argomento di distance_from puo essere sia point che position. guardare funzione per capire
		if new_dist<dist:
			dist=new_dist
			chosen_rob=rob
	return chosen_rob


def choose_rob(robs,hum):		# il piu vicino tra quelli liberi
	return nearest(free_robs(robs),hum)

def rob_piu_carico_tra_i_liberi(robs: List[MobileRobot]):
	return rob_piu_carico(free_robs(robs))

def rob_piu_carico(robs: List[MobileRobot]):
	if len(robs) == 0:
		return None
	rob_scelto=robs[0]
	for rob in robs:
		if rob.get_charge() > rob_scelto.get_charge():
			rob_scelto=rob
	return rob_scelto

def rob_piu_vicino_abbastanza_carico(robs: List[MobileRobot],hum: Human, carica_minima_accettabile: int):
	return nearest(robs_abbastanza_carichi(free_robs(robs), carica_minima_accettabile) , hum)



def robs_abbastanza_carichi(robs: List[MobileRobot] , carica_minima_accettabile: int):
	new_robs=[]
	for rob in robs:
		if rob.get_charge() > carica_minima_accettabile:
			new_robs.append(rob)
	return new_robs

def piu_scarico(robs: List[MobileRobot]):
	if len(robs) == 0:
		return None
	rob_scelto=robs[0]
	for rob in robs:
		if rob.get_charge() < rob_scelto.get_charge():
			rob_scelto=rob
	return rob_scelto

def scegli_rob(robs: List[MobileRobot],hum: Human, var: int, carica_minima_accettabile: int):
	if var == 1:							# 1 = il piu vicino tra quelli liberi
		return choose_rob(robs,hum)
	if var == 2:							# 2 = il piu carico
		return rob_piu_carico_tra_i_liberi(robs)
	if var == 3:
		return rob_piu_vicino_abbastanza_carico(robs,hum, carica_minima_accettabile)	# 3 = piu vicino con carica sufficente
	if var == 4:
		return piu_scarico(robs_abbastanza_carichi(free_robs(robs) , carica_minima_accettabile))	# 4 = piu scarico con carica sufficente



##################################################################################

def rob_go_to_pos(rob: MobileRobot, dest: Point):
	robot_pos = rob.get_position()
	robot_pt = Point(robot_pos.x, robot_pos.y)
	while robot_pt.distance_from(dest) > 0.8 :
		if rob.execution_finished == True:
			print(' termino il thread del rob')
			return
		traj = nav.plan_traj(rob.get_position(), dest, nav.init_walls())
		str_traj = ''
		for point in traj:
			str_traj += str(point.x) + ',' + str(point.y)
			if not traj.index(point)==len(traj)-1:
				str_traj += '#'
		if len(traj)>0:
			rob_id=rob.rob_id
			filename='MobileRobot'+str(rob_id)
			vrep.set_trajectory(const.VREP_CLIENT_ID, str_traj,filename)
		start_moving_rob(rob,rob.max_speed)
		time.sleep(5)
		robot_pos = rob.get_position()
		robot_pt = Point(robot_pos.x, robot_pos.y)

	print(' rob ', rob.rob_id,' vicino alla destinazione')
	rob.stop_moving()



def rob_go_to_hum(rob: MobileRobot, hum: Human):
	robot_pos = rob.get_position()
	robot_pt = Point(robot_pos.x, robot_pos.y)
	hum_pos = hum.get_position()
	hum_pt = Point(hum_pos.x , hum_pos.y)
	dest= hum_pt
	while robot_pt.distance_from(dest) > 1.8 :
		traj = nav.plan_traj(rob.get_position(), dest, nav.init_walls())
		str_traj = ''
		for point in traj:
			str_traj += str(point.x) + ',' + str(point.y)
			if not traj.index(point)==len(traj)-1:
				str_traj += '#'
		if len(traj)>0:
			rob_id=rob.rob_id
			filename='MobileRobot'+str(rob_id)
			vrep.set_trajectory(const.VREP_CLIENT_ID, str_traj,filename)
		start_moving_rob(rob,rob.max_speed)
		time.sleep(5)
		robot_pos = rob.get_position()
		robot_pt = Point(robot_pos.x, robot_pos.y)
		hum_pos = hum.get_position()
		hum_pt = Point(hum_pos.x , hum_pos.y)
		dest= hum_pt

	print(' rob ', rob.rob_id,' vicino all hum')
	rob.stop_moving()

def robs_go_to( rob1: MobileRobot, rob2: MobileRobot, hum: Human):
	robot_pos = rob1.get_position()
	robot_pt = Point(robot_pos.x, robot_pos.y)
	robot_pos2 = rob2.get_position()
	robot_pt2 = Point(robot_pos2.x, robot_pos2.y)
	hum_pos = hum.get_position()
	hum_pt = Point(hum_pos.x , hum_pos.y)
	dest= hum_pt
	#metto dei time.sleep in modo da non far partire troppi thread
	while robot_pt.distance_from(dest) > 1.8 or robot_pt2.distance_from(dest) > 1.8:
		if robot_pt.distance_from(dest) > 1.8 and robot_pt2.distance_from(dest) > 1.8:
			Thread(target=rob_go_to_hum, args=[rob1,hum]).start()
			Thread(target=rob_go_to_hum, args=[rob2,hum]).start()
			time.sleep(5)
		if robot_pt.distance_from(dest) > 1.8 and not robot_pt2.distance_from(dest) > 1.8:
			Thread(target=rob_go_to_hum, args=[rob1,hum]).start()
			time.sleep(5)
		if not robot_pt.distance_from(dest) > 1.8 and robot_pt2.distance_from(dest) > 1.8:
			Thread(target=rob_go_to_hum, args=[rob2,hum]).start()
			time.sleep(5)
		robot_pos = rob1.get_position()
		robot_pt = Point(robot_pos.x, robot_pos.y)
		robot_pos2 = rob2.get_position()
		robot_pt2 = Point(robot_pos2.x, robot_pos2.y)
		hum_pos = hum.get_position()
		hum_pt = Point(hum_pos.x , hum_pos.y)
		dest= hum_pt


def rob_full_charge(rob: MobileRobot):
	while rob.get_charge() < rob.stop_recharge:
		if rob.execution_finished == True:
			print(' termino il thread del rob')
			return
		pass
	print(' ROB ', rob.rob_id, ' È CARICO')
	rob.free = True



def rob_go_to_recharge_station(rob: MobileRobot):
	rob_go_to_pos(rob,const.VREP_RECH_STATION)

def rob_charged(rob: MobileRobot):
	rob_go_to_recharge_station(rob)
	rob_full_charge(rob)

