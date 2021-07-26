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


class MobileRobot:
	def __init__(self, rob_id, max_speed, max_accel):	# max_accel anche se poi non la usiamo la lasciamo come parametro. la velocita è infatti gestita internamente a vrep
		self.rob_id = rob_id
		self.max_speed = max_speed
		self.max_accel = max_accel
		self.curr_speed = 0.0
		self.sim_running = 0

		self.free=True



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




	# PERCHE FOLLOW CHARGE MI FUNZIONA LO STESSO ANCHE SE NON MODIFICO LA RICERCA DELL ULTIMA RIGA UTILE (LAST_LINR_GIUSTA)
	#			??????????????????????????????????????????????


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

				last_line = lines[-1]			# ogni tanto mi da errore IndexError, devo fare aspettare piu il programma, ma fuori, non in questa funzione
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
		new_dist=rob_coord.distance_from(human_pos)
		if new_dist<dist:
			dist=new_dist
			chosen_rob=rob

	return chosen_rob


def choose_rob(robs,hum):
	return nearest(free_robs(robs),hum)



