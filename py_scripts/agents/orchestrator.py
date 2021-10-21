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
from agents.mobilerobot import * #MobileRobot, start_moving_rob
from agents.human import Human
from agents.mission import *

from threading import Thread

class Operating_Modes(Enum):
	ROBOT_IDLE = 1
	ROBOT_LEAD = 2
	ROBOT_RECH = 3
	ROBOT_CARR = 4				# robot viene usato per trasportare/tenere un oggetto
	ROBOT_FOLL = 5				# foll= follower


class OpChk:
	def __init__(self, t_int: int, t_proc: int, rob: MobileRobot, hum: List[Human], m: Mission):

		# aggiunto io
		self.to_rech=False
		self.orch_id=0
		self.iterazione=1

		# AUTOMATON
		self.LOCATION = "off"
		self.scs = False
		self.fail = False
		self.stop = False

		# PARAMS											# sono tempi di pausa lasciati al programma
		self.t_int = t_int
		self.t_proc = t_proc

		self.currOp = Operating_Modes.ROBOT_IDLE			# parte in idle
		self.currH = 0											# è l indice del vettore umani. inidica l umano a cui ci stiamo riferendo


		self.rob = rob
		self.humans = hum
		self.mission = m
		self.rec_stages = 1					# recipient ha 2 fasi: 1 dove gli si da la posizione della stazione che deve raggiungere ; 2 dove si sta effettivamente ricaricando
		self.stopped_human = False
		if self.mission.p[self.currH] == Pattern.HUM_LEADER:     # all inizio self.currH è =0 perche definito cosi prima
			human_pos = self.humans[self.currH].get_position()   #self.human è l umano di questa classe. get position è	# la funzione della classe human
			human_coord = Point(human_pos.x, human_pos.y)
			self.curr_dest = human_coord
		else:
			self.curr_dest = m.dest[self.currH]

		#   ATTRIBUTO CURR_DEST AGGIUNTO ALLA CLASSE


		# THRESHOLDS
		self.STOP_DIST = 4.0
		self.RESTART_DIST = 2.0

		self.RECHARGE_TH = 10.0
		self.STOP_RECHARGE = 95.0		# ho cambiato a 95 perche a 100 dopo non si scarica piu la batteria.

		# !!!! se cambio qua STOP_RECHARGE cambiare stop_recharge su Class MobileRobot in mobilerobot.py	!!!
		# !!!! se cambio qua RECHARGE_TH cambiare recharge_th su Class MobileRobot in mobilerobot.py	!!!


		self.FAIL_CHARGE = 1.0
		
		self.FAIL_FATIGUE = 0.97	#originale
		#self.FAIL_FATIGUE = 0.097
		self.STOP_FATIGUE = 0.7	#originale
		#self.STOP_FATIGUE = 0.07
		self.RESUME_FATIGUE = 0.3	#originale
		#self.RESUME_FATIGUE = 0.06



	def initialize(self):
		self.stop = False
		self.scs = False
		self.fail = False

	def start(self):
		self.initialize()
		while not self.scs and not self.fail and not self.stop:
			# print(' dentro al while di start')
			#print('sono all inizio del while: self.scs = ', self.scs,' self.fail = ', self.fail,' self.stop = ', self.stop)
			self.LOCATION = 'op'
			time.sleep(self.t_int)
			# t_act >= T_int
			self.check_actions()
			#print('self.scs = ', self.scs,' self.fail = ', self.fail, ' self.stop = ', self.stop)
			self.LOCATION = 'chk'
			time.sleep(self.t_proc)
			# t_act >= T_proc

		self.LOCATION = 'off'
		return

	def check_actions(self):
		# print(' in check actions prima di check scs')
		self.check_scs()
		if self.scs:
			self.rob.free=True
			for hum in self.humans:
				hum.free=True
			return

		# print(' in check actions prima di check fail')
		self.check_fail()
		if self.fail:
			self.rob.free=True
			for hum in self.humans:
				hum.free=True
			return
		#print(' self.cuuOp = ', self.currOp,' pattern corrente = ', self.mission.p[self.currH])
		# print(' in check actions prima degli if')
		# print(' self.currOP = ' , self.currOp)
		if self.currOp == Operating_Modes.ROBOT_IDLE:
			#print('SONO PRIMA DI CHECK START SU ORCH.PY : ROBOT ', self.rob.rob_id,' hum_id = ', self.humans[0].hum_id,' orch_id = ',self.orch_id,' rec_stages = ', self.rec_stages)
			# print(' vorrei mi entrasse qui')
			self.check_start()
		elif self.currOp == Operating_Modes.ROBOT_LEAD or self.currOp == Operating_Modes.ROBOT_CARR:
			#print('SONO PRIMA DI CHECK R MOVE SU ORCH.PY : ROBOT', self.rob.rob_id,' hum_id = ', self.humans[0].hum_id,' orch_id = ',self.orch_id,' rec_stages = ', self.rec_stages)
			self.check_r_move()
		elif self.currOp == Operating_Modes.ROBOT_RECH:
			#print('SONO PRIMA DI CHECK R RECH SU ORCH.PY : ROBOT', self.rob.rob_id,' hum_id = ', self.humans[0].hum_id,' orch_id = ',self.orch_id,' rec_stages = ', self.rec_stages)
			self.check_r_rech()
		elif self.currOp == Operating_Modes.ROBOT_FOLL:
			#print('SONO PRIMA DI CHECK H MOVE SU ORCH.PY : ROBOT', self.rob.rob_id,' hum_id = ', self.humans[0].hum_id,' orch_id = ',self.orch_id,' rec_stages = ', self.rec_stages)
			self.check_h_move()



		if self.currOp == Operating_Modes.ROBOT_CARR and (self.mission.p[self.currH] == Pattern.HUM_RECIPIENT or self.mission.p[self.currH] == Pattern.HUM_INTER) and self.rec_stages == 2:
			curr_human_pos = self.humans[self.currH].get_position()
			self.curr_dest = Point(curr_human_pos.x, curr_human_pos.y)
			self.plan_trajectory()
			start_moving_rob(self.rob,self.rob.max_speed)



		#print(' sono prima di check_service_provided: rob_id = ',self.rob.rob_id,' hum_id = ', self.humans[0].hum_id)
		self.check_service_provided()

	# CHECK IF ALL SERVICES HAVE BEEN PROVIDED, THUS MISSION HAS BEEN SUCCESSFULLY COMPLETED 
	def check_scs(self):
		self.scs = self.mission.get_scs()



	# CHECK IF MISSION HAS FAILED DUE TO BATTERY CHARGE TOO LOW, OR FATIGUE TOO HIGH
	def check_fail(self):
		if vrep.check_connection(const.VREP_CLIENT_ID) not in [0, 1]:
			print('PROBLEMA CONNESSIONE VREP')
			# AGGIUNGO QUI LA PRINT E LA LEVO DAL MAIN
			print('Mission Robot',self.rob.rob_id,' failed.')
			self.mission.fail = True

		if self.rob.get_charge()<=self.FAIL_CHARGE:
			print('PROBLEMA BATTERIA')
			# AGGIUNGO QUI LA PRINT E LA LEVO DAL MAIN
			print('Mission Robot',self.rob.rob_id,' failed.')
			self.mission.fail = True								# questo va bene lasciarlo, perche è quando si scarica del tutto ( vuol dire che quando doveva andare a ricaricarsi non lo ha fatto ed ha continuato a lavorare e a scaricarsi)

		if self.humans[self.currH].get_fatigue()>=self.FAIL_FATIGUE and self.mission.p[self.currH] == Pattern.HUM_FOLLOWER: # ho aggiunto che il pattern devev essere hum follower, senno mi si stoppa anche negli altri pattern dove non c è bisogno
			print('PROBLEMA FATIGUE')
			# AGGIUNGO QUI LA PRINT E LA LEVO DAL MAIN
			print('Mission Robot',self.rob.rob_id,' failed.')
			self.mission.fail = True
		
		self.fail = self.mission.fail

	# CHECK IF CURRENT SERVICE HAS BEEN PROVIDED, THUS THE MISSION CAN MOVE ON
	def check_service_provided(self):
		human_served = False
		# If human is a follower, service is provided when 
		# both human and robot are close to the destination
		if self.mission.p[self.currH] == Pattern.HUM_FOLLOWER:
			dest = self.mission.dest[self.currH]
			#print('DESTINAZIONE CORRENTE',dest,'X=',dest.x,'Y=',dest.y,'ROBOT',self.rob.rob_id,'HUMAN',self.humans[self.currH].hum_id)
			position = self.humans[self.currH].get_position()
			pos = Point(position.x, position.y)
			#print('POSIZIONE',pos,'HUMAN',self.humans[self.currH].hum_id)
			human_robot_dist = self.get_human_robot_dist()
			_min_dist = 1.5
			if position is not None and pos.distance_from(dest) <= _min_dist and human_robot_dist <= _min_dist:
				# if entrambi sono arrivati nell intorno della destinazione, allora ...
				print('HUMAN ',self.humans[self.currH].hum_id,' SERVED.')
				human_served = True

		# If human is leading, service is provided when 
		# human player says so
		elif self.mission.p[self.currH] == Pattern.HUM_LEADER or (self.mission.p[self.currH] == Pattern.HUM_RECIPIENT and self.rec_stages == 2) or (self.mission.p[self.currH] == Pattern.HUM_INTER and self.rec_stages == 2):
			filename = '../scene_logs/humansServed.log'
			f = open(filename, 'r+')
			lines = f.read().splitlines()
			for line in lines:
				if line == 'human'+ str(self.humans[self.currH].hum_id) + 'served':
					print('HUMAN ',self.humans[self.currH].hum_id,' SERVED.')
					human_served = True
					f.truncate(0)		# mi cancella quanto scritto fino ad ora nel file.

					# se piu persone vengono servite contemporaneamente, io ne controllo solo una ma poi me le cancella tutte ??


					# riporto la rec_stage a 1 se nella mission ho poi altri pattern hum_recipient che devono iniziare
					# if self.rec_stages==2:
						#print(' qua lo ricambio oooooooooo')
						#self.rec_stages=1
						# print(' qua non lo ricambio piuuu oooooooooo')
					break
			
		# In any case, if current service has been completed,
		# robot stops and human index increases, and robot goes back
		# to idle if the human that was just served was not the last one
		if human_served:
			self.stop = True
			self.mission.set_served(self.currH)

			# queste 2 righe c erano da prima
			# self.currH+=1								# aumento l indice degli umani
			# self.rec_stages = 1

			# io provo cosi
			if (self.mission.p[self.currH] == Pattern.HUM_RECIPIENT or self.mission.p[self.currH] == Pattern.HUM_INTER) and self.rec_stages==1:		# in questo caso devo aumentare solo la rec_stag
				self.rec_stages=2
				# print(' aumentato il currH anche in passaggio stage 1 --> 2 : vediamo che succede')
				# self.currH+=1
				print(' stage 1--> 2')
				print(' pattern attuale = ' , self.mission.p[self.currH])
			else:
				if self.rec_stages == 2:
					print(' adesso ho cambiato rec stages 2 --> 1 ')
					self.rec_stages=1
				self.currH+=1
				print(' ho aumentato il currH')
			# da qui in poi è come prima
			if self.currH < len(self.humans):
				print(' pattern attuale = ' , self.mission.p[self.currH])
				# se il  pattern è human follower, allora prima il rob deve raggiungere l umano
				if self.mission.p[self.currH]== Pattern.HUM_FOLLOWER:
					print('pattern è hum follower: prima il rob deve raggiungere l umano')
					rob_go_to_hum(self.rob , self.humans[self.currH])
					print('umano raggiunto, ora proseguo con la misison')
				self.currOp = Operating_Modes.ROBOT_IDLE
				print(' self.currOP = ' , self.currOp)
				
	# METHODS TO CHECK WHETHER ACTION CAN START
	def get_human_robot_dist(self):
		human_pos = self.humans[self.currH].get_position()
		robot_pos = self.rob.get_position()
		if human_pos is not None and robot_pos is not None:   # condizioni messe per robustezza
			human_coord = Point(human_pos.x, human_pos.y)
			robot_coord = Point(robot_pos.x, robot_pos.y)
			# robot.coord è un Point ( file coordinates). chiamo la funzione distance_from applicata sul point robot.coord
			human_robot_dist = robot_coord.distance_from(human_coord)
			return human_robot_dist
		else:
			return 1000															# cioe distanza infinita

	# PLAN (AND PUBLISH) TRAJECTORY FROM CURRENT POS TO CURRENT DESTINATION
	def plan_trajectory(self):
		traj = nav.plan_traj(self.rob.get_position(), self.curr_dest, nav.init_walls())
		str_traj = ''
		for point in traj:
			str_traj += str(point.x) + ',' + str(point.y)
			if not traj.index(point)==len(traj)-1:
				str_traj += '#'
		if len(traj)>0:
			# print(str_traj)
			#vrep.set_trajectory(const.VREP_CLIENT_ID, str_traj)

			# PROVA
			rob_id=self.rob.rob_id
			filename='MobileRobot'+str(rob_id)


			vrep.set_trajectory(const.VREP_CLIENT_ID, str_traj,filename)

			# node = 'robTrajPub.py'
			# pool = Pool()
			# pool.starmap(hriros.rosrun_nodes, [(node, [str_traj])])

	# CHECK WHETHER CURRENT ACTION SHOULD START
	def check_start(self):
		# print(' QUI NON MI CI ARRIVA PIU')
		# If battery charge is getting low, 
		# robot switches to charging mode
		if self.rob.get_charge() < self.RECHARGE_TH:
			print('ROBOT',self.rob.rob_id,' CHARGE TOO LOW ')
			self.stop = True
			self.currOp = Operating_Modes.ROBOT_LEAD			#  metto _lead per farlo spostare verso la recharge station
			self.curr_dest = const.VREP_RECH_STATION
			self.plan_trajectory()
		# otherwise the start condition depends on the pattern
		else:
			# print(' QUI NON MI CI ARRIVA PIU')
			start = self.get_start_condition(self.mission.p[self.currH])
			if start:
				print('Action can start for ROBOT',self.rob.rob_id,', setting parameters...')
				self.set_op_params(self.mission.p[self.currH])
				self.plan_trajectory()
				self.stop = True							# stop è riferito allo stato in cui si trova. in questo caso riferito a idle
		return
	
	# GET START CONDITION BASED ON CURRENT PATTERN
	def get_start_condition(self, p: int):	
		human_robot_dist = self.get_human_robot_dist()  
		battery_charge_sufficient = self.rob.get_charge() >= self.RECHARGE_TH
		human_fatigue_low = self.humans[self.currH].get_fatigue() <= self.RESUME_FATIGUE
		# non dovrebbe mai entrare qui perche prima di assegnare le missioni se c è qualche rob scarico lo mando subito in recharge
		# lasciarlo per robustezza
		if not battery_charge_sufficient:
			print('ROBOT',self.rob.rob_id,' CHARGE TOO LOW')
		if human_fatigue_low and self.stopped_human:							# stopped_human parte dal false
			self.stopped_human = False
			print('HUMAN FATIGUE SUFFICIENTLY LOW')
			self.mission.publish_status('DFTG#' + str(self.currH+1))				# se printo, ovviamente l uomo sara indicato nello schermo con (indice vettore +1)

		print('HUMAN-ROBOT DISTANCE: ' + str(human_robot_dist),'ROBOT NUMBER',self.rob.rob_id,' HUMAN ID',self.humans[self.currH].hum_id)
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
				# questa condizione è ridondante, ma ormai è cosi. la cuurent destination in questo caso è infatti proprio la posizione dell uomo
			else:
				return human_robot_dist >= self.RESTART_DIST
				# non dovrebbe mai entrare in questo else. la distanza è quella fittizia a 1000, quindi ritona true

		# If human is a recipient, the action can start if
		# battery charge is sufficient and human fatigue is low
		elif p == Pattern.HUM_RECIPIENT or p == Pattern.HUM_INTER:
			if self.rec_stages == 1:
				return battery_charge_sufficient 
			else:	
				robot_pos = self.rob.get_position()
				robot_pt = Point(robot_pos.x, robot_pos.y)
				return human_robot_dist >= self.RESTART_DIST or robot_pt.distance_from(self.curr_dest) > 2.0	# condizione ridondante, come prima
		else:
			return False

	def set_op_params(self, p: int):
		# Human follower -> destination = prescribed destination
		if p == Pattern.HUM_FOLLOWER:
			self.currOp = Operating_Modes.ROBOT_CARR		# lasciare robot_carr
			self.curr_dest = self.mission.dest[self.currH]
		# Human leader -> destination = current human position
		elif p == Pattern.HUM_LEADER:
			self.currOp = Operating_Modes.ROBOT_FOLL
			curr_human_pos = self.humans[self.currH].get_position()
			self.curr_dest = Point(curr_human_pos.x, curr_human_pos.y)
		# Human recipient -> (stage1) dest = prescribed dest, (stage2) dest = current human position
		elif p == Pattern.HUM_RECIPIENT or p == Pattern.HUM_INTER:
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
		#print(' controllo batteria = ' , self.rob.get_charge())
		human_fatigue_high = self.humans[self.currH].get_fatigue() >= self.STOP_FATIGUE
		if battery_charge_insufficient:
			print('!!ROBOT ', self.rob.rob_id, ' BATTERY CHARGE TOO LOW!!')
		if human_fatigue_high:
			self.mission.publish_status('FTG#' + str(self.currH+1))
			self.stopped_human = True
			print('!!HUMAN ', self.humans[self.currH].hum_id , ' FATIGUE TOO HIGH!!')

		# Human Follower -> action must stop if battery charge is too low, fatigue is too high
		# or human and robot are excessively far from each other
		if p == Pattern.HUM_FOLLOWER:
			return battery_charge_insufficient or human_fatigue_high or human_robot_dist > self.STOP_DIST
		# Human Leader -> action must stop if current destination has been reached or robot is already
		# sufficiently close to human
		elif p == Pattern.HUM_LEADER:
			robot_pos = self.rob.get_position()
			robot_pt = Point(robot_pos.x, robot_pos.y)
			hum_pos = self.humans[self.currH].get_position()
			hum_pt = Point(hum_pos.x, hum_pos.y)
			return human_robot_dist < self.RESTART_DIST or battery_charge_insufficient
			# return robot_pt.distance_from(self.curr_dest) <= 1.0 or human_robot_dist < self.RESTART_DIST or hum_pt.distance_from(self.curr_dest) > 4.0
		# Human Recipient -> action must stop if battery charge is too low or destination has already been reached
		elif p == Pattern.HUM_RECIPIENT or p == Pattern.HUM_INTER:
			robot_pos = self.rob.get_position()
			robot_pt = Point(robot_pos.x, robot_pos.y)
			if self.rec_stages == 1:
				if (p == Pattern.HUM_RECIPIENT and robot_pt.distance_from(self.curr_dest) <= 1.0) or (p == Pattern.HUM_INTER and robot_pt.distance_from(self.curr_dest) <= 1.0 and robot_pt.distance_from(self.humans[self.currH].get_position()) < self.RESTART_DIST):
					print(' pattern attuale = ' , self.mission.p[self.currH])
					self.rec_stages = 2
					# mettere il rob in stato di idle senno poi non riparte nella rec_stages 2
					self.currOp=Operating_Modes.ROBOT_IDLE
				return battery_charge_insufficient or robot_pt.distance_from(self.curr_dest) <= 1.0
			else:
				return robot_pt.distance_from(self.curr_dest) <= 1.0 or human_robot_dist < self.RESTART_DIST or battery_charge_insufficient# anche qua ho aggiunto la condizione su battery charge insufficient senno non mi va in recharge
		else:
			return False
	
	def check_r_move(self):
		self.stop = self.get_stop_condition(self.mission.p[self.currH])
		if self.stop:
			print('Action has to stop, ROBOT',self.rob.rob_id)
			#print('SONO SU CHECK_R_MOVE')

	def check_h_move(self):
		self.stop = self.get_stop_condition(self.mission.p[self.currH])
		if self.stop:
			print('Action has to stop ROBOT',self.rob.rob_id)
			#print('SONO SU CHECK_H_MOVE')
		else:
			human_pos = self.humans[self.currH].get_position()
			hum_pt = Point(human_pos.x, human_pos.y)
			if hum_pt.distance_from(self.curr_dest) > 3.0:
				self.curr_dest = Point(human_pos.x, human_pos.y)
				self.plan_trajectory()									# se si muove, dopo un po devo ricalcolare la traiettoria

	def check_r_rech(self):
		robot_pos = self.rob.get_position()
		robot_pt = Point(robot_pos.x, robot_pos.y)
		# While the robot is in recharging mode, motion must stop if 
		# charging dock has already been reached
		#if robot_pt.distance_from(const.VREP_RECH_STATION) < 0.5 and self.rob.curr_speed > 0.0:
		if robot_pt.distance_from(self.curr_dest) < 0.5 and self.rob.curr_speed > 0.0:
			# print('stop moving su check r reach')
			self.rob.stop_moving()



class Orchestrator:
	def __init__(self, opchk: OpChk, orch_id: int):
		self.opchk = opchk
		self.LOCATION = 'o_init'

		self.rob = opchk.rob
		self.humans = opchk.humans
		self.mission = opchk.mission

		# aggiunto io
		self.opchk.orch_id=orch_id
		self.need_sync=False
		self.sync_in_rec_stage2=False
		self.proceed_in_main=False



	def run_mission(self):

		print('Starting mission ROBOT',self.rob.rob_id,'...',' e hum hum_id = ', self.humans[0].hum_id, ' e orch_id = ', self.opchk.orch_id)

		# questo lo fa solo la prima volta. considero negligible il tempo/scraica della batteria rispetto alla carica totale del rob
		# se il primo pattern è human follower, allora prima il rob deve raggiungere l umano
		if self.opchk.mission.p[0]== Pattern.HUM_FOLLOWER:
			print('pattern è hum follower: prima il rob deve raggiungere l umano')
			rob_go_to_hum(self.opchk.rob , self.opchk.humans[0])
			print('umano raggiunto, ora proseguo con la misison')

		while not self.LOCATION=='o_fail_1' and not self.LOCATION=='o_fail_2' and not self.LOCATION=='o_scs':
			self.opchk.start()
			self.LOCATION = 'idle'

			if self.opchk.stop:										# faccio OPCHK.STOP PER POI FARE .START()
				self.LOCATION = 'r_start'
				start_moving_rob(self.rob,self.rob.max_speed)
				#self.rob.start_moving(self.rob.max_speed)
				self.LOCATION = 'h_start' 
				# no effect, we cannot control the human
				self.LOCATION = 'x_move'
				#print(' ORCH / WHILE / PRIMA DI OPCHK.START')
				self.opchk.start()
				# print(' prima dell if')
				if self.rob.get_charge() > self.opchk.RECHARGE_TH: 	# ho aggiunto la condizione dell if senno sul follower quando mi si allontana non si ferma
				#if self.opchk.mission.p[self.opchk.currH] == Pattern.HUM_FOLLOWER and self.opchk.get_human_robot_dist() > self.opchk.STOP_DIST:
					# print(' stop moving 1 nel while')
					self.rob.stop_moving()
					#continue 	# provo a far ripartire tutto cosi mi guarda le condizioni dall inizio
				if self.rob.get_charge() <= self.opchk.RECHARGE_TH:
					print(' ROB ', self.rob.rob_id, ' NEED SYNCRONIZATION ')
					self.need_sync=True
					if self.opchk.rec_stages==2:
						while self.sync_in_rec_stage2==False:
							pass
						self.sync_in_rec_stage2=False
					# metto lo stato di idle per riiniziare dopo la sync
					self.opchk.currOp=Operating_Modes.ROBOT_IDLE
					Thread(target=rob_charged, args=[self.rob]).start()
					if self.opchk.rec_stages==2:
						self.proceed_in_main=True
					# anche se sto per chiudere la funzione, l orch con i suoi attributi rimangono
					# chiudo la run mission cosi non mi va in collisione con quella nuova che sto aprendo
					return

				self.LOCATION = 'stopping'
				rob_to_rech = self.opchk.currOp == Operating_Modes.ROBOT_LEAD
				# print(' rob to reach = ' , rob_to_rech)
				rob_leading = self.opchk.currOp == Operating_Modes.ROBOT_CARR #or self.opchk.mission.p[self.opchk.currH] == Pattern.HUM_FOLLOWER  # forse se qui guardo al pattern hum follower è meglio
				# print(' rob leading = ' , rob_leading)
				#print('(con il -1) pattern = ', self.mission.p[self.opchk.currH-1], ' pattern normale (senza -1) = ',self.mission.p[self.opchk.currH] )
				#print(' currH-1 =  ',self.opchk.currH-1, ' currH = ',self.opchk.currH )
				hum_recipient = self.mission.p[self.opchk.currH-1] == Pattern.HUM_RECIPIENT or self.mission.p[self.opchk.currH-1] == Pattern.HUM_INTER		# senza -1 mi da errore
				# print('hum recipient = ' , hum_recipient)
				#print(' hum_recipient = ', hum_recipient,' human_id = ', self.opchk.humans[self.opchk.currH].hum_id, ' pattern = ', self.opchk.mission.p[self.opchk.currH],' rec_stage = ', self.opchk.rec_stages) 	# questa print mi da errore
				if self.opchk.stop and not (rob_to_rech or (rob_leading and hum_recipient)) or (self.opchk.mission.p[self.opchk.currH-1] == Pattern.HUM_FOLLOWER and self.opchk.get_human_robot_dist() > self.opchk.STOP_DIST):
					# stopHuman or not, we cannot control the human
					# print(' prima condizione soddifatta')
					# print(' metto currOp a IDLE')
					self.opchk.currOp = Operating_Modes.ROBOT_IDLE
					self.LOCATION = 'o_init'
				elif self.opchk.stop and (rob_to_rech or (rob_leading and hum_recipient)):
					# print(' seconda condizione soddifatta')
					self.LOCATION = 'to_2nd_task'
					if rob_to_rech:	
						# recharging starts when robot is close to the dock
						self.LOCATION = 'starting_2'
						# print('metto currOp a REACH')
						self.opchk.currOp = Operating_Modes.ROBOT_RECH
						self.LOCATION = 'r_rech'
						# print(' sono qua')
						self.opchk.start()
						self.LOCATION = 'stopping_2'
						if self.opchk.stop:
							# print(' if piu interno')
							# print(' metto currOp a IDLE')
							self.opchk.currOp = Operating_Modes.ROBOT_IDLE
							self.LOCATION = 'o_init'							
					elif rob_leading and hum_recipient:
						# cannot control human sync
						self.LOCATION = 'delivering'
						self.LOCATION = 'o_init'
						# questo cambio di operating mode lo aggiungo io
						# print(' aggiunto io')
						# print(' metto currOp a IDLE')
						self.opchk.currOp = Operating_Modes.ROBOT_IDLE
				elif self.opchk.scs:
					self.LOCATION = 'o_scs'
				elif self.opchk.fail:
					self.LOCATION = 'o_fail_2'	
			elif self.opchk.scs:
				print('MISSIONE HA AVUTO SUCCESSO E STO FERMANDO IL ROB ',self.rob.rob_id, ' primo hum_id = ',self.humans[0].hum_id, ' e orch_id = ', self.opchk.orch_id)
				# print(' stop moving 2 nel while dopo che missione ha avuto successo')
				self.rob.stop_moving()
				self.LOCATION = 'o_scs'
			elif self.opchk.fail:
				# print(' stop moving 3 nel while dopo che la missione è fallita')
				self.rob.stop_moving() # nel caso della batteria va bene, perche è quando il rob doveva andare a ricaricarsi ma non lo ha fatto e ora la batteria è proprio a 0
				self.LOCATION = 'o_fail_1'

		#print(' sono in fondo alla run mission con primo hum hum_id = ',self.humans[0].hum_id,' e rob rob_id = ',self.rob.rob_id, ' e orch_id = ', self.opchk.orch_id)
		return

