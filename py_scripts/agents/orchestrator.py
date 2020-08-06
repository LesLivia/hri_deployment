#!/usr/bin/env python
import time
from agents.mobilerobot import MobileRobot

class Orchestrator:
	def __init__(self, t_int: int, t_proc: int, rob: MobileRobot):
		self.t_int = t_int
		self.t_proc = t_proc
		self.currOp = 1
		self.rob = rob

	def run_mission(self):
		cnt = 0
		while cnt < 3:
			print('checking actions...')
			time.sleep(self.t_int)
			cnt +=1
		self.rob.set_sim_running(0)

	def check_actions(self):
		check_scs()
		if scs:
			return

		check_fail()
		if fail:
			return

		#if self.currOp==1:
			#check_start()
		#elif self.currOp==2 or self.currOp==4:
			#check_r_move()
		#elif self.currOp==3:
			#check_r_rech()
		#elif self.currOp==5:
			#check_h_move()

		#check_service_provided()

	def check_scs(self):
		return

	def check_fail(self):
		return

		
