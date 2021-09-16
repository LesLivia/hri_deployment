#!/usr/bin/env python
import time
import os
import sys
import rospy_utils.hrirosnode as hriros
import vrep_utils.vrep as vrep
import agents.navigation as nav
from threading import Thread
from multiprocessing import Pool
from agents.mobilerobot import *   #MobileRobot, start_reading_data_robs, follow_position_rob
from agents.human import Human, start_reading_data, follow_position, follow_fatigue, FatigueProfile  # posso importare tutto ma cosi è piu comodo quando chiamo le funzioni
from agents.coordinates import Point
from agents.orchestrator import Orchestrator, OpChk  # lo stesso
from agents.mission import *  # * vuol dire importo tutto

print('Launching application...')

vrep_sim = vrep.connect(19997)
vrep.start_sim(vrep_sim)


# HUMANS

bill = Human(1, 10, FatigueProfile.YOUNG_SICK, 1)  # giovane ammalato      quale è il parametro da cambiare su vrep per differenziare la fatigue profile dei pazienti ???
carl = Human(2, 10, FatigueProfile.ELDERLY_HEALTHY, 1)  # dottore
david = Human(3, 10, FatigueProfile.YOUNG_HEALTHY, 1)  # giovane sano
eve = Human(4, 6, FatigueProfile.ELDERLY_SICK, 1)	# vecchio ammalato
frank = Human(5, 5, FatigueProfile.ELDERLY_HEALTHY, 1)	# vecchio sano
giuly = Human(6, 4, FatigueProfile.YOUNG_HEALTHY, 1)	# infermiera
hank = Human(7, 15, FatigueProfile.YOUNG_HEALTHY, 1)	#infermiere




humans = [bill, carl, david,eve,frank,giuly,hank]

# ROBOTS

rob1 = MobileRobot(1, 8.0, 5.0)
rob2 = MobileRobot(2, 8.0, 5.0)
rob3 = MobileRobot(3, 8.0, 5.0)

robs=[rob1,rob2,rob3]


# DESTINATIONS

#dest = [Point(24.0, 10.5)]
dest1 = Point(12,9)
dest2 = Point(20,12)
dest3 = Point(24.0, 10.5)
dest4 = Point(27,12)
dest5 = Point(27,9)



dests1 = [dest5]
dests2 = [dest2,dest1,dest2]
dests3 = [dest3,dest1]


# PATTERNS

#per HUM_LEADER passo una destinazione a caso tanto poi viene ricalcolata
# HUM_RECIPIENT deve funzionare in quel modo? poi quando mi arriva alla destinazione finale mi cade in loop

patterns1 = [Pattern.HUM_FOLLOWER]
#patterns1 = [Pattern.HUM_LEADER]
#patterns1 = [Pattern.HUM_RECIPIENT]
patterns2=[Pattern.HUM_FOLLOWER,Pattern.HUM_FOLLOWER,Pattern.HUM_FOLLOWER]
patterns3 = [Pattern.HUM_FOLLOWER,Pattern.HUM_FOLLOWER]



# MISSIONS

mission1 = Mission(patterns1, dests1, 1)
mission2 = Mission(patterns2, dests2, 2)
mission3 = Mission(patterns3, dests3, 3)

total_missions=[mission1,mission2,mission3]
#total_missions=[mission1]
hums_to_serve=[[giuly],[frank,frank,eve],[david,hank]]
#hums_to_serve=[[bill]]


# funzioni poi da trasferire nell orch generale

def all_hums_free(hums: List[Human]):
	for hum in hums:
		if hum.free==False:
			return False
	return True

def need_sync(orch_list: List[Orchestrator]):
	if len(orch_list)==0:
		return False
	for orch in orch_list:
		if orch.need_sync==True:
			return True
	return False






#def rob_go_to_hum()


#def rob_full_charge()

# QUESTO SARA POI L ORCH GENERALE

try:
	# START ROS NODES THAT ACQUIRE DATA FROM SENSORS
	time.sleep(5)
	print('START READING DATA HUMANS AND ROBOTS')
	start_reading_data(humans)
	start_reading_data_robs(robs)
	time.sleep(5)  # in secondi            # con meno secondi a volte non faccio in tempo a leggere dati e mi da IndexError

	print('START MONITORING HUMANS SENSOR DATA LOGS')
	# START MONITORING HUMAN SENSOR DATA LOGS
	thread_h = Thread(target=follow_position, args=[humans])  # thread fa partire un processo che viene eseguito simultaneamente alle successive istruzioni del codice. cosi posso far partire piu processi insieme e monitorarli nello stesso tempo
	thread_h.start()
	thread_h_f = Thread(target=follow_fatigue, args=[humans])
	thread_h_f.start()

	print('START MONITORING ROBOTS SENSOR DATA LOGS')
	time.sleep(5)
	for rob in robs:
		thread_r = Thread(target=follow_position_rob, args=[rob])
		thread_r.start()
		thread_rb = Thread(target=rob.follow_charge)
		thread_rb.start()


	# START MISSION
	time.sleep(5)
	orch_list=[]
	orch_id=1
	threads=[]          # funzione che ordina da fare

		#funzione che mette in ordine gli hums da modificare. devo tener conto il piu importante all interno di hums (?)
		# e poi devo cambiare l ordine anche delle missioni. passerei hums to serve e total mission alla stessa funzione


	#prima fare funzione che controlla se c è qualche rob sotto la soglia di batteria. se si, lo metto occupato e lo mando a recharge




	while len(total_missions)!=0:

# controllo orch per la sync

		if need_sync(orch_list):            # mettere il WHILE !!!!!!!!!!!!!!!!!!!!! e testarlo

			for orc in orch_list:
				if orc.need_sync==True:
					orch_need_sync=orc
					break

			orch_need_sync.selected = True				# mi serve se ci sono piu sync contemporanee, visto che affronto una sync alla volta

			print(' sono prima del while nel main')

			while orch_need_sync.proceed_with_sync==False:
				pass


			orch_need_sync.proceed_with_sync=False

			time.sleep(2)	# cosi sono sicuro che nell orch l altra run mission è stata chiusa

			#print(' SONO DENTRO IL SYNC PRIMA DEL WHILE la prima persona dell orch che ha bisogno il sync è hum_id = ',orch_need_sync.humans[0].hum_id)

			print(' nel main prima di scegliere il rob')

			while len(free_robs(robs))==0:
				pass

			# parametro che mi serve nel orch                       ???????
			#orch_need_sync.end_waiting=True

			for rob in robs:
				if rob.free==True:
					rob_chosen=rob
					break

			print(' free rob id = ', rob_chosen.rob_id)



			# se voglio essere piu efficiente ( cioè non aspettare che rob raggiunga hum se hum_follower prima di continuare nel main)
			# fare funzione che gli passo orch, orch id, rob chosen. quindi faccio partire un thread che esegue questa funzione
			# se hum_follower: metto non free il rob, il orch id lo cambio dopo, ... , faccio rob go to hum e poi thread funzione sync
			# senno: thread funzione sync

			# cosi è piu snello il main e piu efficiente se ci sono piu sync contemporanemente ( senno chi è in coda deve aspettare che
			# il rob dello hum foll raggiunga prima lo hum.

			if orch_need_sync.mission.p[orch_need_sync.opchk.currH] == Pattern.HUM_FOLLOWER:

				rob_go_to_hum(rob_chosen , orch_need_sync.opchk.humans[orch_need_sync.opchk.currH])

			# da aggiungere hum follower oppure levare l if ma satre attento all indentazione sotto che a quel punto non torna

			if orch_need_sync.mission.p[orch_need_sync.opchk.currH] == Pattern.HUM_LEADER or orch_need_sync.mission.p[orch_need_sync.opchk.currH] == Pattern.HUM_RECIPIENT or orch_need_sync.mission.p[orch_need_sync.opchk.currH] == Pattern.HUM_FOLLOWER:

				print(' sto per cambiare i parametri dell orch e avviare run mission')

				orch_need_sync.rob=rob_chosen
				orch_need_sync.opchk.rob=rob_chosen
				orch_need_sync.opchk.orch_id=orch_id

				Thread(target=orch_need_sync.run_mission).start()

				orch_need_sync.need_sync=False
				rob_chosen.free=False
				orch_id+=1



		# if c è bisogno di sync:            se non c è bisogno vado al for hum in hums_to_serve
			# prendo i dati di chi ha bisogno di sync
			# while non si è liberato almeno un rob:
				# pass
		# se arrivo qui vuol dire chre un rob si è liberato
		# se è hum recipient o leader: prendo l orch, cambio il rob che gli è assegnato, faccio orch.run_mission            (1)
			#




		# se è hum_ follower: prima faccio arrivare il rob dove era quello prima, poi faccio come in (1)

		# prima di andare sul for ricontrollo se qualche mission si è chiusa, perche il while need_sync puo durare molto
		for mission in total_missions:
			if mission.get_scs():
				hums_to_serve.pop(total_missions.index(mission))
				total_missions.remove(mission)




# da fare piu prove se poi gli indici degli hum  emission mi si sfasano con la sync perche aggiungo thread e orch_list
# fare prove con synch all inizio in mezzo in fondo e lo stesso con piu sync


		for hums in hums_to_serve:          # hums_to_serve è lista di liste. hums è lista

			if all_hums_free(hums)==False:  # per ogni insieme di umani assegnati ad ogni missione, controllo che questi siano liberi prima di iniziare la missione
				continue                    # il pass non ti fa fare niente. il continue ti fa andare alla prossima iterazione del for

			#print(' il primo hum degli humans liberi analizzati è hum_id = ',hums[0].hum_id)

			# devo cambiare il rob
			rob_chosen=choose_rob(robs,hums[0])     # qua gli devo passare solo l umano del primo pattern perche la mission parte da quello
			if rob_chosen==None:
				break                      # voglio uscire dal ciclo for.
				# nel caso non lo facessi, ci possono essere piu gruppi di persone libere. quando sto analizzando uno meno importante
				# di altri e in quel momento mi si libera il rob, allora il rob mi viene assegnato un gruppo meno importante.
				# invece se ogni volta esco, quando mi si libera il rob, per forza viene assegnato al gruppo piu importante libero in quel momento

			# trovare l indice giusto della missione
			assigned_mission = total_missions[hums_to_serve.index(hums)]

			# creo orch. assegno rob , hums e mission
			opchk = OpChk(0.5, 0.0, rob_chosen, hums, assigned_mission)
			orch = Orchestrator(opchk, orch_id)

			orch_id+=1

			orch_list.append(orch)

			# lancio il thread

			threads.append(Thread(target=orch.run_mission))
			threads[-1].start()

			# metto umani impegnati e rob come occupati

			for hum in hums:
					hum.free=False
			rob_chosen.free=False
			# ho messo nell orch. che una volta che la missione è andata in scs o in fail
			# sia umani coinvolti che rob scelto diventano free

		# missioni e rispetivi hums hanno lo stesso indice.
		# rimuovo missione e hums dai rispettivi vettori.

		# i thread che monitorano i dati degli umani però non li chiudo piu, perche poi possono servire su altre missions

		for mission in total_missions:
			if mission.get_scs():
				hums_to_serve.pop(total_missions.index(mission))
				total_missions.remove(mission)


	# when mission is over (either with success or failure),
	# shut everything down

	for hum in humans:
		hum.set_sim_running(0)
	for rob in robs:
		rob.set_sim_running(0)
	vrep.stop_sim(vrep_sim)


	time.sleep(7)
	print('Execution finished.')
	quit()

except (KeyboardInterrupt, SystemExit):
	for rob in robs:
		rob.set_sim_running(0)
	for hum in humans:
			hum.set_sim_running(0)
	vrep.stop_sim(vrep_sim)
	quit()

# non capisco perche mi va in collisione. il thread vecchio lo lascio aperto cosi il rob va alla recharge station.
# pensavo che una volta che ci è arrivato, metto un while (non si è caricato) pass, e quando si è caricato esco dal while -pass
# e allora metto rob.free= true e chiudo il thread vecchio.

# ma intanto quello nuovo che lancio mi va in conflitto.


######## oppure: quando va a ricaricarsi lancio un thread che lo fa andare alla recharge station, lo carica, poi diventa free.
# subito dopo aver lanciato questo thread pero chiudo il thread vecchio su cui mi trovo e avvio quello nuovo ( devo recuperare le
# altre info tipo il currH.


# per il momento non mi fa mettere il self to rech sull orch e continua ad andarmi in collisione

