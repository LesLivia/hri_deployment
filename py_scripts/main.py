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

vrep_sim = vrep.connect(19997)  # prenderlo per buono
vrep.start_sim(vrep_sim)


# HUMANS

bill = Human(1, 10, FatigueProfile.YOUNG_SICK, 1)  # paziente
carl = Human(2, 10, FatigueProfile.ELDERLY_HEALTHY, 1)  # dottore
david = Human(3, 10, FatigueProfile.YOUNG_SICK, 1)  # paziente.         IN FUTURO DA CAMBIARE I PARAMETRI PER DIFFERENZIARE PAZIENT

humans = [bill, carl, david]

# ROBOTS

rob1 = MobileRobot(1, 8.0, 5.0)
rob2 = MobileRobot(2, 8.0, 5.0)

robs=[rob1,rob2]


# DESTINATIONS

#dest = [Point(24.0, 10.5)]
dest1 = Point(12,9)
dest2 = Point(22,11)
dest3 = Point(24.0, 10.5)

dests1 = [dest1]
dests2 = [dest3]



# PATTERNS

#per HUM_LEADER passo una destinazione a caso tanto poi viene ricalcolata
# HUM_RECIPIENT non funziona che arriva alla destinazione e poi cade in loop
patterns1 = [Pattern.HUM_FOLLOWER]
#patterns1 = [Pattern.HUM_LEADER]
#patterns1 = [Pattern.HUM_RECIPIENT]
#patterns2 = [Pattern.HUM_LEADER]
#patterns2 = [Pattern.HUM_FOLLOWER,Pattern.HUM_LEADER]
patterns2=[Pattern.HUM_FOLLOWER]



# MISSIONS

mission1 = Mission(patterns1, dests1)
mission2 = Mission(patterns2, dests2)

total_missions=[mission1,mission2]
hums_to_serve=[[bill],[david]]


# funzioni poi da trasferire nell orch generale

def all_hums_free(hums: List[Human]):
    for hum in hums:
        if hum.free==False:
            return False
    return True

# def missions_complete(missions: List[Mission]):
#     for mission in missions:
#         if mission.get_scs()==False:
#             return False
#     return True






# QUESTO SARA POI L ORCH GENERALE

try:
    # START ROS NODES THAT ACQUIRE DATA FROM SENSORS
    print('START READING DATA HUMANS AND ROBOTS')
    start_reading_data(humans)
    start_reading_data_robs(robs)
    time.sleep(10)  # in secondi            # con meno secondi a volte non faccio in tempo a leggere dati e mi da IndexError

    print('START MONITORING HUMANS SENSOR DATA LOGS')
    # START MONITORING HUMAN SENSOR DATA LOGS
    thread_h = Thread(target=follow_position, args=[humans])  # thread fa partire un processo che viene eseguito simultaneamente alle successive istruzioni del codice. cosi posso far partire piu processi insieme e monitorarli nello stesso tempo
    thread_h.start()
    thread_h_f = Thread(target=follow_fatigue, args=[humans])
    thread_h_f.start()

    print('START MONITORING ROBOTS SENSOR DATA LOGS')
    for rob in robs:
        thread_r = Thread(target=follow_position_rob, args=[rob])
        thread_r.start()
        thread_rb = Thread(target=rob.follow_charge)
        thread_rb.start()


    # START MISSION
    time.sleep(7)

    #definizione dei thread


    # fictitius_mission=Mission([Pattern.HUM_FOLLOWER],[dest3])
    # opchk = OpChk(0.5, 0.0, rob1, [bill], fictitius_mission)        # li metto cosi non da errore nel for dopo
    # orch = Orchestrator(opchk)

    threads=[]
    # for n in range(len(total_missions)):
	#     threads.append(Thread(target = orch.run_mission))


        # total_missions= mission1 mission2 mission3 mission4
        # hums_to_serve=  [a,b,c]  [a d e]  [ e ]    [ f g  ]

    #while not missions_complete(total_missions):
    while len(total_missions)!=0:
        # per ogni insieme di umani assegnati ad ogni missione, controllo che questi siano liberi prima
        # di iniziare la missione
        #funzione che mette in ordine gli hums da modificare. devo tener conto il piu importante all interno di hums (?)
        # e poi devo cambiare l ordine anche delle missioni. passerei hums to serve e total mission alla stessa funzione

        # adesso far conto che sono tutti ordinati.

        # l ordinamento lo faccio solo una volta prima di entrare nel while



        # faccio partire

        for hums in hums_to_serve:          # hums_to_serve è lista di liste. hums è lista

            if all_hums_free(hums)==False:  #
                continue

            # devo cambiare il rob
            rob_chosen=choose_rob(robs,hums[0])     # qua gli devo passare solo l umano del primo pattern perche la mission parte da quello
            if rob_chosen==None:
                continue


            # trovare l indice giusto della missione
            assigned_mission = total_missions[hums_to_serve.index(hums)]

            # creo orch. assegno rob , hums e mission
            opchk = OpChk(0.5, 0.0, rob_chosen, hums, assigned_mission)
            orch = Orchestrator(opchk)

            # lancio il thread
            #threads[total_missions.index(assigned_mission)].start()

            threads.append(Thread(target=orch.run_mission))
            threads[-1].start()

            # metto umani impegnati e rob come occupati
            for hum in hums:
                    hum.free=False
            rob_chosen.free=False

            # ho messo nell orch. che una volta che la missione è andata in scs o in fail
            # sia umani coinvolti che rob scelto diventano free

            #(potrebbe capitare che dottore serve al secondo pattern, non lo posso mantenere
            # occupato fino ,ad esempio, al decimo). se voglio ci penserò, senno non importa



        # missioni e rispetivi hums hanno lo stesso indice.
        # rimuovo missione e hums dai rispettivi vettori.
        # devo levare anche il rispettivo thread

        for mission in total_missions:
            if mission.get_scs():
                total_missions.remove(mission)
                hums_to_serve.pop(total_missions.index(mission))
                #threads.pop(total_missions.index(mission))
                print('mission removed !!!!!!!!!!!!!!!!!!!!!!!!!')







    # START MISSION
    # time.sleep(7)
    # opchk = OpChk(0.5, 0.0, rob1, [bill],mission)  # comunque qua gli passo humans anche se c è una mission con un pattern e funziona uguale
    # orch = Orchestrator(opchk)
    # thread_m = Thread(target=orch.run_mission)
    # thread_m.start()
    #
    # opchk2 = OpChk(0.5, 0.0, rob2, [david,david], mission2)
    # orch2 = Orchestrator(opchk2)
    # thread_m2 = Thread(target=orch2.run_mission)
    # thread_m2.start()


    # when mission is over (either with success or failure),
    # shut everything down
    for hum in humans:
        hum.set_sim_running(0)
    vrep.stop_sim(vrep_sim)


    time.sleep(7)
    print('Execution finished.')
    quit()

except (KeyboardInterrupt, SystemExit):
    #rob.set_sim_running(0)
    for rob in robs:
        rob.set_sim_running(0)


    #bill.set_sim_running(0)
    for hum in humans:
            hum.set_sim_running(0)



    vrep.stop_sim(vrep_sim)
    quit()
